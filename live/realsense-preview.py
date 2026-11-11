"""Stream a D435 depth preview to the SLAM Lab relay on Windows."""
import argparse
from contextlib import closing
from io import BytesIO
import json
from pathlib import Path
import struct
import sys
import time


def point_packet(vertices, np, world_from_camera=None):
    # RealSense optical axes are right/down/forward. The viewer uses forward/left/up.
    valid = vertices[np.isfinite(vertices).all(axis=1) & (vertices[:, 2] > 0)]
    stride = max(1, (len(valid) + 39_999) // 40_000)
    selected = valid[::stride]
    if world_from_camera is not None:
        selected = selected @ world_from_camera[:3, :3].T + world_from_camera[:3, 3]
    points = selected[:, [2, 0, 1]].astype("<f4")
    points[:, 1:] *= -1
    return struct.pack("<4sI", b"PTS1", len(points)) + points.tobytes()


def camera_packet(color, depth, capture_time, np, image):
    intrinsics = color.profile.as_video_stream_profile().get_intrinsics()
    pixels = np.asanyarray(color.get_data())
    encoded = BytesIO()
    image.fromarray(pixels).save(encoded, format="JPEG", quality=90)
    jpeg = encoded.getvalue()
    meters = np.asanyarray(depth.get_data()).astype("<f4")
    meters *= depth.get_units()
    depth_bytes = meters.tobytes()
    header = json.dumps({
        "type": "camera", "t": capture_time,
        "color": {"width": intrinsics.width, "height": intrinsics.height,
                  "fx": intrinsics.fx, "fy": intrinsics.fy,
                  "cx": intrinsics.ppx, "cy": intrinsics.ppy},
        "depth": {"width": depth.get_width(), "height": depth.get_height()},
        "jpegBytes": len(jpeg), "depthBytes": len(depth_bytes), "confidenceBytes": 0,
    }).encode()
    return struct.pack("<4sI", b"IPH1", len(header)) + header + jpeg + depth_bytes


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--relay", help="Override the relay WebSocket URL")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--slam", action="store_true", help="Send RGB-D to SLAM Lab with iPhone IMU")
    mode.add_argument("--track", action="store_true", help="Estimate trajectory from D435 color and depth")
    parser.add_argument("--fps", type=int, choices=range(1, 31), metavar="1-30", help="Target output FPS (tracking default: 15)")
    parser.add_argument("--time-shift-ms", type=float, default=0, help="Shift D435 frame timestamps for measured IMU synchronization")
    parser.add_argument("--record", type=Path, help="Record the original SDK depth stream to a new .bag file")
    args = parser.parse_args()
    if args.record and (args.record.suffix.lower() != ".bag" or args.record.exists()):
        parser.error("--record must name a new .bag file")

    import numpy as np
    import pyrealsense2 as rs
    import websocket

    config = rs.config()
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    if args.slam or args.track:
        from PIL import Image
    if args.track:
        import open3d as o3d
    if args.slam or args.track:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
        align = rs.align(rs.stream.color)
    if args.record:
        config.enable_record_to_file(str(args.record.resolve()))
    period_ms = 1000 / (args.fps or (15 if args.track else 4 if args.slam else 10))
    pipeline = rs.pipeline()
    cloud = None if args.slam else rs.pointcloud()
    url = args.relay or ("ws://127.0.0.1:8788/d435" if args.slam else "ws://127.0.0.1:8788/runner")

    with closing(websocket.create_connection(url, timeout=5)) as connection:
        pipeline.start(config)
        try:
            mode = "D435 RGB-D to SLAM Lab" if args.slam else "depth preview"
            if args.track:
                mode = "D435 RGB-D odometry"
            previous = None
            world_from_camera = np.eye(4)
            first_timestamp = None
            frame_count = 0
            tracking_lost = False
            print(f"Streaming {mode}. Press Ctrl+C to stop.", flush=True)
            next_frame_ms = None
            while True:
                frames = pipeline.wait_for_frames()
                capture_time = time.time()
                depth = frames.get_depth_frame()
                if not depth:
                    continue
                timestamp = depth.get_timestamp()
                if next_frame_ms is not None and timestamp + 2 < next_frame_ms < timestamp + period_ms * 2:
                    continue
                if args.slam or args.track:
                    aligned = align.process(frames)
                    color = aligned.get_color_frame()
                    depth = aligned.get_depth_frame()
                    if not color or not depth:
                        continue
                if args.slam:
                    packet = camera_packet(color, depth, capture_time + args.time_shift_ms / 1000, np, Image)
                elif args.track and not tracking_lost:
                    intr = color.profile.as_video_stream_profile().get_intrinsics()
                    calibration = o3d.camera.PinholeCameraIntrinsic(
                        intr.width, intr.height, intr.fx, intr.fy, intr.ppx, intr.ppy)
                    rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                        o3d.geometry.Image(np.asanyarray(color.get_data()).copy()),
                        o3d.geometry.Image(np.asanyarray(depth.get_data()).copy()),
                        depth_scale=1.0 / depth.get_units(), depth_trunc=4.0)
                    if previous is not None:
                        success, current_from_previous, _ = o3d.pipelines.odometry.compute_rgbd_odometry(
                            previous, rgbd, calibration)
                        if not success or not np.isfinite(current_from_previous).all():
                            message = "D435 tracking lost. Stop live capture and restart the sender."
                            connection.send(json.dumps({"type": "status", "source": "d435-odometry", "trackingLost": True, "message": message}))
                            print(message, flush=True)
                            tracking_lost = True
                            continue
                        world_from_camera = world_from_camera @ np.linalg.inv(current_from_previous)
                    previous = rgbd
                    if first_timestamp is None:
                        first_timestamp = timestamp
                    frame_count += 1
                    x, y, z = world_from_camera[:3, 3]
                    elapsed = (timestamp - first_timestamp) / 1000
                    connection.send(json.dumps({
                        "type": "state", "source": "d435-odometry", "time": elapsed,
                        "pose": {"x": float(z), "y": float(-x), "z": float(-y), "q": [1, 0, 0, 0]},
                        "keyframes": [], "edges": [], "captured": elapsed, "frames": frame_count,
                        "queued": 0, "loops": 0, "confidence": 0,
                    }))
                    points = cloud.calculate(depth)
                    vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
                    packet = point_packet(vertices, np, world_from_camera)
                else:
                    points = cloud.calculate(depth)
                    vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
                    packet = point_packet(vertices, np)
                connection.send_binary(packet)
                if args.track:
                    connection.send_binary(camera_packet(color, depth, capture_time, np, Image))
                next_frame_ms = max(timestamp, next_frame_ms or timestamp) + period_ms
        finally:
            pipeline.stop()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (ImportError, OSError, RuntimeError) as error:
        sys.exit(str(error))
