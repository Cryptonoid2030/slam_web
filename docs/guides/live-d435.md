# Live RealSense D435 preview on Windows

[Use the D435 and iPhone IMU together in the main SLAM Lab app](live-d435-slam.md).

## World View and Trajectory in SLAM Lab

With the relay running, install Open3D and start the sender in tracking mode:

```powershell
py -3.12 -m pip install open3d
py -3.12 live/realsense-preview.py --track
```

Open [SLAM Lab](http://127.0.0.1:5174/?source=d435-preview&live=ws://127.0.0.1:8788/runner).
In **Live sensors**, choose **RealSense D435**, leave **Use iPhone IMU** off and click
**Start live capture**. World View shows the current depth cloud and Trajectory shows the
path estimated from successive color and depth frames. Move slowly through a scene with
visible texture and geometry. This mode estimates incremental motion and can drift.

To switch to the iPhone IMU, stop live capture and stop the sender with Ctrl+C. Check
**Use iPhone IMU**, run the sender again with `--slam` in place of `--track`, and follow
the [phone connection steps](live-d435-slam.md).

## Standalone depth preview

Connect the D435 with a USB 3 data cable. This path renders one depth frame at a time in the
SLAM Lab relay viewer. Points are in camera coordinates. An accumulated world map needs a
pose for every frame, which the plain D435 does not provide. Use a dedicated relay on port 8788
so the D435 sender and the browser's iPhone SLAM runner never compete for the same connection.

From the root of the fork, with Node 22 or newer and 64-bit Python 3.12, run:

```powershell
npm ci --ignore-scripts
py -3.12 -m pip install pyrealsense2 numpy websocket-client
npm run live -- --host=127.0.0.1 --port=8788
```

In a second terminal, from the same directory:

```powershell
py -3.12 live/realsense-preview.py --record d435-depth.bag
```

Open [D435 depth preview](http://127.0.0.1:8788/view.html?source=d435). Press Ctrl+C in the
sender terminal to finish the recording. Omit `--record` for preview only. Choose a new file
name for each recording. The SDK records its original Z16 depth stream and metadata before
the sender converts any points. Z16 is the camera's computed stereo depth. The preview
publishes up to 10 frames per second and samples up to 40,000 points per frame for display.
