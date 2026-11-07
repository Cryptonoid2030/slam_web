import {imuIntervals} from './imu-intervals';
import {validateSensorFrame,validateSensorObservation} from './packet';
import type {CameraImageLayout} from './gpu-realsense-packing';
import type {InitialSensorFrame,LandmarkInputs,SensorFrame,SensorObservation} from './types';

interface SlamCameraFrame {
  imageEpoch:number;frameTime:number;
  rgb:Uint8Array;depth:Uint16Array;imageLayout:CameraImageLayout;landmarks?:LandmarkInputs;
  rgbCalibration:number[];depthCalibration:number[];depthUnits:number;
  opticalToBody:number[];cameraOriginBody:number[];
  disparityNoise:number;noiseReferenceFx:number;baseline:number;
}
export interface SlamIntervalFrame extends SlamCameraFrame {intervals:NonNullable<SensorFrame['imuIntervals']>}
export interface SlamInitialFrame extends SlamCameraFrame {imu:SensorFrame['imu']}

/** Transport for RGBDFastSLAMIntervals, independent of the pending Rumoca ABI.
 * Borrow images unchanged; own only small metadata. The compiler must bind the
 * declared strides and raw element types, and carry the complete Modelica State. */
export function slamIntervalFrameFromSensor(frame:SensorFrame):SlamIntervalFrame {
  requireRawImages(frame);
  validateSensorFrame(frame);
  const intervals=imuIntervals(frame).map(({time,dt,imu})=>({time,dt,
    imu:{accel:imu.accel.slice(),gyro:imu.gyro.slice()}}));
  return {...cameraFrame(frame),intervals};
}

/** Initialization observes the time-zero image; it integrates no IMU interval. */
export function slamInitialFrameFromSensor(frame:InitialSensorFrame):SlamInitialFrame {
  if(frame.time!==0||frame.dt!==0||!Array.isArray(frame.imuIntervals)||frame.imuIntervals.length!==0)
    throw new Error('Modelica SLAM initialization requires time zero and no elapsed IMU intervals');
  requireRawImages(frame);
  validateSensorObservation(frame);
  return {...cameraFrame(frame),imu:{accel:frame.imu.accel.slice(),gyro:frame.imu.gyro.slice()}};
}

function requireRawImages(frame:SensorObservation):asserts frame is SensorObservation&{depth:Uint16Array;imageLayout:CameraImageLayout} {
  if(!frame.imageLayout||!(frame.depth instanceof Uint16Array))
    throw new Error('Modelica SLAM requires native RGB8/Z16 camera observations');
}
function cameraFrame(frame:SensorObservation&{depth:Uint16Array;imageLayout:CameraImageLayout}):SlamCameraFrame {
  const k=frame.calibration;
  if(k.depthEncoding!=='axial-z16-le')throw new Error('Modelica SLAM requires axial Z16 depth encoding');
  if(k.depthNoiseDisparityPx===undefined||k.depthNoiseReferenceFx===undefined)
    throw new Error('Modelica SLAM requires explicit depth noise calibration');
  const {color,depth}=frame.imageLayout;
  const dimensions=(image:CameraImageLayout['color']|CameraImageLayout['depth'])=>({
    width:image.width,height:image.height,strideBytes:image.strideBytes,bytes:image.bytes});
  return {imageEpoch:frame.sequence,frameTime:frame.time,rgb:frame.rgb,depth:frame.depth,landmarks:frame.landmarks,
    imageLayout:{color:{...dimensions(color),format:'RGB8'},
      depth:{...dimensions(depth),format:'Z16',unitsMeters:depth.unitsMeters,isBigEndian:false},
      rowOrder:'top-down',aligned:false,clockDomain:'simulation'},
    rgbCalibration:[k.rgbFx,k.rgbFy,k.cx,k.cy],depthCalibration:[k.fx,k.fy,k.cx,k.cy],
    depthUnits:frame.imageLayout.depth.unitsMeters,
    opticalToBody:k.opticalToBody?.slice()??[0,0,1,-1,0,0,0,-1,0],
    cameraOriginBody:k.originFlu?.slice()??[k.forward,0,k.up],
    disparityNoise:k.depthNoiseDisparityPx,noiseReferenceFx:k.depthNoiseReferenceFx,baseline:k.baseline};
}
