export interface Pose { x: number; y: number; z: number; quaternion: number[] }
export interface Truth extends Pose { time: number; velocity: number[]; accel: number[]; gyro: number[]; command?:Command; propellerAngles?:number[] }
export interface Command { forward: number; left: number; up: number; yaw: number }
export interface Calibration { width: number; height: number; fx: number; fy: number; cx: number; cy: number; near: number; far: number; forward: number; up: number; baseline: number; rgbFx: number; rgbFy: number; opticalToBody?:number[];originFlu?:number[];depthNoiseDisparityPx?:number;depthNoiseReferenceFx?:number;depthEncoding?:'axial-f32-le-rgba8'|'axial-z16-le' }
import type {CameraImageLayout} from './gpu-realsense-packing';
export interface SensorObservation {
  sequence: number; time: number; calibration: Calibration;
  rgb: Uint8Array; depth: Float32Array|Uint16Array;
  /** Present for native RGB8/Z16. Absent only for historical RGBA/F32 datasets. */
  imageLayout?:CameraImageLayout;
  imu: { accel: number[]; gyro: number[] };
  /** Phone-selected landmarks for the landmark-input SLAM; rgb/depth then hold only their patches. */
  landmarks?:LandmarkInputs;
}
/** Native landmark inputs: zero-based pixels (x,y interleaved), scores, and the number of valid leading rows. */
export interface LandmarkInputs {pixels:Float64Array;score:Float64Array;count:number}
export interface SensorFrame extends SensorObservation {
  dt:number;
  /** Held measurements over each interval; endpoint samples apply next. */
  imuIntervals?:{time:number;dt:number;imu:{accel:number[];gyro:number[]};/** The device was at rest for the whole frame: the estimator may apply a zero-velocity update. */stationary?:boolean}[];
  capture?:Record<string,unknown>;
}
export interface InitialSensorFrame extends SensorFrame {dt:0;imuIntervals:[]}
export interface Estimate extends Pose {
  points: number[][];confidence: number;features?: number[][];diagnostics?: Record<string,number|string>;
  uncertainty?:{positionCovariance:number[];attitudeCovariance:number[]};
  tracking?:{matches:{current:number[];previous:number[]}[];accepted:boolean;reason:string;referenceSequence?:number};
  loopClosure?:{attempted:boolean;retrievalReason:number;graphReason:number;candidateCount:number;
    candidates:{referenceId:number;currentId:number;score:number;reason:number;verified:boolean;matchedCount?:number;inlierCount?:number;rms?:number}[]};
  poseGraph?:{keyframes:{position:number[];quaternion:number[];id?:number;time?:number;imageEpoch?:number}[];
    edges:{from:number;to:number;kind:'loop'|'odometry';id?:number;similarity?:number}[]};
}
export interface RunRecord { frame: SensorFrame; truth?: Truth; command: Command; estimate: Estimate }
export type Environment = 'city' | 'warehouse' | 'courtyard' | 'tokyo' | 'asset-city' | 'big-city' | 'hallway' | 'hallway-marked' | 'office-loop';
export type SceneDetail='low'|'medium'|'high';
