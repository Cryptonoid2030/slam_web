import type {Calibration,InitialSensorFrame,RunRecord,SensorFrame} from './types';
import {D435_PAIRED_RESOLUTIONS} from './camera-resolutions.mjs';
import {validateSensorFrame,validateSensorObservation} from './packet';
import {LANDMARK_CAPACITY,sparseFrame,type LandmarkHeader,type LandmarkSet} from './landmark-source';
import type {LandmarkInputs} from './types';

/** Pure conversion layer between the iPhone capture app and the lab's RGB-D/IMU
 * frame contract. No DOM or network access: everything here is unit tested.
 *
 * Phone conventions (documented once, applied once, here):
 *  - Timestamps: seconds on the ARKit/CoreMotion clock (system uptime).
 *  - IMU rows `[t, ax, ay, az, gx, gy, gz]` are CoreMotion values in the device
 *    frame (x right, y up the screen, z out of the screen): acceleration is
 *    userAcceleration+gravity in g (face up at rest reads z = -1), rotation rate
 *    in rad/s.
 *  - Depth is ARKit sceneDepth: float32 meters along the optical axis, sampled
 *    on a grid covering the same field of view as the color image; confidence is
 *    0 low / 1 medium / 2 high.
 *
 * Lab conventions: body is forward-left-up with specific-force accelerometer
 * (+9.81 up at rest). The body frame is chosen to coincide with the rear camera:
 * forward = -z_device, left = +y_device, up = +x_device (phone held landscape-
 * right, which is the orientation of ARKit's capturedImage). */
export const STANDARD_GRAVITY=9.80665;
/** 'IPH1' as a little-endian uint32. */
export const PHONE_MAGIC=0x31485049;
export const MAX_FRAME_GAP_SECONDS=1;

export interface PhoneCameraHeader {
  type:'camera';t:number;
  color:{width:number;height:number;fx:number;fy:number;cx:number;cy:number};
  depth:{width:number;height:number};
  jpegBytes:number;depthBytes:number;confidenceBytes:number;
  /** ARKit camera-to-world, column-major. Carried for an optional odometry prior; unused by the estimator. */
  pose?:number[];
}
export interface PhoneCamera {header:PhoneCameraHeader;jpeg:Uint8Array;depth:Float32Array;confidence:Uint8Array}
export type ResolutionId=typeof D435_PAIRED_RESOLUTIONS[number]['id'];
export interface LiveConfig {
  /** `landmarks`: the phone selects corners and sends only those (IPL1). `raw`: it sends JPEG + depth (IPH1). */
  mode:'landmarks'|'raw';
  resolution:ResolutionId;
  /** Depth with ARKit confidence below this reads as no return (0..2). */
  minConfidence:0|1|2;
  nearMeters:number;farMeters:number;depthUnitsMeters:number;
  /** Longest held IMU interval; each interval costs one estimator evaluation. */
  intervalSeconds:number;
  /** Target spacing of camera frames the phone sends (landmark mode) and the page keeps. */
  frameIntervalSeconds:number;
  /** Apply a zero-velocity update while the phone is at rest (needs the landmark programs built with it). */
  zeroVelocity:boolean;
  /** Depth noise model inputs the native estimator requires (a stereo model; LiDAR is mapped onto it). */
  baselineMeters:number;depthNoiseDisparityPx:number;
}
export const DEFAULT_LIVE_CONFIG:LiveConfig=Object.freeze({
  mode:'landmarks',resolution:'640x480',minConfidence:1,nearMeters:.2,farMeters:5,depthUnitsMeters:.001,
  intervalSeconds:.2,frameIntervalSeconds:.7,zeroVelocity:true,baselineMeters:.05,depthNoiseDisparityPx:.1,
});

export const MIN_FRAME_INTERVAL_SECONDS=.2,MAX_FRAME_INTERVAL_SECONDS=.9;
/** Frame-selection window for a target interval: the default .7 s gives the original .6-.95 s window.
 * Faster capture means more estimator work per second of motion, so the SLAM falls further behind. */
export function frameSpacing(interval:number):{minGap:number;maxGap:number} {
  if(!Number.isFinite(interval))throw new Error('Frame interval must be a number');
  const target=Math.min(MAX_FRAME_INTERVAL_SECONDS,Math.max(MIN_FRAME_INTERVAL_SECONDS,interval));
  return {minGap:target*.85,maxGap:Math.min(.95,target*1.35)};
}

export class LiveGapError extends Error {constructor(message:string){super(message);this.name='LiveGapError';}}

/** Wire format for one camera message: magic, JSON header length, JSON header,
 * then JPEG bytes, float32 LE depth, uint8 confidence. IMU travels as JSON text
 * `{"type":"imu","samples":[[t,ax,ay,az,gx,gy,gz],…]}`. */
export function parsePhoneCamera(bytes:Uint8Array):PhoneCamera {
  if(bytes.length<8)throw new Error('Truncated phone camera message');
  const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
  if(view.getUint32(0,true)!==PHONE_MAGIC)throw new Error('Unsupported phone camera message');
  const headerLength=view.getUint32(4,true);
  if(8+headerLength>bytes.length)throw new Error('Truncated phone camera header');
  const header=JSON.parse(new TextDecoder().decode(bytes.subarray(8,8+headerLength))) as PhoneCameraHeader;
  if(header?.type!=='camera'||!Number.isFinite(header.t)||header.t<0)throw new Error('Invalid phone camera header');
  const {color,depth}=header;
  for(const value of [color?.width,color?.height,depth?.width,depth?.height])
    if(!Number.isInteger(value)||value<=0||value>8192)throw new Error('Invalid phone camera dimensions');
  for(const value of [color.fx,color.fy,color.cx,color.cy])if(!Number.isFinite(value))throw new Error('Invalid phone intrinsics');
  if(color.fx<=0||color.fy<=0)throw new Error('Invalid phone intrinsics');
  const samples=depth.width*depth.height;
  if(header.depthBytes!==samples*4||(header.confidenceBytes!==0&&header.confidenceBytes!==samples))throw new Error('Phone depth/confidence size mismatch');
  if(8+headerLength+header.jpegBytes+header.depthBytes+header.confidenceBytes!==bytes.length)throw new Error('Phone camera payload length mismatch');
  const start=8+headerLength;
  const depthStart=start+header.jpegBytes;
  const depthCopy=new Uint8Array(header.depthBytes);depthCopy.set(bytes.subarray(depthStart,depthStart+header.depthBytes));
  return {header,jpeg:bytes.subarray(start,depthStart),depth:new Float32Array(depthCopy.buffer),
    confidence:bytes.slice(depthStart+header.depthBytes)};
}

/** Limits for calling a device "at rest": it must be on a table or stand, not merely held steadily.
 * Hand tremor exceeds these, so a held phone is never declared at rest. */
export const REST_LIMITS=Object.freeze({minSamples:8,gyroMean:.02,gyroPeak:.06,accelStd:.05,gravityTolerance:.3,gravity:9.80665});
/** True when every sample in the window is consistent with a motionless device. */
export function isAtRest(window:readonly ImuSample[]):boolean {
  const n=window.length,L=REST_LIMITS;
  if(n<L.minSamples)return false;
  const mean=[0,0,0],gyro=[0,0,0];let peak=0;
  for(const s of window){for(let i=0;i<3;i++){mean[i]+=s.accel[i]/n;gyro[i]+=s.gyro[i]/n;}peak=Math.max(peak,Math.hypot(...s.gyro));}
  let variance=0;for(const s of window)for(let i=0;i<3;i++)variance+=(s.accel[i]-mean[i])**2/n;
  return Math.hypot(...gyro)<=L.gyroMean&&peak<=L.gyroPeak&&Math.sqrt(variance)<=L.accelStd&&Math.abs(Math.hypot(...mean)-L.gravity)<=L.gravityTolerance;
}
export interface ImuSample {t:number;accel:[number,number,number];gyro:[number,number,number]}
/** Device-frame CoreMotion row to forward-left-up specific force (m/s²) and rate (rad/s). */
export function phoneImuToFlu(row:readonly number[]):ImuSample {
  if(row.length!==7||!row.every(Number.isFinite))throw new Error('Invalid phone IMU row');
  const [t,ax,ay,az,gx,gy,gz]=row;
  // specific force in device axes is -g·a; FLU = (-z, +y, +x)
  return {t,accel:[STANDARD_GRAVITY*az,-STANDARD_GRAVITY*ay,-STANDARD_GRAVITY*ax],gyro:[-gz,gy,gx]};
}

export interface ImuInterval {time:number;dt:number;imu:{accel:number[];gyro:number[]}}
/** Time-ordered IMU history with interval builders for the lab's held-measurement contract. */
export class ImuBuffer {
  private samples:ImuSample[]=[];
  get latestTime(){return this.samples.at(-1)?.t;}
  get size(){return this.samples.length;}
  push(sample:ImuSample){
    const last=this.samples.at(-1);
    if(last&&sample.t<=last.t)return false; // drop duplicates and reordered samples
    this.samples.push(sample);return true;
  }
  /** Most recent measurement at or before t (or the first one when none precedes it). */
  heldAt(t:number):ImuSample {
    if(!this.samples.length)throw new Error('No IMU samples received');
    let lo=0,hi=this.samples.length-1;
    if(this.samples[0].t>t)return this.samples[0];
    while(lo<hi){const mid=(lo+hi+1)>>1;if(this.samples[mid].t<=t)lo=mid;else hi=mid-1;}
    return this.samples[lo];
  }
  /** Equal intervals of at most maxDt covering (t0,t1]; each holds the mean of its samples. Times are relative to epoch. */
  intervals(t0:number,t1:number,epoch:number,maxDt:number,detectRest=false):ImuInterval[] {
    if(!(t1>t0)||!(maxDt>0))throw new Error('Invalid IMU interval span');
    if(!this.samples.length)throw new Error('No IMU samples received');
    // The lab rejects any interval above maxDt, so leave headroom for float rounding at exact multiples.
    const count=Math.max(1,Math.ceil((t1-t0)/(maxDt*(1-1e-6))));
    if(count>36)throw new Error('IMU span needs more than 36 intervals');
    const dt=(t1-t0)/count,out:ImuInterval[]=[];
    const stationary=detectRest&&isAtRest(this.samples.filter(s=>s.t>t0&&s.t<=t1));
    for(let k=0;k<count;k++){
      const a=t0+k*dt,b=k===count-1?t1:t0+(k+1)*dt;
      const inside=this.samples.filter(s=>s.t>a&&s.t<=b);
      const accel=[0,0,0],gyro=[0,0,0];
      if(inside.length){
        for(const s of inside)for(let i=0;i<3;i++){accel[i]+=s.accel[i];gyro[i]+=s.gyro[i];}
        for(let i=0;i<3;i++){accel[i]/=inside.length;gyro[i]/=inside.length;}
      }else{const held=this.heldAt(a);accel[0]=held.accel[0];accel[1]=held.accel[1];accel[2]=held.accel[2];gyro[0]=held.gyro[0];gyro[1]=held.gyro[1];gyro[2]=held.gyro[2];}
      out.push({time:(k===count-1?t1:t0+(k+1)*dt)-epoch,dt:b-a,imu:{accel,gyro},...(stationary?{stationary:true}:{})});
    }
    return out;
  }
  /** Forget samples older than t, keeping one earlier sample so heldAt() still works. */
  prune(t:number){
    let drop=0;while(drop+1<this.samples.length&&this.samples[drop+1].t<=t)drop++;
    if(drop>0)this.samples.splice(0,drop);
  }
}

/** Chooses which camera frames the estimator receives. The lab rejects frame gaps
 * above one second and cannot skip IMU time, so frames are kept 0.6–0.95 s apart:
 * the latest arrival inside the window is committed once the next one passes it. */
export class FrameSelector<T extends {t:number}> {
  private lastKept?:number;private candidate?:T;
  constructor(private readonly minGap=.6,private readonly maxGap=.95){
    if(!(minGap>0)||!(maxGap>minGap)||maxGap>=MAX_FRAME_GAP_SECONDS)throw new Error('Invalid frame spacing');
  }
  offer(item:T):T|undefined {
    if(this.lastKept===undefined){this.lastKept=item.t;return item;}
    if(item.t<=this.lastKept)return undefined;
    let gap=item.t-this.lastKept,committed:T|undefined;
    if(gap>this.maxGap){
      if(!this.candidate){
        // No earlier arrival to fall back on: the item itself is usable only while it stays under the hard limit.
        if(gap<MAX_FRAME_GAP_SECONDS-.01){this.lastKept=item.t;return item;}
        throw new LiveGapError(`Camera frames are ${gap.toFixed(2)} s apart; the estimator accepts at most ${MAX_FRAME_GAP_SECONDS} s. Reset and restart the capture.`);
      }
      committed=this.candidate;this.lastKept=committed.t;this.candidate=undefined;gap=item.t-this.lastKept;
      if(gap>this.maxGap)throw new LiveGapError('Camera frames arrived too slowly to keep a continuous chain. Reset and restart the capture.');
    }
    if(gap>=this.minGap)this.candidate=item;
    return committed;
  }
  reset(){this.lastKept=undefined;this.candidate=undefined;}
}

export function resolutionMode(id:ResolutionId){
  const mode=D435_PAIRED_RESOLUTIONS.find(m=>m.id===id);
  if(!mode)throw new Error(`Unsupported camera resolution ${id}`);
  return mode;
}
export interface Crop {x:number;y:number;w:number;h:number}
/** Largest centered rectangle of the source with the destination's aspect ratio. */
export function cropFor(srcW:number,srcH:number,dstW:number,dstH:number):Crop {
  const srcAspect=srcW/srcH,dstAspect=dstW/dstH;
  if(srcAspect>dstAspect){const w=srcH*dstAspect;return {x:(srcW-w)/2,y:0,w,h:srcH};}
  const h=srcW/dstAspect;return {x:0,y:(srcH-h)/2,w:srcW,h};
}
/** Camera calibration for the cropped, resampled image. Depth is resampled onto the color grid, so both share intrinsics. */
export function liveCalibration(header:PhoneCameraHeader,config:LiveConfig=DEFAULT_LIVE_CONFIG):{calibration:Calibration;crop:Crop} {
  const mode=resolutionMode(config.resolution),{color}=header;
  const crop=cropFor(color.width,color.height,mode.width,mode.height),scaleX=mode.width/crop.w,scaleY=mode.height/crop.h;
  const fx=color.fx*scaleX,fy=color.fy*scaleY;
  return {crop,calibration:{width:mode.width,height:mode.height,fx,fy,cx:(color.cx-crop.x)*scaleX,cy:(color.cy-crop.y)*scaleY,
    near:config.nearMeters,far:config.farMeters,forward:0,up:0,baseline:config.baselineMeters,rgbFx:fx,rgbFy:fy,
    opticalToBody:[0,0,1,-1,0,0,0,-1,0],originFlu:[0,0,0],
    depthNoiseDisparityPx:config.depthNoiseDisparityPx,depthNoiseReferenceFx:fx,depthEncoding:'axial-z16-le'}};
}
/** Nearest-neighbour resample (never blends across depth edges) into Z16, zero where there is no trustworthy return. */
export function depthToZ16(camera:PhoneCamera,crop:Crop,width:number,height:number,config:LiveConfig=DEFAULT_LIVE_CONFIG):Uint16Array {
  const {color,depth}=camera.header,out=new Uint16Array(width*height),hasConfidence=camera.confidence.length>0;
  const maxCode=65535;
  for(let y=0;y<height;y++){
    const v=crop.y+(y+.5)*crop.h/height,row=Math.min(depth.height-1,Math.max(0,Math.floor(v*depth.height/color.height)));
    for(let x=0;x<width;x++){
      const u=crop.x+(x+.5)*crop.w/width,col=Math.min(depth.width-1,Math.max(0,Math.floor(u*depth.width/color.width)));
      const index=row*depth.width+col,z=camera.depth[index];
      if(hasConfidence&&camera.confidence[index]<config.minConfidence)continue;
      if(!Number.isFinite(z)||z<config.nearMeters||z>config.farMeters)continue;
      out[y*width+x]=Math.min(maxCode,Math.round(z/config.depthUnitsMeters));
    }
  }
  return out;
}

const EMPTY_ESTIMATE=()=>({x:0,y:0,z:0,quaternion:[1,0,0,0],confidence:0,points:[]});
/** Turns selected phone camera frames plus buffered IMU into lab sensor frames. The first becomes the time-zero initial frame. */
export class LiveFrameBuilder {
  private epoch?:number;private previous=0;private sequence=0;
  /** Most landmarks kept per frame. Landmarks arrive strongest first, so this keeps the top N by score. */
  landmarkLimit=LANDMARK_CAPACITY;
  constructor(readonly imu:ImuBuffer,readonly config:LiveConfig=DEFAULT_LIVE_CONFIG){}
  get started(){return this.epoch!==undefined;}
  /** `rgb` is RGB8 already cropped/scaled to the target resolution (see liveCalibration). */
  build(camera:PhoneCamera,rgb:Uint8Array):InitialSensorFrame|SensorFrame {
    const {calibration,crop}=liveCalibration(camera.header,this.config);
    const {width,height}=calibration;
    if(rgb.length!==width*height*3)throw new Error('Color image does not match the live resolution');
    return this.assemble(camera.header.t,calibration,rgb,depthToZ16(camera,crop,width,height,this.config));
  }
  /** A frame from phone-selected landmarks: only their 7x7 patches and depth samples are painted, the rest is zero. */
  buildLandmarks(header:LandmarkHeader,set:LandmarkSet):InitialSensorFrame|SensorFrame {
    const stub:PhoneCameraHeader={type:'camera',t:header.t,color:header.color,depth:{width:1,height:1},jpegBytes:0,depthBytes:0,confidenceBytes:0};
    const {calibration}=liveCalibration(stub,this.config);
    const {width,height}=calibration;
    if(header.target.width!==width||header.target.height!==height)
      throw new Error(`The phone selected landmarks on ${header.target.width}x${header.target.height}, but the SLAM runs at ${width}x${height}`);
    const sparse=sparseFrame(set.count>this.landmarkLimit?{...set,count:this.landmarkLimit}:set,width,height);
    return this.assemble(header.t,calibration,sparse.rgb,sparse.depth,{pixels:sparse.pixels,score:sparse.score,count:sparse.count});
  }
  private assemble(t:number,calibration:Calibration,rgb:Uint8Array,depth:Uint16Array,landmarks?:LandmarkInputs):InitialSensorFrame|SensorFrame {
    const {width,height}=calibration;
    const imageLayout={color:{width,height,strideBytes:width*3,bytes:width*3*height,format:'RGB8' as const},
      depth:{width,height,strideBytes:width*2,bytes:width*2*height,format:'Z16' as const,unitsMeters:this.config.depthUnitsMeters,isBigEndian:false as const},
      rowOrder:'top-down' as const,aligned:false as const,clockDomain:'simulation' as const};
    const base={rgb,depth,imageLayout,calibration,sequence:this.sequence,...(landmarks?{landmarks}:{})};
    if(this.epoch===undefined){
      const held=this.imu.heldAt(t),frame:InitialSensorFrame={...base,time:0,dt:0,imu:{accel:[...held.accel],gyro:[...held.gyro]},imuIntervals:[]};
      validateSensorObservation(frame);
      this.epoch=t;this.previous=t;this.sequence++;return frame;
    }
    if(t<=this.previous)throw new LiveGapError('Camera timestamps are not increasing');
    const intervals=this.imu.intervals(this.previous,t,this.epoch,this.config.intervalSeconds,this.config.zeroVelocity);
    const last=intervals.at(-1)!,frame:SensorFrame={...base,time:t-this.epoch,dt:t-this.previous,imu:{accel:[...last.imu.accel],gyro:[...last.imu.gyro]},imuIntervals:intervals};
    validateSensorFrame(frame);
    this.previous=t;this.sequence++;this.imu.prune(this.previous);return frame;
  }
  reset(){this.epoch=undefined;this.previous=0;this.sequence=0;}
}
export function recordFor(frame:SensorFrame):RunRecord {
  return {frame,command:{forward:0,left:0,up:0,yaw:0},estimate:EMPTY_ESTIMATE()};
}

/** Pull-based queue between the network and the estimator. A frame is never
 * dropped (that would skip IMU time); instead the queue reports its byte size so
 * the controller can stop the capture when memory is the limit. */
export class LiveFeed {
  private items:RunRecord[]=[];private waiting?:(record:RunRecord|undefined)=>void;private bytes=0;
  initial?:InitialSensorFrame;onChange:()=>void=()=>{};
  get length(){return this.items.length;}
  get queuedBytes(){return this.bytes;}
  push(record:RunRecord){
    if(this.waiting){const resolve=this.waiting;this.waiting=undefined;resolve(record);}
    else{this.items.push(record);this.bytes+=frameBytes(record.frame);}
    this.onChange();
  }
  next():Promise<RunRecord|undefined> {
    const item=this.items.shift();
    if(item){this.bytes-=frameBytes(item.frame);this.onChange();return Promise.resolve(item);}
    // A second concurrent consumer supersedes the first, which is released with no frame.
    this.waiting?.(undefined);
    return new Promise(resolve=>{this.waiting=resolve;});
  }
  /** Releases a pending next() with no frame (pause, reset, disconnect). */
  wake(){const resolve=this.waiting;this.waiting=undefined;resolve?.(undefined);}
  clear(){this.items=[];this.bytes=0;this.wake();this.onChange();}
}
function frameBytes(frame:SensorFrame){return frame.rgb.byteLength+frame.depth.byteLength;}
