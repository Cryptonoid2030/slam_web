import {DEFAULT_LIVE_CONFIG,FrameSelector,frameSpacing,MAX_FRAME_INTERVAL_SECONDS,MIN_FRAME_INTERVAL_SECONDS,ImuBuffer,LiveFeed,LiveFrameBuilder,LiveGapError,liveCalibration,parsePhoneCamera,phoneImuToFlu,PHONE_MAGIC,recordFor,resolutionMode,
  type Crop,type LiveConfig,type PhoneCamera} from './live-source';
import {LANDMARK_CAPACITY,LANDMARK_MAGIC,parseLandmarkPacket,selectionThreshold,type LandmarkPacket} from './landmark-source';
import type {Estimate,InitialSensorFrame} from './types';

export interface LivePeers {ingest:number;runner:number;views:number}
export interface LiveStats {
  connected:boolean;armed:boolean;peers:LivePeers;
  /** Seconds of phone time received (since the first frame) and consumed by the SLAM. */
  capturedSeconds:number;processedSeconds:number;queued:number;queuedMB:number;received:number;
  /** Frames per second over the last 5 s: arriving from the phone, and completed by the SLAM. */
  receivedFps:number;processedFps:number;
}

const RATE_WINDOW_MS=5000;
/** Events per second over a sliding window; the window is shorter until it has filled. */
export class RateMeter {
  private times:number[]=[];
  mark(now=performance.now()){this.times.push(now);this.trim(now);}
  reset(){this.times=[];}
  perSecond(now=performance.now()):number{
    this.trim(now);
    if(this.times.length<2)return 0;
    // Intervals between events, measured to the latest of (now, last event) so a stall lowers the rate.
    const span=now-this.times[0];
    return span>0?(this.times.length-1)/(span/1000):0;
  }
  private trim(now:number){while(this.times.length&&now-this.times[0]>RATE_WINDOW_MS)this.times.shift();}
}

/** One phone message awaiting frame selection: a raw camera frame or a landmark packet. */
type Incoming={t:number}&({kind:'camera';camera:PhoneCamera}|{kind:'landmarks';packet:LandmarkPacket});

/** What the page tells the phone to do; the relay replays it to a phone that connects later. */
export function phoneConfig(config:LiveConfig,landmarkLimit:number=LANDMARK_CAPACITY){
  const mode=resolutionMode(config.resolution);
  return {type:'config',mode:config.mode,resolution:config.resolution,width:mode.width,height:mode.height,
    minConfidence:config.minConfidence,nearMeters:config.nearMeters,farMeters:config.farMeters,depthUnitsMeters:config.depthUnitsMeters,
    landmarkCapacity:Math.max(1,Math.min(LANDMARK_CAPACITY,Math.round(landmarkLimit))),fastThreshold:selectionThreshold(mode.width),frameIntervalSeconds:Math.min(MAX_FRAME_INTERVAL_SECONDS,Math.max(MIN_FRAME_INTERVAL_SECONDS,config.frameIntervalSeconds))};
}

/** Decode the phone's JPEG, crop it to the target aspect and scale it into tightly packed RGB8. */
export async function decodeColor(camera:PhoneCamera,crop:Crop,width:number,height:number):Promise<Uint8Array> {
  const bitmap=await createImageBitmap(new Blob([camera.jpeg as BlobPart],{type:'image/jpeg'}),{imageOrientation:'none'} as ImageBitmapOptions);
  try{
    const sx=bitmap.width/camera.header.color.width,sy=bitmap.height/camera.header.color.height;
    const canvas=new OffscreenCanvas(width,height),context=canvas.getContext('2d',{willReadFrequently:true});
    if(!context)throw new Error('2D canvas unavailable for JPEG decoding');
    context.imageSmoothingEnabled=true;context.imageSmoothingQuality='high';
    context.drawImage(bitmap,crop.x*sx,crop.y*sy,crop.w*sx,crop.h*sy,0,0,width,height);
    const rgba=context.getImageData(0,0,width,height).data,rgb=new Uint8Array(width*height*3);
    for(let i=0,j=0;i<rgba.length;i+=4){rgb[j++]=rgba[i];rgb[j++]=rgba[i+1];rgb[j++]=rgba[i+2];}
    return rgb;
  }finally{bitmap.close();}
}

/** Browser side of the live link: receives phone messages from the relay, turns them
 * into lab frames, and publishes the estimator's results back for the Safari mirror. */
export class LiveClient {
  readonly feed=new LiveFeed();
  readonly imu=new ImuBuffer();
  onStats:(stats:LiveStats)=>void=()=>{};
  onStatus:(message:string,error?:boolean)=>void=()=>{};
  private socket?:WebSocket;
  private builder:LiveFrameBuilder;
  private selector:FrameSelector<Incoming>;
  private chain:Promise<void>=Promise.resolve();
  private armed=false;
  private initialWaiters:((frame:InitialSensorFrame)=>void)[]=[];
  private peers:LivePeers={ingest:0,runner:0,views:0};
  private readonly arrivals=new RateMeter();private readonly completions=new RateMeter();private received=0;private lastCaptured=0;private firstTime?:number;private processed=0;
  private stopped=false;
  /** Memory ceiling for queued frames; beyond it the phone is told to stop. */
  maxQueuedBytes=200*1024*1024;
  constructor(readonly url:string,readonly config:LiveConfig=DEFAULT_LIVE_CONFIG){
    this.builder=new LiveFrameBuilder(this.imu,config);
    const spacing=frameSpacing(config.frameIntervalSeconds);this.selector=new FrameSelector<Incoming>(spacing.minGap,spacing.maxGap);
    this.feed.onChange=()=>this.emit();
  }
  connect():Promise<void> {
    return new Promise((resolve,reject)=>{
      const socket=this.socket=new WebSocket(this.url);socket.binaryType='arraybuffer';
      let opened=false;
      socket.onopen=()=>{opened=true;this.send(phoneConfig(this.config,this.builder.landmarkLimit));this.emit();resolve();};
      socket.onerror=()=>{if(!opened)reject(new Error(`Cannot reach the relay at ${this.url}`));};
      socket.onclose=()=>{this.peers={ingest:0,runner:0,views:0};this.emit();if(opened&&!this.stopped)this.onStatus('Relay connection closed',true);};
      socket.onmessage=event=>this.onMessage(event.data);
    });
  }
  close(){this.stopped=true;this.armed=false;this.socket?.close();this.feed.clear();}
  get connected(){return this.socket?.readyState===WebSocket.OPEN;}
  /** Begin accepting camera frames: the next one becomes the time-zero initial frame. */
  arm():Promise<InitialSensorFrame> {
    this.builder.reset();this.selector.reset();this.feed.clear();this.feed.initial=undefined;
    this.firstTime=undefined;this.processed=0;this.received=0;this.arrivals.reset();this.completions.reset();this.lastCaptured=0;this.armed=true;this.send(phoneConfig(this.config,this.builder.landmarkLimit));this.emit();
    return new Promise(resolve=>this.initialWaiters.push(resolve));
  }
  /** Keep only the strongest N landmarks per frame. Applies from the next frame; the phone is told too. */
  setLandmarkLimit(limit:number){
    this.builder.landmarkLimit=Math.max(1,Math.min(LANDMARK_CAPACITY,Math.round(limit)));
    this.send(phoneConfig(this.config,this.builder.landmarkLimit));
  }
  get landmarkLimit(){return this.builder.landmarkLimit;}
  disarm(){this.armed=false;this.emit();}
  private onMessage(data:string|ArrayBuffer){
    try{
      if(typeof data==='string'){
        const message=JSON.parse(data);
        if(message.type==='imu'&&Array.isArray(message.samples)){
          for(const row of message.samples)this.imu.push(phoneImuToFlu(row));
          // Not capturing: keep only the last couple of seconds so an idle connection cannot grow without bound.
          const latest=this.imu.latestTime;if(!this.armed&&latest!==undefined)this.imu.prune(latest-2);
        }
        else if(message.type==='peers'){this.peers={ingest:message.ingest,runner:message.runner,views:message.views};this.emit();}
        else if(message.type==='command')this.onCommand(message);
        return;
      }
      if(!this.armed)return;
      const bytes=new Uint8Array(data);
      const magic=bytes.length>=4?new DataView(bytes.buffer,bytes.byteOffset,4).getUint32(0,true):0;
      let incoming:Incoming;
      if(magic===LANDMARK_MAGIC){
        if(this.config.mode!=='landmarks')throw new Error('The phone sent landmarks, but this page asked for raw frames. Restart the capture.');
        const packet=parseLandmarkPacket(bytes);incoming={kind:'landmarks',packet,t:packet.header.t};
      }else if(magic===PHONE_MAGIC){
        if(this.config.mode!=='raw')throw new Error('The phone is still streaming raw frames. Update the capture app, or switch this page to raw frames.');
        const camera=parsePhoneCamera(bytes);incoming={kind:'camera',camera,t:camera.header.t};
      }else throw new Error('Unrecognised phone message');
      this.received++;this.arrivals.mark();
      const kept=this.selector.offer(incoming);
      if(kept)this.chain=this.chain.then(()=>this.ingest(kept)).catch(error=>this.fail(error));
    }catch(error){this.fail(error);}
  }
  private async ingest(item:Incoming){
    if(!this.armed)return;
    let frame:ReturnType<LiveFrameBuilder['build']>;
    if(item.kind==='landmarks')frame=this.builder.buildLandmarks(item.packet.header,item.packet.landmarks);
    else{
      const {calibration,crop}=liveCalibration(item.camera.header,this.config);
      const rgb=await decodeColor(item.camera,crop,calibration.width,calibration.height);
      if(!this.armed)return;
      frame=this.builder.build(item.camera,rgb);
    }
    if(this.firstTime===undefined)this.firstTime=item.t;
    this.lastCaptured=item.t-this.firstTime;
    if(frame.dt===0){
      this.feed.initial=frame as InitialSensorFrame;
      for(const resolve of this.initialWaiters.splice(0))resolve(frame as InitialSensorFrame);
      this.onStatus('First phone frame received · SLAM initializing');
    }else{
      this.feed.push(recordFor(frame));
      if(this.feed.queuedBytes>this.maxQueuedBytes){
        this.armed=false;this.control('stop','buffer-full');
        this.onStatus(`Capture stopped: the SLAM is ${Math.round(this.feed.queuedBytes/1048576)} MB behind. It will finish the queued frames.`,true);
      }
    }
    this.emit();
  }
  /** Called by the controller after each SLAM frame completes. */
  noteProcessed(seconds:number){this.processed=seconds;this.completions.mark();this.emit();}
  /** Re-emit the statistics so the rates decay when frames stop. */
  refresh(){this.emit();}
  private fail(error:unknown){
    if(error instanceof LiveGapError){this.armed=false;this.control('stop','gap');}
    this.onStatus(error instanceof Error?error.message:String(error),true);
  }
  onCommand:(message:{action?:string})=>void=()=>{};
  control(action:string,reason?:string){this.send({type:'control',action,reason});}
  send(value:unknown){if(this.connected)this.socket!.send(JSON.stringify(value));}
  sendBinary(bytes:ArrayBufferView){if(this.connected)this.socket!.send(bytes);}
  /** Mirror: pose, graph and a decimated map for viewers (Safari on the phone). */
  publishState(frameTime:number,estimate:Estimate,extra:{loops:number;frames:number;rtf:number}){
    const graph=estimate.poseGraph;
    this.send({type:'state',time:frameTime,captured:this.lastCaptured,queued:this.feed.length,
      pose:{x:estimate.x,y:estimate.y,z:estimate.z,q:estimate.quaternion},confidence:estimate.confidence,...extra,
      keyframes:graph?.keyframes.map(k=>[...k.position,...k.quaternion])??[],
      edges:graph?.edges.map(e=>[e.from,e.to,e.kind==='loop'?1:0])??[]});
  }
  /** Binary points message: uint32 'PTS1', uint32 count, then count×(x,y,z) float32 in the map frame. */
  publishPoints(points:number[][],limit=40000){
    const stride=Math.max(1,Math.ceil(points.length/limit)),count=Math.ceil(points.length/stride);
    const bytes=new ArrayBuffer(8+count*12),view=new DataView(bytes),floats=new Float32Array(bytes,8,count*3);
    view.setUint32(0,0x31535450,true);view.setUint32(4,count,true);
    for(let i=0,k=0;i<points.length;i+=stride,k++){floats[3*k]=points[i][0];floats[3*k+1]=points[i][1];floats[3*k+2]=points[i][2];}
    this.sendBinary(new Uint8Array(bytes));
  }
  private emit(){
    this.onStats({connected:this.connected,armed:this.armed,peers:this.peers,capturedSeconds:this.lastCaptured,processedSeconds:this.processed,
      queued:this.feed.length,queuedMB:this.feed.queuedBytes/1048576,received:this.received,receivedFps:this.arrivals.perSecond(),processedFps:this.completions.perSecond()});
  }
}
