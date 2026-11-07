import {slamInitialFrameFromSensor,slamIntervalFrameFromSensor,type SlamInitialFrame,type SlamIntervalFrame} from './modelica-slam-frame';
import type {SlamNativeResult} from './modelica-slam-session';
import type {SlamRequest} from './modelica-slam.worker';
import type {Estimate,InitialSensorFrame,SensorFrame} from './types';
import type {LoopClosureSettings} from './loop-closure-settings';
import type {ModelParameter,ParameterValues} from './model-parameters';
import type {NativeVariant} from './native-assets';

export interface SlamReady {estimate:Estimate;sourceSha256:string;tuningSupported?:boolean;parameters?:ModelParameter[];models:Record<string,{model:string;pages:number}>}
type Pending={resolve:(value:unknown)=>void;reject:(error:Error)=>void;timer:ReturnType<typeof setTimeout>;started:number};

/** One native SLAM worker per vehicle. Requests are strictly sequential: the
 * runtime awaits each camera frame before capturing the next, and images move
 * into the worker by transfer rather than copy. */
export class ModelicaSlamClient {
  private readonly worker=new Worker(new URL('./modelica-slam.worker.ts',import.meta.url),{type:'module'});
  private pending?:Pending;
  private nextId=0;
  private closed=false;
  lastTimings:Record<string,number>={};
  constructor(readonly onProgress:(text:string)=>void=()=>{}){
    this.worker.onmessage=({data})=>{
      if(typeof data?.progress==='string'){this.onProgress(data.progress);return;}
      const pending=this.pending;
      if(!pending||data?.id!==this.nextId-1){this.fail(new Error('Unexpected native SLAM worker response'));return;}
      clearTimeout(pending.timer);this.pending=undefined;
      this.lastTimings={roundTripMs:performance.now()-pending.started,...data.timings};
      if(typeof data.error==='string')pending.reject(new Error(data.error));else pending.resolve(data.result);
    };
    this.worker.onerror=event=>{event.preventDefault();this.fail(new Error(event.message||'Native SLAM worker failed'));};
    this.worker.onmessageerror=()=>this.fail(new Error('Native SLAM worker message could not be decoded'));
  }
  private call<T>(message:SlamRequest extends infer R?R extends SlamRequest?Omit<R,'id'>:never:never,transfer:Transferable[],timeout:number):Promise<T>{
    if(this.closed)return Promise.reject(new Error('Native SLAM worker is closed'));
    if(this.pending)return Promise.reject(new Error('Native SLAM worker is busy'));
    const id=this.nextId++;
    return new Promise<T>((resolve,reject)=>{
      const timer=setTimeout(()=>{this.fail(new Error(`Native SLAM ${message.type} exceeded ${timeout/1000}s; worker stopped`));},timeout);
      this.pending={resolve:value=>resolve(value as T),reject,timer,started:performance.now()};
      try{this.worker.postMessage({...message,id},transfer);}
      catch(error){clearTimeout(timer);this.pending=undefined;reject(error instanceof Error?error:new Error(String(error)));}
    });
  }
  initialize(base:string,frame:InitialSensorFrame,settings?:LoopClosureSettings,parameters?:ParameterValues,variant:NativeVariant='image'){
    // Copy the images: the sensor buffers stay with the viewer and recorder.
    const initial=owned(slamInitialFrameFromSensor(frame));
    return this.call<SlamReady>({type:'init',base,variant,frame:initial,settings,parameters},transferable(initial),600_000);
  }
  reset(frame:InitialSensorFrame,settings?:LoopClosureSettings,parameters?:ParameterValues){
    const initial=owned(slamInitialFrameFromSensor(frame));
    return this.call<Estimate>({type:'reset',frame:initial,settings,parameters},transferable(initial),600_000);
  }
  configure(settings:LoopClosureSettings){return this.call({type:'configure',settings},[],30_000);}
  configureParameters(values:ParameterValues){return this.call({type:'configureParameters',values},[],30_000);}
  step(frame:SensorFrame){
    const interval=owned(slamIntervalFrameFromSensor(frame));
    return this.call<SlamNativeResult>({type:'step',frame:interval},transferable(interval),300_000);
  }
  predict(imageEpoch:number,interval:SlamIntervalFrame['intervals'][number]){
    return this.call({type:'predict',imageEpoch,interval},[],300_000);
  }
  private fail(error:Error){
    if(this.closed)return;this.closed=true;this.worker.terminate();
    if(this.pending){clearTimeout(this.pending.timer);this.pending.reject(error);this.pending=undefined;}
  }
  close(){this.fail(new Error('Native SLAM worker closed'));}
}
function owned<T extends SlamInitialFrame|SlamIntervalFrame>(frame:T):T {return {...frame,rgb:frame.rgb.slice(),depth:frame.depth.slice()};}
function transferable(frame:SlamInitialFrame|SlamIntervalFrame):Transferable[] {
  const buffers=[frame.rgb.buffer,frame.depth.buffer];
  if(buffers.some(buffer=>!(buffer instanceof ArrayBuffer)))throw new Error('Native SLAM transfer requires unshared image buffers');
  return [...new Set(buffers as ArrayBuffer[])];
}
