/// <reference lib="webworker" />
import {ModelicaSlamSession} from './modelica-slam-session';
import {fetchNativeAsset,fetchNativeManifest} from './slam-native-program';
import type {SlamInitialFrame,SlamIntervalFrame} from './modelica-slam-frame';
import type {LoopClosureSettings} from './loop-closure-settings';
import type {ParameterValues} from './model-parameters';
import type {NativeVariant} from './native-assets';

export type SlamRequest={id:number}&({type:'init';base:string;variant?:NativeVariant;frame:SlamInitialFrame;settings?:LoopClosureSettings;parameters?:ParameterValues}|{type:'reset';frame:SlamInitialFrame;settings?:LoopClosureSettings;parameters?:ParameterValues}|{type:'configureParameters';values:ParameterValues}|{type:'configure';settings:LoopClosureSettings}|{type:'step';frame:SlamIntervalFrame}|{type:'predict';imageEpoch:number;interval:SlamIntervalFrame['intervals'][number]});
let session:ModelicaSlamSession|undefined;
async function execute(message:SlamRequest){
  if(message.type==='init'){
    const base=new URL(message.base,self.location.href);
    if(base.origin!==self.location.origin)throw new Error('Native SLAM assets must be same origin');
    const {width,height}=message.frame.imageLayout.color;
    const variant=message.variant??'image',manifest=await fetchNativeManifest(base,`${width}x${height}`,variant);
    session=await ModelicaSlamSession.create(manifest,(file,encoding)=>fetchNativeAsset(base,file,encoding,variant),message.frame,
      progress=>self.postMessage({progress}),message.settings,message.parameters);
    return {estimate:session.initialEstimate,tuningSupported:session.tuningSupported,parameters:manifest.models.step.parameters,sourceSha256:manifest.sourceSha256,models:Object.fromEntries(Object.entries(manifest.models).map(([key,layout])=>[key,{model:layout.model,pages:layout.pages}]))};
  }
  if(!session)throw new Error('Initialize native SLAM first');
  if(message.type==='reset')return session.reset(message.frame,message.settings,message.parameters);
  if(message.type==='configure'){session.configure(message.settings);return {};}
  if(message.type==='configureParameters'){session.configureParameters(message.values);return {};}
  if(message.type==='predict'){session.predict(message.imageEpoch,message.interval);return {};}
  return session.advance(message.frame);
}
self.onmessage=async({data}:MessageEvent<SlamRequest>)=>{
  const started=performance.now();
  try{self.postMessage({id:data.id,result:await execute(data),timings:{workerMs:performance.now()-started}});}
  catch(error){self.postMessage({id:data.id,error:error instanceof Error?error.message:String(error),timings:{workerMs:performance.now()-started}});}
};
