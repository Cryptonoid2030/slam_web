// Runs the pinned image SLAM programs and the landmark-input programs on the same frames
// and compares what they estimate. Usage: tsx scripts/landmark-equivalence.mts FRAMES_DIR [WIDTH HEIGHT COUNT]
import {readFileSync} from 'node:fs';
import {gunzipSync} from 'node:zlib';
import {join} from 'node:path';
import {ModelicaSlamSession} from '../src/modelica-slam-session';
import {slamInitialFrameFromSensor,slamIntervalFrameFromSensor} from '../src/modelica-slam-frame';
import {extractLandmarks,sparseFrame} from '../src/landmark-source';
import type {InitialSensorFrame,SensorFrame} from '../src/types';

const [dir,w='424',h='240',n='6']=process.argv.slice(2);
const width=+w,height=+h,count=+n,id=`${width}x${height}`;
const load=(root:string)=>{
  const manifest=JSON.parse(readFileSync(join(root,'manifest.json'),'utf8'));
  const profile=manifest.profiles?.[id]??manifest;
  return {manifest:{schemaVersion:1 as const,sourceSha256:profile.sourceSha256,models:profile.models},
    fetch:async(file:string,encoding:string)=>{const b=readFileSync(join(root,file));return new Uint8Array(encoding==='gzip'?gunzipSync(b):b);}};
};
const calibration={width,height,fx:width*1.2,fy:width*1.2,cx:(width-1)/2,cy:(height-1)/2,near:.2,far:5,forward:0,up:0,baseline:.05,rgbFx:width*1.2,rgbFy:width*1.2,
  opticalToBody:[0,0,1,-1,0,0,0,-1,0],originFlu:[0,0,0],depthNoiseDisparityPx:.1,depthNoiseReferenceFx:width*1.2,depthEncoding:'axial-z16-le' as const};
const layout={color:{width,height,strideBytes:width*3,bytes:width*height*3,format:'RGB8' as const},
  depth:{width,height,strideBytes:width*2,bytes:width*height*2,format:'Z16' as const,unitsMeters:.001,isBigEndian:false as const},rowOrder:'top-down' as const,aligned:false as const,clockDomain:'simulation' as const};
const raster=(i:number)=>{
  const rgb=new Uint8Array(readFileSync(join(dir,`frame-${i}.rgb8`)));const raw=readFileSync(join(dir,`frame-${i}.z16-le`));
  return {rgb,depth:new Uint16Array(raw.buffer.slice(raw.byteOffset,raw.byteOffset+raw.byteLength))};
};
const imu={accel:[0,0,9.80665],gyro:[0,0,0]};
const spacing=.5;
function frame(i:number,sparse:boolean):SensorFrame{
  let {rgb,depth}=raster(i),landmarks;
  if(sparse){const s=sparseFrame(extractLandmarks(rgb,depth,width,height,{depthUnitsMeters:.001}),width,height);rgb=s.rgb;depth=s.depth;landmarks={pixels:s.pixels,score:s.score,count:s.count};}
  const intervals=i===0?[]:[0,1,2,3,4].map(k=>({time:(i-1)*spacing+(k+1)*spacing/5,dt:spacing/5,imu}));
  return {sequence:i,time:i*spacing,dt:i===0?0:spacing,imu,imuIntervals:intervals,rgb,depth,imageLayout:layout,calibration,landmarks};
}
const run=async(root:string,sparse:boolean)=>{
  let {manifest,fetch}=load(root);
  if(process.env.MIX&&sparse){
    // Debug: pinned image Initialize with the landmark Step.
    const o=load('public/vendor/slam'),f=fetch;
    manifest={...manifest,models:{initialize:o.manifest.models.initialize,step:manifest.models.step}};
    fetch=async(file,enc)=>{try{return await f(file,enc);}catch{return o.fetch(file,enc);}};
  }
  const session=await ModelicaSlamSession.create(manifest,fetch,slamInitialFrameFromSensor({...frame(0,sparse&&!process.env.MIX),dt:0,imuIntervals:[]} as InitialSensorFrame));
  const out=[{est:session.initialEstimate,ms:[] as number[]}];
  for(let i=1;i<count;i++){const r=session.advance(slamIntervalFrameFromSensor(frame(i,sparse)));if(process.env.PEEK){const st=(session as any).step;console.log('peek',i,'count',st.real('landmarkCount')[0],'pix0',st.real('landmarkPixels')[0],'imageRequested',st.boolean('imageRequested')[0],'epoch',st.integer('imageEpoch')[0],'h',st.real('h')[0],'t',st.real('intervalTime')[0],'req',st.boolean('requested')[0],'lcr',st.boolean('localCaptureRequested')[0],'dqs',st.boolean('depthQualifiedSelection')[0],'limit',st.real('selectedFeatureLimit')[0],'thr',st.real('absoluteThreshold')[0],'outs',['predictionAccepted','selectionValid','imageCompleted'].map(n=>st.output(n)?.[0]));}out.push({est:r.estimate,ms:r.stepMs});}
  return out;
};
const original=await run('public/vendor/slam',false),landmark=await run('public/vendor/slam-landmark',true);
let worst=0;
for(let i=0;i<count;i++){
  const a=original[i].est,b=landmark[i].est;
  const dp=Math.hypot(a.x-b.x,a.y-b.y,a.z-b.z),dq=Math.hypot(...a.quaternion.map((v,k)=>v-b.quaternion[k]));
  const fa=JSON.stringify(a.features),fb=JSON.stringify(b.features);
  worst=Math.max(worst,dp,dq);
  console.log(JSON.stringify({frame:i,dPos:dp,dQuat:dq,featuresEqual:fa===fb,featuresOriginal:a.features?.length,featuresLandmark:b.features?.length,
    matchOriginal:a.diagnostics?.matchCount,matchLandmark:b.diagnostics?.matchCount,mapOriginal:a.points.length,mapLandmark:b.points.length,
    msOriginal:Math.round(original[i].ms.reduce((x,y)=>x+y,0)),msLandmark:Math.round(landmark[i].ms.reduce((x,y)=>x+y,0))}));
}
console.log(worst===0?'IDENTICAL':`max difference ${worst}`);
if(process.env.DETAIL){
  const a=original[0].est.features!,b=landmark[0].est.features!;
  console.log('f0 original',JSON.stringify(a.slice(0,3)),'landmark',JSON.stringify(b.slice(0,3)));
  let n=0;for(let k=0;k<a.length;k++)if(a[k][0]!==b[k][0]||a[k][1]!==b[k][1])n++;console.log('pixel mismatches frame0',n);
  console.log(JSON.stringify(landmark[1].est.diagnostics),JSON.stringify(original[1].est.diagnostics));
}
if(process.env.DETAIL){
  const keys=Object.keys(original[0].est.diagnostics!).filter(k=>!k.includes('['));
  for(const k of keys){const a=original[0].est.diagnostics![k],b=landmark[0].est.diagnostics![k];if(a!==b)console.log('init diag differs',k,a,b);}
  for(const k of keys){const a=original[1].est.diagnostics![k],b=landmark[1].est.diagnostics![k];if(a!==b&&!k.includes('stepMs'))console.log('step1 diag differs',k,a,b);}
}
