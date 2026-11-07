#!/usr/bin/env node
// Stands in for the iPhone capture app so the relay, SLAM tab and mirror viewer can be tried
// without a device. It follows the page's `config` message: raw JPEG + depth frames, or landmark packets (IPL1) selected here.
// Streams a stationary phone: IMU at 100 Hz (batched), a tilted-plane depth
// map at 256x192 and a fixed JPEG at a few Hz. It uses the same wire format as the Swift app.
import fs from 'node:fs';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {WebSocket} from 'ws';
// Node strips the types of this dependency-free module, so the simulator selects landmarks exactly as the page's reference does.
import {encodeLandmarkPacket,extractLandmarks} from '../src/landmark-source.ts';

const here=path.dirname(fileURLToPath(import.meta.url));
const opts=Object.fromEntries(process.argv.slice(2).map(a=>a.replace(/^--/,'').split('=')).map(([k,v])=>[k,v??'true']));
const url=opts.url??'ws://localhost:8787/ingest';
const fps=Number(opts.fps??4),seconds=Number(opts.seconds??60),imuHz=Number(opts.imu??100);
const jpeg=fs.readFileSync(opts.jpeg??path.join(here,'fixtures','room.jpg'));
const DW=256,DH=192,MAGIC=0x31485049;
const clock0=1000+Math.random()*1e4,wall0=Date.now();
const now=()=>clock0+(Date.now()-wall0)/1000;   // seconds on a shared "phone" clock

function cameraMessage(t){
  const depth=new Float32Array(DW*DH),confidence=new Uint8Array(DW*DH);
  for(let y=0;y<DH;y++)for(let x=0;x<DW;x++){depth[y*DW+x]=1.6+.5*x/DW+.3*y/DH;confidence[y*DW+x]=(x+y)%97===0?0:2;}
  const header=Buffer.from(JSON.stringify({type:'camera',t,color:{width:1920,height:1440,fx:1450,fy:1450,cx:960,cy:720},
    depth:{width:DW,height:DH},jpegBytes:jpeg.length,depthBytes:depth.byteLength,confidenceBytes:confidence.length}));
  const out=Buffer.alloc(8+header.length+jpeg.length+depth.byteLength+confidence.length);
  out.writeUInt32LE(MAGIC,0);out.writeUInt32LE(header.length,4);header.copy(out,8);jpeg.copy(out,8+header.length);
  Buffer.from(depth.buffer).copy(out,8+header.length+jpeg.length);Buffer.from(confidence).copy(out,8+header.length+jpeg.length+depth.byteLength);
  return out;
}
let config=null;   // the page's capture settings (relay replays them on connect)
/** A textured raster on the target grid (blobs on a gradient) with a tilted-plane depth in Z16, to select landmarks from. */
function syntheticScene(width,height,unitsMeters){
  const rgb=new Uint8Array(width*height*3),depth=new Uint16Array(width*height);
  let seed=12345;const rand=()=>{seed=(seed*1664525+1013904223)>>>0;return seed/2**32;};
  const blobs=Array.from({length:Math.round(width*height/900)},()=>({x:8+rand()*(width-16),y:8+rand()*(height-16),r:2+rand()*5,v:rand()<.5?30:225}));
  for(let y=0;y<height;y++)for(let x=0;x<width;x++){
    let v=110+20*Math.sin(x/17)+15*Math.cos(y/11);
    for(const b of blobs)if(Math.hypot(x-b.x,y-b.y)<b.r)v=b.v;
    const i=y*width+x;rgb[3*i]=v;rgb[3*i+1]=v*.9;rgb[3*i+2]=v*.8;depth[i]=Math.round((1.6+.5*x/width+.3*y/height)/unitsMeters);
  }
  return {rgb,depth};
}
function landmarkMessage(t){
  const {width,height,depthUnitsMeters}=config,{rgb,depth}=syntheticScene(width,height,depthUnitsMeters);
  const set=extractLandmarks(rgb,depth,width,height,{depthUnitsMeters,threshold:config.fastThreshold});
  const header={type:'landmarks',t,count:set.count,color:{width:1920,height:1440,fx:1450,fy:1450,cx:960,cy:720},target:{width,height}};
  return Buffer.from(encodeLandmarkPacket(header,set));
}
const ws=new WebSocket(url);
ws.on('error',e=>{console.error('connection failed:',e.message);process.exit(1);});
ws.on('message',m=>{try{const j=JSON.parse(m.toString());if(j.type==='config'){config=j;console.log(`page asks for ${j.mode} at ${j.resolution}`);}if(j.type==='control'){console.log('relay says:',j);if(j.action==='stop')process.exit(0);}}catch{}});
ws.on('open',()=>{
  console.log(`streaming to ${url} for ${seconds}s at ${fps} fps camera, ${imuHz} Hz IMU`);
  const stop=Date.now()+seconds*1000;let pending=[],sent=0;
  // Landscape-right, held upright: CoreMotion reads x = -1 g; tiny gyro/accel noise.
  const imuTimer=setInterval(()=>{pending.push([now(),-1+(Math.random()-.5)*.004,(Math.random()-.5)*.004,(Math.random()-.5)*.004,(Math.random()-.5)*.002,(Math.random()-.5)*.002,(Math.random()-.5)*.002]);},1000/imuHz);
  const flushTimer=setInterval(()=>{if(pending.length){ws.send(JSON.stringify({type:'imu',samples:pending}));pending=[];}},50);
  const camTimer=setInterval(()=>{
    // IMU must reach the runner before the camera frame that it precedes.
    if(pending.length){ws.send(JSON.stringify({type:'imu',samples:pending}));pending=[];}
    ws.send(config?.mode==='landmarks'?landmarkMessage(now()):cameraMessage(now()));sent++;
  },1000/fps);
  const done=setInterval(()=>{if(Date.now()>stop){[imuTimer,flushTimer,camTimer,done].forEach(clearInterval);console.log(`sent ${sent} camera frames`);ws.close();}},200);
});
