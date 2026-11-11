#!/usr/bin/env node
// Relay for live phone capture: iPhone app -> /ingest, SLAM tab on the Mac -> /runner,
// Safari mirror viewers -> /view. Serves the mirror page over plain HTTP, or HTTPS when
// --cert/--key are given. The relay never inspects frames; it only forwards them.
import http from 'node:http';
import https from 'node:https';
import fs from 'node:fs';
import os from 'node:os';
import path from 'node:path';
import {fileURLToPath} from 'node:url';
import {WebSocketServer} from 'ws';

const here=path.dirname(fileURLToPath(import.meta.url));
const root=path.resolve(here,'..');
const args=new Map();
for(let i=2;i<process.argv.length;i++){
  const a=process.argv[i];
  if(!a.startsWith('--'))continue;
  const [k,v]=a.slice(2).split('=');
  args.set(k,v??((process.argv[i+1]&&!process.argv[i+1].startsWith('--'))?process.argv[++i]:'true'));
}
const port=Number(args.get('port')??process.env.PORT??8787);
const token=args.get('token')??process.env.LIVE_TOKEN??'';
const certPath=args.get('cert'),keyPath=args.get('key');
/** Runner-bound camera messages are dropped above this backlog; IMU (text) is never dropped. */
const MAX_RUNNER_BACKLOG=Number(args.get('max-backlog')??16*1024*1024);

const MIME={'.html':'text/html; charset=utf-8','.js':'text/javascript; charset=utf-8','.mjs':'text/javascript; charset=utf-8','.css':'text/css; charset=utf-8','.json':'application/json'};
const staticFiles=new Map([
  ['/view.html',path.join(here,'viewer','view.html')],
  ['/view.js',path.join(here,'viewer','view.js')],
  ['/three/three.module.js',path.join(root,'node_modules','three','build','three.module.js')],
  ['/three/three.core.js',path.join(root,'node_modules','three','build','three.core.js')],
  ['/three/OrbitControls.js',path.join(root,'node_modules','three','examples','jsm','controls','OrbitControls.js')],
]);

function handler(req,res){
  const url=new URL(req.url,'http://x'),p=url.pathname;
  if(p==='/'){res.writeHead(302,{Location:'/view.html'+url.search});res.end();return;}
  if(p==='/health'){res.writeHead(200,{'Content-Type':'application/json'});res.end(JSON.stringify(peerCounts()));return;}
  const file=staticFiles.get(p);
  if(!file){res.writeHead(404);res.end('Not found');return;}
  fs.readFile(file,(error,data)=>{
    if(error){res.writeHead(404);res.end('Missing '+path.basename(file));return;}
    res.writeHead(200,{'Content-Type':MIME[path.extname(file)]??'application/octet-stream','Cache-Control':'no-store'});res.end(data);
  });
}
const secure=!!(certPath&&keyPath);
const server=secure?https.createServer({cert:fs.readFileSync(certPath),key:fs.readFileSync(keyPath)},handler):http.createServer(handler);
const wss=new WebSocketServer({noServer:true,maxPayload:32*1024*1024});

let ingest=null,runner=null,d435=null;
let imuClockSamples=0,phoneOffset=Infinity,latestImu=-Infinity,pendingDepth=null;
function resetMixedClock(){imuClockSamples=0;phoneOffset=Infinity;latestImu=-Infinity;pendingDepth=null;}
function mixedMode(){return lastConfig&&JSON.parse(lastConfig).source==="d435";}
function flushDepth(){
  if(!pendingDepth||runner?.readyState!==1)return;
  const t=pendingDepth.header.t-phoneOffset;
  if(t>latestImu)return;
  const {header,payload}=pendingDepth;pendingDepth=null;
  if(t<latestImu-0.5||t<0||runner.bufferedAmount>MAX_RUNNER_BACKLOG)return;
  const json=Buffer.from(JSON.stringify({...header,t}));
  const prefix=Buffer.alloc(8);prefix.write("IPH1");prefix.writeUInt32LE(json.length,4);
  runner.send(Buffer.concat([prefix,json,payload]),{binary:true});
}
const views=new Set();
let lastState=null,lastPoints=null;   // replayed to late-joining viewers
let lastConfig=null;                  // runner's capture settings, replayed to a phone that connects later

function peerCounts(){return {type:'peers',ingest:ingest?1:0,runner:runner?1:0,d435:d435?1:0,views:views.size};}
function sendJson(ws,value){if(ws&&ws.readyState===1)ws.send(JSON.stringify(value));}
function broadcastPeers(){const m=JSON.stringify(peerCounts());for(const ws of [ingest,runner,d435,...views])if(ws&&ws.readyState===1)ws.send(m);}

server.on('upgrade',(req,socket,head)=>{
  const url=new URL(req.url,'http://x'),role=url.pathname.slice(1);
  if(!['ingest','runner','view','d435'].includes(role)||(token&&url.searchParams.get('token')!==token)){socket.write('HTTP/1.1 403 Forbidden\r\n\r\n');socket.destroy();return;}
  wss.handleUpgrade(req,socket,head,ws=>attach(ws,role,url.searchParams.get('rgbd')==='1'));
});

function attach(ws,role,receiveRgbd){
  ws.receiveRgbd=receiveRgbd;
  if(role==='ingest'){if(ingest&&ingest!==ws)ingest.close(4000,'replaced');ingest=ws;resetMixedClock();if(lastConfig)ws.send(lastConfig);}
  else if(role==='d435'){if(d435)d435.close(4000,'replaced');d435=ws;pendingDepth=null;}
  else if(role==='runner'){if(runner&&runner!==ws)runner.close(4000,'replaced');runner=ws;lastState=null;lastPoints=null;lastConfig=null;resetMixedClock();}
  else{views.add(ws);if(lastState)ws.send(lastState);if(lastPoints)ws.send(lastPoints,{binary:true});}
  console.log(`+ ${role} connected (${new Date().toLocaleTimeString()})`);
  broadcastPeers();
  ws.on('message',(data,isBinary)=>{
    if(role==='ingest'){
      if(ws!==ingest||runner?.readyState!==1)return;
      if(mixedMode()){
        if(isBinary)return;
        let message;try{message=JSON.parse(data.toString());}catch{return;}
        const sample=message.type==='imu'&&Array.isArray(message.samples)?message.samples.at(-1):null;
        if(!Array.isArray(sample)||!Number.isFinite(sample[0]))return;
        if(sample[0]<=latestImu)return;
        latestImu=sample[0];
        // Lowest observed batch delay approximates the phone-to-host clock offset.
        if(imuClockSamples<12){phoneOffset=Math.min(phoneOffset,Date.now()/1000-latestImu);imuClockSamples++;}
        runner.send(data,{binary:false});
        flushDepth();
        return;
      }
      // Camera frames are binary; IMU batches are text. Only camera frames may be shed.
      if(isBinary&&runner.bufferedAmount>MAX_RUNNER_BACKLOG)return;
      runner.send(data,{binary:isBinary});
    }else if(role==='d435'){
      if(ws!==d435||!mixedMode()||imuClockSamples<12||!isBinary||data.length<8||data.toString('ascii',0,4)!=='IPH1')return;
      const end=8+data.readUInt32LE(4);
      if(end>data.length||end>65536)return;
      let header;try{header=JSON.parse(data.subarray(8,end));}catch{return;}
      if(header?.type!=='camera'||!Number.isFinite(header.t)||Math.abs(Date.now()/1000-header.t)>1)return;
      pendingDepth={header,payload:data.subarray(end)};
      flushDepth();
    }else if(role==='runner'){
      if(ws!==runner)return;
      if(isBinary){
        const rgbd=data.length>=4&&data.toString('ascii',0,4)==='IPH1';
        if(!rgbd)lastPoints=data;
        for(const v of views){
          if(v.readyState===1&&(!rgbd||v.receiveRgbd)&&v.bufferedAmount<MAX_RUNNER_BACKLOG)v.send(data,{binary:true});
        }
      }
      else{
        const text=data.toString();
        let message;try{message=JSON.parse(text);}catch{return;}
        if(message.type==='state'){lastState=text;for(const v of views)if(v.readyState===1)v.send(text);}
        else if(message.type==='control'){sendJson(ingest,message);}      // e.g. {type:'control',action:'stop'}
        else if(message.type==='config'){if(!lastConfig||message.source!==JSON.parse(lastConfig).source)resetMixedClock();lastConfig=text;if(ingest&&ingest.readyState===1)ingest.send(text);}  // what the phone should send
        else if(message.type==='status'){for(const v of views)sendJson(v,message);sendJson(ingest,message);}
      }
    }else{
      // viewer commands go to the runner (reset, start, stop)
      if(!isBinary){try{const m=JSON.parse(data.toString());if(m.type==='command')sendJson(runner,m);}catch{}}
    }
  });
  ws.on('close',()=>{
    if(ws===ingest){ingest=null;resetMixedClock();}else if(ws===d435){d435=null;pendingDepth=null;}else if(ws===runner){runner=null;lastState=null;lastPoints=null;lastConfig=null;}else views.delete(ws);
    console.log(`- ${role} disconnected`);broadcastPeers();
  });
  ws.on('error',error=>console.error(`${role} socket error:`,error.message));
}

function lanAddresses(){
  const out=[];
  for(const list of Object.values(os.networkInterfaces()))for(const a of list??[])if(a.family==='IPv4'&&!a.internal)out.push(a.address);
  return out;
}
server.listen(port,args.get('host')??'0.0.0.0',()=>{
  const scheme=secure?'https':'http',ws=secure?'wss':'ws',q=token?`?token=${token}`:'';
  console.log(`Live relay listening on port ${port} (${secure?'HTTPS':'plain HTTP'})`);
  for(const ip of lanAddresses()){
    console.log(`  phone app  -> ${ws}://${ip}:${port}/ingest${q}`);
    console.log(`  Safari view -> ${scheme}://${ip}:${port}/view.html${q}`);
  }
  console.log(`  SLAM tab   -> add ?live=${ws}://localhost:${port}/runner${q?'&':''}${q.replace('?','')} to the slam_web URL`);
  if(!lanAddresses().length)console.log('  (no LAN address found; connect the Mac to Wi-Fi)');
});
