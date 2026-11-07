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

let ingest=null,runner=null;
const views=new Set();
let lastState=null,lastPoints=null;   // replayed to late-joining viewers
let lastConfig=null;                  // runner's capture settings, replayed to a phone that connects later

function peerCounts(){return {type:'peers',ingest:ingest?1:0,runner:runner?1:0,views:views.size};}
function sendJson(ws,value){if(ws&&ws.readyState===1)ws.send(JSON.stringify(value));}
function broadcastPeers(){const m=JSON.stringify(peerCounts());for(const ws of [ingest,runner,...views])if(ws&&ws.readyState===1)ws.send(m);}

server.on('upgrade',(req,socket,head)=>{
  const url=new URL(req.url,'http://x'),role=url.pathname.slice(1);
  if(!['ingest','runner','view'].includes(role)||(token&&url.searchParams.get('token')!==token)){socket.write('HTTP/1.1 403 Forbidden\r\n\r\n');socket.destroy();return;}
  wss.handleUpgrade(req,socket,head,ws=>attach(ws,role));
});

function attach(ws,role){
  if(role==='ingest'){if(ingest&&ingest!==ws)ingest.close(4000,'replaced');ingest=ws;if(lastConfig)ws.send(lastConfig);}
  else if(role==='runner'){if(runner&&runner!==ws)runner.close(4000,'replaced');runner=ws;lastState=null;lastPoints=null;lastConfig=null;}
  else{views.add(ws);if(lastState)ws.send(lastState);if(lastPoints)ws.send(lastPoints,{binary:true});}
  console.log(`+ ${role} connected (${new Date().toLocaleTimeString()})`);
  broadcastPeers();
  ws.on('message',(data,isBinary)=>{
    if(role==='ingest'){
      if(!runner||runner.readyState!==1)return;
      // Camera frames are binary; IMU batches are text. Only camera frames may be shed.
      if(isBinary&&runner.bufferedAmount>MAX_RUNNER_BACKLOG)return;
      runner.send(data,{binary:isBinary});
    }else if(role==='runner'){
      if(isBinary){lastPoints=data;for(const v of views)if(v.readyState===1)v.send(data,{binary:true});}
      else{
        const text=data.toString();
        let message;try{message=JSON.parse(text);}catch{return;}
        if(message.type==='state'){lastState=text;for(const v of views)if(v.readyState===1)v.send(text);}
        else if(message.type==='control'){sendJson(ingest,message);}      // e.g. {type:'control',action:'stop'}
        else if(message.type==='config'){lastConfig=text;if(ingest&&ingest.readyState===1)ingest.send(text);}  // what the phone should send
        else if(message.type==='status'){for(const v of views)sendJson(v,message);sendJson(ingest,message);}
      }
    }else{
      // viewer commands go to the runner (reset, start, stop)
      if(!isBinary){try{const m=JSON.parse(data.toString());if(m.type==='command')sendJson(runner,m);}catch{}}
    }
  });
  ws.on('close',()=>{
    if(ws===ingest)ingest=null;else if(ws===runner){runner=null;lastState=null;lastPoints=null;lastConfig=null;}else views.delete(ws);
    console.log(`- ${role} disconnected`);broadcastPeers();
  });
  ws.on('error',error=>console.error(`${role} socket error:`,error.message));
}

function lanAddresses(){
  const out=[];
  for(const list of Object.values(os.networkInterfaces()))for(const a of list??[])if(a.family==='IPv4'&&!a.internal)out.push(a.address);
  return out;
}
server.listen(port,'0.0.0.0',()=>{
  const scheme=secure?'https':'http',ws=secure?'wss':'ws',q=token?`?token=${token}`:'';
  console.log(`Live relay listening on port ${port} (${secure?'HTTPS':'plain HTTP'})`);
  for(const ip of lanAddresses()){
    console.log(`  phone app  -> ${ws}://${ip}:${port}/ingest${q}`);
    console.log(`  Safari view -> ${scheme}://${ip}:${port}/view.html${q}`);
  }
  console.log(`  SLAM tab   -> add ?live=${ws}://localhost:${port}/runner${q?'&':''}${q.replace('?','')} to the slam_web URL`);
  if(!lanAddresses().length)console.log('  (no LAN address found; connect the Mac to Wi-Fi)');
});
