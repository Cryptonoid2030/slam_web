// @vitest-environment node
import {afterAll,beforeAll,describe,expect,it} from 'vitest';
import {spawn,type ChildProcess} from 'node:child_process';
import {WebSocket} from 'ws';
import {parsePhoneCamera} from '../src/live-source';

const PORT=18787;
let relay:ChildProcess;
const open=(role:string,query='')=>new Promise<WebSocket>((resolve,reject)=>{
  const ws=new WebSocket(`ws://127.0.0.1:${PORT}/${role}${query}`);ws.once('open',()=>resolve(ws));ws.once('error',reject);
});
const until=async(check:()=>boolean,ms=4000)=>{const end=Date.now()+ms;while(!check()){if(Date.now()>end)throw new Error('timed out');await new Promise(r=>setTimeout(r,20));}};

beforeAll(async()=>{
  relay=spawn('node',['live/server.mjs',`--port=${PORT}`,'--token=secret'],{stdio:['ignore','pipe','inherit']});
  await new Promise<void>((resolve,reject)=>{relay.stdout!.on('data',d=>{if(String(d).includes('listening'))resolve();});relay.once('error',reject);setTimeout(()=>reject(new Error('relay did not start')),5000);});
});
afterAll(()=>{relay?.kill();});

describe('live relay',()=>{
  it('forwards the runner config to the phone, now and when the phone connects later',async()=>{
    const runner=await open('runner','?token=secret');
    const early=await open('ingest','?token=secret'),earlyText:string[]=[];early.on('message',d=>earlyText.push(String(d)));
    runner.send(JSON.stringify({type:'config',mode:'landmarks',width:424,height:240}));
    await until(()=>earlyText.some(t=>t.includes('"config"')));
    const late=await open('ingest','?token=secret'),lateText:string[]=[];late.on('message',d=>lateText.push(String(d)));
    await until(()=>lateText.some(t=>t.includes('"landmarks"')));
    runner.close();early.close();late.close();
  });
  it('rejects connections without the token',async()=>{
    await expect(open('ingest')).rejects.toBeDefined();
  });
  it('forwards phone IMU text and camera binaries to the runner, and runner state to viewers',async()=>{
    const runner=await open('runner','?token=secret'),view=await open('view','?token=secret'),phone=await open('ingest','?token=secret');
    const runnerText:string[]=[],runnerBinary:Buffer[]=[],viewText:string[]=[],viewBinary:Buffer[]=[];
    runner.on('message',(d,bin)=>{(bin?runnerBinary:runnerText).push(d as Buffer&string);});
    view.on('message',(d,bin)=>{(bin?viewBinary:viewText).push(d as Buffer&string);});
    await until(()=>viewText.some(t=>JSON.parse(String(t)).ingest===1&&JSON.parse(String(t)).runner===1));

    phone.send(JSON.stringify({type:'imu',samples:[[1,0,0,0,0,0,0]]}));
    const json=Buffer.from(JSON.stringify({type:'camera',t:2,color:{width:4,height:3,fx:5,fy:5,cx:2,cy:1.5},depth:{width:2,height:1},jpegBytes:3,depthBytes:8,confidenceBytes:2}));
    const bytes=Buffer.alloc(8+json.length+3+8+2);bytes.writeUInt32LE(0x31485049,0);bytes.writeUInt32LE(json.length,4);json.copy(bytes,8);
    phone.send(bytes);
    await until(()=>runnerText.some(t=>String(t).includes('"imu"'))&&runnerBinary.length===1);
    expect(parsePhoneCamera(new Uint8Array(runnerBinary[0])).header.t).toBe(2);

    runner.send(JSON.stringify({type:'state',time:3,frames:2}));
    runner.send(Buffer.from([1,2,3,4]),{binary:true});
    await until(()=>viewText.some(t=>JSON.parse(String(t)).type==='state')&&viewBinary.length===1);

    // late joiners get the latest state and points immediately
    const late=await open('view','?token=secret'),lateText:string[]=[],lateBinary:Buffer[]=[];
    late.on('message',(d,bin)=>{(bin?lateBinary:lateText).push(d as Buffer&string);});
    await until(()=>lateBinary.length===1&&lateText.some(t=>JSON.parse(String(t)).type==='state'));

    // runner control reaches the phone; viewer commands reach the runner
    const phoneText:string[]=[];phone.on('message',d=>phoneText.push(String(d)));
    runner.send(JSON.stringify({type:'control',action:'stop',reason:'buffer-full'}));
    await until(()=>phoneText.some(t=>t.includes('buffer-full')));
    view.send(JSON.stringify({type:'command',action:'reset'}));
    await until(()=>runnerText.some(t=>t.includes('"command"')));
    [runner,view,phone,late].forEach(ws=>ws.close());
  });
});
