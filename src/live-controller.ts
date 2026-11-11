import {LiveClient,RateMeter,decodeColor,type LiveStats} from './live-client';
import {DEFAULT_LIVE_CONFIG,parsePhoneCamera,PHONE_MAGIC,type PhoneCamera,type LiveConfig,type ResolutionId} from './live-source';
import type {Runtime} from './runtime';

export interface LiveHost {
  runtime:Runtime;
  status:(message:string,error?:boolean)=>void;
  /** Switch to the native estimator at this camera resolution and program set, recompiling if needed. */
  prepare:(resolution:ResolutionId,variant:'image'|'landmark')=>Promise<void>;
  /** Mark the compiled estimator stale so the next ordinary run rebuilds it with the usual image programs. */
  release:()=>void;
  /** Called when live playback begins so the page can reset its plots and Run label. */
  started:()=>void;
  stopped:()=>void;
  d435Started:()=>void;
  d435Frame:(points:number[][],position:number[]|undefined,time:number|undefined)=>void;
  d435Stopped:()=>void;
  d435Images:(camera:PhoneCamera,rgb:Uint8Array)=>void;
}

const RESOLUTIONS:{id:ResolutionId;label:string}[]=[
  {id:'640x480',label:'640 × 480 · full phone field of view (default)'},
  {id:'424x240',label:'424 × 240 · fastest, crops top and bottom'},
  {id:'640x360',label:'640 × 360 · crops top and bottom'},
  {id:'848x480',label:'848 × 480 · crops top and bottom, slowest'},
];

/** Adds a "Live iPhone" panel next to the dataset controls and drives a LiveClient against the runtime. */
export function installLiveCapture(host:LiveHost){
  const row=document.querySelector('.bottom-row');if(!row)return;
  const params=new URLSearchParams(location.search);
  const defaultUrl=params.get('live')??`ws://${location.hostname||'localhost'}:8787/runner`;
  const section=document.createElement('section');
  section.className='panel recordings live-capture';
  section.innerHTML=`<h2>Live sensors</h2>
<p>Use the iPhone for all sensors, or use USB RealSense color and depth with the iPhone IMU. The live estimator uses measurements from the relay.</p>
<label class="live-field">Source<select id="live-source"><option value="iphone">iPhone camera, depth and IMU</option><option value="d435-preview">RealSense D435</option></select></label>
<label class="live-field" id="d435-imu-field"><span><input type="checkbox" id="d435-imu"> Use iPhone IMU</span></label>
<label class="live-field">Relay<input id="live-url" type="text" spellcheck="false" autocomplete="off"></label>
<label class="live-field">Camera<select id="live-resolution"></select></label>
<label class="live-field">Phone sends<select id="live-mode"><option value="landmarks">Landmarks only (about 40 KB per frame)</option><option value="raw">Raw color + depth (about 1 MB per frame)</option></select></label>
<label class="live-field">Landmarks kept (top N by score): <b id="live-limit-value">350</b><input id="live-limit" type="range" min="20" max="350" step="10" value="350"></label>
<label class="live-field"><span><input id="live-zupt" type="checkbox" checked> Zero-velocity update when the phone is at rest</span></label>
<label class="live-field" id="live-interval-field">SLAM frame interval<select id="live-interval"><option value="0.3">0.3 s (about 3 fps, SLAM will lag most)</option><option value="0.4">0.4 s (2.5 fps)</option><option value="0.5">0.5 s (2 fps)</option><option value="0.7" selected>0.7 s (1.4 fps, default)</option></select></label>
<div class="live-buttons"><button id="live-start" class="primary">▶ Start live capture</button><button id="live-stop" disabled>■ Stop</button></div>
<small id="live-state">Not connected</small><small id="live-stats"></small>`;
  row.append(section);
  const style=document.createElement('style');
  style.textContent='.live-capture code{color:var(--green)}.live-field{display:flex;flex-direction:column;gap:4px;font-size:10px;color:var(--muted);margin-bottom:8px}.live-field input,.live-field select{font-size:11px}.live-buttons{display:flex;gap:8px;margin:6px 0 10px}.live-capture small{display:block;font-size:10px;color:var(--muted);margin-top:4px}';
  document.head.append(style);
  const $=<T extends HTMLElement>(id:string)=>section.querySelector<T>('#'+id)!;
  const url=$<HTMLInputElement>('live-url'),resolution=$<HTMLSelectElement>('live-resolution'),mode=$<HTMLSelectElement>('live-mode'),interval=$<HTMLSelectElement>('live-interval'),limit=$<HTMLInputElement>('live-limit'),zupt=$<HTMLInputElement>('live-zupt'),limitValue=$<HTMLElement>('live-limit-value');
  const source=$<HTMLSelectElement>('live-source');
  const phoneImu=$<HTMLInputElement>('d435-imu');
  const depthOnly=()=>source.value==='d435-preview'&&!phoneImu.checked;
  let preview:WebSocket|undefined;
  const updateSource=()=>{
    const mixed=source.value!=='iphone';
    $('d435-imu-field').style.display=mixed?'':'none';
    resolution.disabled=interval.disabled=depthOnly();
    $('live-interval-field').style.display=depthOnly()?'none':'';
    mode.disabled=mixed;
    limit.disabled=mixed;
    zupt.disabled=mixed;
    if(mixed)mode.value='raw';
  };
  source.onchange=phoneImu.onchange=updateSource;
  const requestedSource=params.get('source');
  if(requestedSource==='d435'||requestedSource==='d435-preview')source.value='d435-preview';
  phoneImu.checked=requestedSource==='d435';
  updateSource();
  url.value=defaultUrl;
  for(const r of RESOLUTIONS)resolution.add(new Option(r.label,r.id));
  const start=$<HTMLButtonElement>('live-start'),stop=$<HTMLButtonElement>('live-stop'),state=$<HTMLElement>('live-state'),stats=$<HTMLElement>('live-stats');
  const setState=(message:string,error=false)=>{state.textContent=message;state.style.color=error?'#efb29d':'';if(error)host.status(message,true);};

  limit.oninput=()=>{limitValue.textContent=limit.value;client?.setLandmarkLimit(Number(limit.value));};
  let client:LiveClient|undefined,cancel:(()=>void)|undefined,originalOnFrame=host.runtime.onFrame,wrapper:Runtime['onFrame']|undefined,lastState=0,lastPoints=0,rateTimer:ReturnType<typeof setInterval>|undefined;
  const renderStats=(s:LiveStats)=>{
    const lag=Math.max(0,s.capturedSeconds-s.processedSeconds);
    stats.textContent=`Phone ${s.peers.ingest?'connected':'not connected'} · ${source.value==='d435-preview'?(s.peers.d435?'D435 connected':'D435 waiting')+' · ':''}mirrors ${s.peers.views} · phone ${s.receivedFps.toFixed(1)} fps in · SLAM ${s.processedFps.toFixed(2)} fps · captured ${s.capturedSeconds.toFixed(1)} s · processed ${s.processedSeconds.toFixed(1)} s · behind ${lag.toFixed(1)} s · queue ${s.queued} frames (${s.queuedMB.toFixed(0)} MB)`;
  };
  async function begin(){
    await end(false);
    start.disabled=true;stop.disabled=false;source.disabled=phoneImu.disabled=true;
    if(depthOnly()){
      try{
        const viewer=new URL(url.value.trim());
        if(viewer.protocol!=='ws:'&&viewer.protocol!=='wss:')throw new Error('Enter a ws:// or wss:// relay URL');
        viewer.pathname='/view';
        viewer.searchParams.set('rgbd','1');
        const socket=preview=new WebSocket(viewer);
        socket.binaryType='arraybuffer';
        let position:number[]|undefined,time:number|undefined;
        let trackingLost=false, hadRunner=false,decoding=false;
        const frames=new RateMeter();
        socket.onopen=()=>{
          if(preview!==socket)return;
          host.d435Started();
          setState('Waiting for D435 · run the sender with --track for trajectory');
        };
        socket.onmessage=event=>{
          if(preview!==socket)return;
          if(typeof event.data==='string'){
            const message=JSON.parse(event.data);
            if(message.type==='state'&&message.source==='d435-odometry'){
              const pose=message.pose;
              position=[pose?.x,pose?.y,pose?.z];
              if(!position.every(Number.isFinite))position=undefined;
              time=Number.isFinite(message.time)?message.time:undefined;
            }
            if(message.type==='peers'){
              if(hadRunner&&!message.runner){
                position=undefined;time=undefined;trackingLost=false;
                setState('D435 sender disconnected');
              }
              if(!hadRunner&&message.runner)host.d435Started();
              hadRunner=!!message.runner;
            }
            if(message.type==='status'&&message.source==='d435-odometry'&&message.trackingLost){
              position=undefined;time=undefined;trackingLost=true;
              setState('Tracking lost · live depth continues. Restart the sender to begin a new trajectory.');
            }
            return;
          }
          const view=new DataView(event.data);
          if(view.byteLength<8)return;
          const magic=view.getUint32(0,true);
          if(magic===PHONE_MAGIC){
            if(decoding)return;
            decoding=true;
            void (async()=>{
              const camera=parsePhoneCamera(new Uint8Array(event.data));
              const {width,height}=camera.header.color;
              const rgb=await decodeColor(camera,{x:0,y:0,w:width,h:height},width,height);
              if(preview===socket)host.d435Images(camera,rgb);
            })().catch(error=>{if(preview===socket)setState(String(error),true);}).finally(()=>{decoding=false;});
            return;
          }
          if(magic!==0x31535450)return;
          frames.mark();
          stats.textContent='D435 received · '+frames.perSecond().toFixed(1)+' FPS';
          const count=view.getUint32(4,true);
          if(count>40000||view.byteLength!==8+count*12)return;
          const points=Array.from({length:count},(_,i)=>[view.getFloat32(8+i*12,true),view.getFloat32(12+i*12,true),view.getFloat32(16+i*12,true)]);
          host.d435Frame(points,position,time);
          if(!trackingLost)setState(position?'D435 live · RGB-D odometry':'D435 live · camera-relative depth');
        };
        socket.onclose=()=>{
          if(preview!==socket)return;
          void end(false);
          setState('D435 preview relay disconnected',true);
        };
      }catch(error){
        await end(false);
        setState(error instanceof Error?error.message:String(error),true);
      }
      return;
    }
    const cancelled=new Promise<never>((_,reject)=>{cancel=()=>reject(new Error('cancelled'));});
    try{
      const id=resolution.value as ResolutionId,sends=mode.value as LiveConfig['mode'];
      const d435=source.value==='d435-preview';
      setState(`Preparing the native SLAM at ${id}…`);
      await Promise.race([host.prepare(id,sends==='landmarks'?'landmark':'image'),cancelled]);
      const live=client=new LiveClient(url.value.trim(),{...DEFAULT_LIVE_CONFIG,resolution:id,mode:sends,frameIntervalSeconds:Number(interval.value),zeroVelocity:zupt.checked},d435?'d435':undefined);
      live.setLandmarkLimit(Number(limit.value));live.onStats=renderStats;rateTimer=setInterval(()=>live.refresh(),1000);live.onStatus=(m,error)=>setState(m,error);
      live.onCommand=message=>{if(message.action==='stop')void end(true);else if(message.action==='reset')void begin();};
      await Promise.race([live.connect(),cancelled]);
      setState(d435?'Connected · waiting for D435 frames and iPhone IMU…':'Connected · waiting for the first frame from the phone…');
      live.arm();
      await Promise.race([new Promise<void>(resolve=>{const timer=setInterval(()=>{if(live.feed.initial){clearInterval(timer);resolve();}},100);cancelled.catch(()=>clearInterval(timer));}),cancelled]);
      setState('Initializing the SLAM map from the first frame…');
      await Promise.race([host.runtime.startLive(live.feed),cancelled]);
      originalOnFrame=host.runtime.onFrame;
      host.runtime.onFrame=wrapper=(frame,estimate,truth,metrics)=>{
        originalOnFrame(frame,estimate,truth,metrics);
        live.noteProcessed(frame.time);
        const now=performance.now();
        if(now-lastState>500){lastState=now;live.publishState(frame.time,estimate,{loops:metrics.loops,frames:metrics.frames,rtf:metrics.rtf});}
        if(now-lastPoints>2000){lastPoints=now;live.publishPoints(estimate.points);}
      };
      host.started();host.runtime.play();
      setState('Live · the SLAM is processing frames as they arrive');
    }catch(error){
      if((error as Error).message!=='cancelled')setState(error instanceof Error?error.message:String(error),true);
      await end(false);
    }
  }
  async function end(announce:boolean){
    if(preview){const socket=preview;preview=undefined;socket.close();host.d435Stopped();}
    cancel?.();cancel=undefined;
    if(rateTimer){clearInterval(rateTimer);rateTimer=undefined;}
    if(wrapper&&host.runtime.onFrame===wrapper)host.runtime.onFrame=originalOnFrame;
    wrapper=undefined;
    if(host.runtime.liveActive)host.runtime.stopLive();
    client?.control('stop','runner-stopped');client?.close();client=undefined;
    start.disabled=false;stop.disabled=true;source.disabled=phoneImu.disabled=false;
    if(announce){host.release();setState('Stopped');host.stopped();}
  }
  start.onclick=()=>void begin();
  stop.onclick=()=>void end(true);
}
