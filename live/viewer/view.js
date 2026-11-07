import * as THREE from 'three';
import {OrbitControls} from '/three/OrbitControls.js';

// Mirror of the SLAM running on the Mac: pose, trajectory, keyframe graph and a decimated map.
// Map frame is forward-left-up; Three.js is right-handed Y-up, so (x,y,z) -> (x, z, -y).
const $=id=>document.getElementById(id);
const toThree=(x,y,z)=>new THREE.Vector3(x,z,-y);
const query=new URLSearchParams(location.search),token=query.get('token');

const renderer=new THREE.WebGLRenderer({canvas:$('scene'),antialias:true});
renderer.setPixelRatio(Math.min(devicePixelRatio,2));
const scene=new THREE.Scene();scene.background=new THREE.Color(0x0b1118);
const camera=new THREE.PerspectiveCamera(60,1,.05,500);camera.position.set(-3,2.5,3);
const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.dampingFactor=.12;
scene.add(new THREE.GridHelper(40,40,0x2a3b48,0x18242d));
scene.add(new THREE.AxesHelper(.5));

const mapGeometry=new THREE.BufferGeometry();
const mapPoints=new THREE.Points(mapGeometry,new THREE.PointsMaterial({size:.025,vertexColors:true,sizeAttenuation:true}));
mapPoints.frustumCulled=false;scene.add(mapPoints);

const MAX_PATH=20000;
const pathPositions=new Float32Array(MAX_PATH*3);
const pathGeometry=new THREE.BufferGeometry();pathGeometry.setAttribute('position',new THREE.BufferAttribute(pathPositions,3));pathGeometry.setDrawRange(0,0);
const pathLine=new THREE.Line(pathGeometry,new THREE.LineBasicMaterial({color:0x53e2b2}));pathLine.frustumCulled=false;scene.add(pathLine);
let pathCount=0;

const graphGroup=new THREE.Group();scene.add(graphGroup);
const keyframeMaterial=new THREE.PointsMaterial({size:.08,color:0xffd166,sizeAttenuation:true});
const odometryMaterial=new THREE.LineBasicMaterial({color:0x4b6b7d}),loopMaterial=new THREE.LineBasicMaterial({color:0xed8b75});

// current pose: small camera frustum
const marker=new THREE.Group();
const frustum=new THREE.LineSegments(new THREE.EdgesGeometry(new THREE.ConeGeometry(.12,.25,4,1,true)),new THREE.LineBasicMaterial({color:0xffffff}));
const markerInner=new THREE.Group();markerInner.add(frustum);marker.add(markerInner);scene.add(marker);

// 'fit' keeps the whole map framed as it grows; 'follow' tracks the phone; touching the view switches to 'free'.
let mode='fit',showGraph=true,lastPose=null;
function resize(){const w=innerWidth,h=innerHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}
addEventListener('resize',resize);resize();

function applyPose(pose){
  const p=toThree(pose.x,pose.y,pose.z);
  // body forward-left-up -> three: quaternion conjugation by the basis change (x,y,z)->(x,z,-y)
  const [w,x,y,z]=pose.q,q=new THREE.Quaternion(x,z,-y,w);
  marker.position.copy(p);marker.quaternion.copy(q);
  // aim the frustum along body forward (+x in the body frame)
  markerInner.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0),new THREE.Vector3(1,0,0));
  if(pathCount<MAX_PATH){
    const last=pathCount?new THREE.Vector3(pathPositions[3*pathCount-3],pathPositions[3*pathCount-2],pathPositions[3*pathCount-1]):null;
    if(!last||last.distanceTo(p)>.01){pathPositions.set([p.x,p.y,p.z],3*pathCount);pathCount++;pathGeometry.setDrawRange(0,pathCount);pathGeometry.attributes.position.needsUpdate=true;}
  }
  if(mode==='follow'&&lastPose){const delta=p.clone().sub(toThree(lastPose.x,lastPose.y,lastPose.z));camera.position.add(delta);controls.target.add(delta);}
  lastPose=pose;
}
function updateGraph(keyframes,edges){
  graphGroup.clear();
  if(!keyframes.length)return;
  const pts=keyframes.map(k=>toThree(k[0],k[1],k[2]));
  graphGroup.add(new THREE.Points(new THREE.BufferGeometry().setFromPoints(pts),keyframeMaterial));
  for(const [kind,material] of [[0,odometryMaterial],[1,loopMaterial]]){
    const seg=[];
    for(const [from,to,k] of edges)if(k===kind&&pts[from]&&pts[to])seg.push(pts[from],pts[to]);
    if(seg.length)graphGroup.add(new THREE.LineSegments(new THREE.BufferGeometry().setFromPoints(seg),material));
  }
  graphGroup.visible=showGraph;
}
function setMap(floats){
  const count=floats.length/3,pos=new Float32Array(count*3),col=new Float32Array(count*3);
  let zmin=Infinity,zmax=-Infinity;for(let i=0;i<count;i++){const z=floats[3*i+2];if(z<zmin)zmin=z;if(z>zmax)zmax=z;}
  const span=Math.max(.5,zmax-zmin),color=new THREE.Color();
  for(let i=0;i<count;i++){
    const x=floats[3*i],y=floats[3*i+1],z=floats[3*i+2];
    pos[3*i]=x;pos[3*i+1]=z;pos[3*i+2]=-y;
    color.setHSL(.62-.55*Math.min(1,Math.max(0,(z-zmin)/span)),.7,.55);col.set([color.r,color.g,color.b],3*i);
  }
  mapGeometry.setAttribute('position',new THREE.BufferAttribute(pos,3));mapGeometry.setAttribute('color',new THREE.BufferAttribute(col,3));
  mapGeometry.computeBoundingSphere();
  if(mode==='fit'&&count>200)fit();
}
function fit(){
  const sphere=mapGeometry.boundingSphere;if(!sphere||!isFinite(sphere.radius))return;
  controls.target.copy(sphere.center);
  // Frame the whole map in whichever dimension of the (portrait or landscape) screen is tighter.
  const vfov=THREE.MathUtils.degToRad(camera.fov),hfov=2*Math.atan(Math.tan(vfov/2)*camera.aspect);
  const distance=Math.max(1.5,sphere.radius/Math.sin(Math.min(vfov,hfov)/2)*1.1);
  camera.position.copy(sphere.center).add(new THREE.Vector3(-.5,.9,.9).normalize().multiplyScalar(distance));
}

// ---- network -------------------------------------------------------------------------
let socket,peers={ingest:0,runner:0,views:0},state=null,retry=0;
function connect(){
  const scheme=location.protocol==='https:'?'wss':'ws';
  socket=new WebSocket(`${scheme}://${location.host}/view${token?`?token=${encodeURIComponent(token)}`:''}`);socket.binaryType='arraybuffer';
  socket.onopen=()=>{retry=0;render();};
  socket.onclose=()=>{peers={ingest:0,runner:0,views:0};render('Reconnecting…');setTimeout(connect,Math.min(5000,500*2**retry++));};
  socket.onmessage=event=>{
    if(typeof event.data==='string'){
      const message=JSON.parse(event.data);
      if(message.type==='peers'){peers=message;render();}
      else if(message.type==='state'){state=message;applyPose(message.pose);updateGraph(message.keyframes,message.edges);render();}
      else if(message.type==='status'&&message.message){$('note').textContent=message.message;}
    }else{
      const view=new DataView(event.data);
      if(view.getUint32(0,true)===0x31535450){const count=view.getUint32(4,true);setMap(new Float32Array(event.data,8,count*3));}
    }
  };
}
function render(override){
  const live=socket?.readyState===1&&peers.runner===1;
  $('dot').className=live?'ok':'';
  $('title').textContent=override??(socket?.readyState!==1?'Connecting…':!peers.runner?'Waiting for the SLAM tab on the Mac':!peers.ingest?'SLAM ready · waiting for the phone':'Live');
  const s=state,lag=s?Math.max(0,s.captured-s.time):0;
  $('stats').innerHTML=s?`<span>Processed <b>${s.time.toFixed(1)} s</b></span><span>Behind <b>${lag.toFixed(1)} s</b></span>
    <span>Frames <b>${s.frames}</b></span><span>Queued <b>${s.queued}</b></span>
    <span>Loop closures <b>${s.loops}</b></span><span>Confidence <b>${Math.round(s.confidence*100)}%</b></span>`:'';
  $('lag').firstElementChild.style.width=`${Math.min(100,lag*4)}%`;
  $('lag').firstElementChild.style.background=lag>20?'#ed8b75':'#53e2b2';
}
function setMode(next){
  mode=next;$('follow').classList.toggle('on',mode==='follow');$('fit').classList.toggle('on',mode==='fit');
}
controls.addEventListener('start',()=>{if(mode==='fit')setMode('free');});
$('follow').onclick=()=>{
  if(mode==='follow'){setMode('free');return;}
  setMode('follow');
  if(lastPose){const p=toThree(lastPose.x,lastPose.y,lastPose.z),delta=p.clone().sub(controls.target);camera.position.add(delta);controls.target.copy(p);}
};
$('graph').onclick=e=>{showGraph=!showGraph;graphGroup.visible=showGraph;e.currentTarget.classList.toggle('on',showGraph);};
$('fit').onclick=()=>{setMode('fit');fit();};
$('stop').onclick=()=>{if(confirm('Stop live capture?'))socket?.send(JSON.stringify({type:'command',action:'stop'}));};
renderer.setAnimationLoop(()=>{controls.update();renderer.render(scene,camera);});
connect();render();
