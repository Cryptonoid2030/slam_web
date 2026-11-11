import './style.css';
import './diagnostics.css';
import {poseSnapshot} from './trajectory';
import { World } from './world';
import { Runtime,type Metrics } from './runtime';
import { sourceFor,setSource,nodeRuntimeLabel,type GraphNode } from './graph';
import {FLIGHT_TOURS,isFlightTour,selectedFlightTour,selectMapFlightTour} from './flight-tour';
import { algorithms,detectors,visibleDetectorPresets,defaultProject,readSavedProjectText,readProjectRecovery,saveProjectRecovery,withDefaultModels,estimatorModelicaSources,estimatorExampleSources,type ProjectRecoveryBackup,saveProject,parseProject,download,offsetQuadrotorPath,offsetQuadrotorSource } from './project';
import {mountModelParameters} from './model-parameters-panel';
import {modelParameterTargets,parameterTargetVersion} from './model-parameter-targets';
import {nativeParametersFor,NATIVE_SLAM_PARAMETERS,type ParameterValues} from './model-parameters';
import type { Estimate,Pose,RunRecord,SensorFrame,Truth } from './types';
import {installLiveCapture} from './live-controller';
import { uncertaintySummary } from './covariance';
import {ComparisonTracker,type TimedPose} from './comparison';
import {validateSensorFrame} from './packet';
import {createSourceEditor} from './source-editor';
import {NavigationHandoff} from './world-navigation';
import {BrowserPerformanceMonitor} from './performance-monitor';
import {WorkerViewer} from './worker-viewer';
import {GRAPHICS_DESCRIPTIONS} from './graphics-quality';
import {ViewerCameraControls,VIEWER_KEYS,isTextEntry} from './viewer-camera-controls';
import {CAMERA_VIEWS,nextCameraView,type CameraView} from './viewer-camera';
import {mountConfigurationPanel} from './configuration-panel';
import {sensorRates,type SensorRates} from './sensor-clock';
import {createRGBDSlamWorkspace,editRGBDSlamWorkspace,rgbdSlamWorkspaceManifest} from './modelica-slam-workspace';
import {checkRGBDSlamBuild,type SlamBuildReceipt} from './modelica-slam-build';
import {DepthPreview} from './depth-preview';
import {ColorPreview} from './color-preview';
import {createSourceExplorer} from './source-explorer';
import {mountWorkspacePanes} from './workspace-panes';
import {startupScreen} from './startup-screen';
import {STARTUP_COMPILE_DEADLINE_MS,VIEWER_READY_DEADLINE_MS,withDeadline} from './startup-deadline';
import {createModelicaDocs} from './modelica-docs';
import {createAssistantPanel} from './assistant-panel';
import {runAssistantSimulation} from './assistant-simulation';
import {sceneLandmarks} from './scene-landmarks';
import {mountTelemetryPlot} from './telemetry-plot';
import {LoopClosureHistory} from './loop-closure-history';
import {mountLoopClosurePanel} from './loop-closure-panel';
import {useNativeDefault,supportsWasmSimd} from './estimator-default';
import {fetchNativeManifest} from './slam-native-program';
import {nativeManifestUrl} from './native-assets';
import './loop-closure.css';
import './modelica-docs.css';
import './workspace.css';

const startup=startupScreen();
document.querySelector('#app')!.innerHTML=`
<header><a class="brand" href="https://github.com/CogniPilot/slam_web" target="_blank" rel="noopener noreferrer" aria-label="SLAM Lab GitHub repository"><span class="brand-mark">⌘</span><span>SLAM<span class="brand-light">LAB</span><small>UAV robotics · in your browser</small></span></a><div class="project-name"><input id="name" aria-label="Project name"><span id="saved">Local project</span></div><div class="project-actions"><button id="save">Save project</button><label class="button">Open project<input id="open" type="file" accept=".json,.slam.json" hidden></label><button id="download">Download</button><button id="default-models" title="Keep settings and back up source before loading current default models">Load default models</button></div><a class="powered-by" href="https://github.com/CogniPilot/rumoca" target="_blank" rel="noopener noreferrer" aria-label="Powered by Rumoca" title="Rumoca Modelica compiler"><span>Powered by</span><img src="${import.meta.env.BASE_URL}brand/rumoca.svg" width="28" height="28" alt=""></a></header>
<main><div class="toolbar"><div class="run-controls"><button id="run" class="primary" disabled>▶ Run</button><button id="step" disabled>Step</button><button id="reset" disabled>↺ Reset</button><select id="speed" aria-label="Simulation speed"><option value="1">1× real time</option><option value="4">4× real time</option><option value="10">10× real time</option><option value="Infinity">As fast as possible</option></select><span id="estimator-mode" class="estimator-mode" role="status">Estimator: preparing</span><button id="flight-control" aria-pressed="true" aria-label="Manual flight">Manual flight</button><select id="active-vehicle" aria-label="Controlled and followed drone" hidden><option value="0">Drone 1</option><option value="1">Drone 2</option></select></div><div class="scene-controls"><select id="environment" aria-label="Environment"><option value="city">City blocks</option><option value="warehouse">Warehouse</option><option value="hallway">Long hallway · repetitive</option><option value="hallway-marked">Long hallway · distinct landmarks</option><option value="office-loop">Office · closed loop</option><option value="courtyard">Courtyard</option></select><label class="check"><input id="show-map" type="checkbox" checked>Show map</label><button id="help">Lesson guide</button></div></div>
<div class="workspace"><section class="visuals"><div class="scene panel"><div class="panel-label"><span class="live-dot"></span>WORLD VIEW<span class="right">Three.js · orbit to explore</span></div><div id="world"></div><div class="scene-overlay"><span>QUADROTOR + D435</span><small id="flight-status">Initializing browser runtimes</small></div><div class="scene-bottom">WASD move · Q/E yaw · R/F altitude <span id="sim-time">t = 0.00 s</span></div></div>
<div class="sensor-row"><div class="panel sensor"><div class="panel-label">RGB CAMERA<span class="right">features overlay</span></div><canvas id="rgb" width="160" height="90"></canvas><div class="sensor-foot">69° × 42° · RGBA8</div></div><div class="panel sensor"><div class="panel-label">DEPTH CAMERA<span class="right">axial depth, meters</span></div><canvas id="depth" width="160" height="90"></canvas><div class="sensor-foot">87° × 58° · 0.28–10 m</div></div><div class="panel trajectory"><div class="panel-label">TRAJECTORY<span class="right">ENU · meters</span></div><canvas id="trajectory" width="260" height="146"></canvas><div class="legend"><span class="truth-key">Truth</span><span class="estimate-key">Estimate</span><span class="compare-key">External</span></div></div></div>
<div class="metrics"><div><small>SIM TIME</small><strong id="metric-time">0.00 <em>s</em></strong></div><div><small>TRAJECTORY RMSE</small><strong id="metric-ate">— <em>m</em></strong></div><div><small>FEATURES</small><strong id="metric-features">0</strong></div><div><small>MAP POINTS</small><strong id="metric-points">0</strong></div><div><small>SIM / WALL TIME</small><strong id="metric-rtf">— <em>×</em></strong></div></div></section>
<aside class="panel editor"><div class="editor-top"><button id="files-toggle" aria-controls="source-explorer" aria-expanded="false">Files</button><div><small>MODELICA SOURCE</small><h2 id="editor-title">RGB-D / inertial SLAM</h2></div><span id="language" class="badge">Modelica · Rumoca</span></div><div class="editor-selects"><label>Preset <select id="preset" aria-label="Feature detector preset"></select></label></div><p id="editor-description">Estimate a 6-DOF pose from camera images and the airframe IMU.</p><textarea id="code" aria-label="Modelica source" spellcheck="false"></textarea><div class="editor-bottom"><span id="line-count"></span><button id="apply" disabled class="primary">Apply & reset</button></div></aside></div>
<div class="bottom-row"><section class="panel connection"><h2>Embedded deployment</h2><p>The SLAM model compiles to an FMI-LS WebAssembly package (FMI 3 co-simulation with a wasm32-wasip2 component) that runs on embedded hosts; ROS 2 / MAVLink adapters are under development.</p><a id="download-fmu" class="button" href="#" download aria-disabled="true">Download SLAM FMU</a><button id="deploy" disabled>Deploy project</button><small id="deploy-state">Checking published packages…</small></section><section class="panel recordings"><h2>Experiments</h2><p>Record synchronized sensor frames, then rerun them through edited source.</p><button id="record" disabled>● Record dataset</button><button id="export-recording" disabled>Export dataset</button><label class="button">Replay dataset<input id="replay" type="file" accept=".json" hidden></label><button id="export-node">Export source</button><small id="record-state">Up to 300 RGB-D / IMU frames per recording</small></section></div>
<div id="status" role="status">Loading Modelica environment…</div><details id="lesson" class="panel lesson"><summary>Lesson guide · Modelica robotics</summary><ol><li><b>Run the default experiment.</b> Modelica inertial propagation prepares automatically. New projects start in the city with Economical graphics and 424×240 RGB-D. Click Run to start; no initial Apply is needed. Test physics and camera data. This inertial baseline has no visual localization, map or loop closure. For RGB-D mapping on a desktop, open Configuration and select Native Modelica SLAM under Estimator. Use Parameters to tune loop retrieval and verification without clearing the map. The Loop closures panel shows committed loops and rejected candidate images with Modelica reasons.</li><li><b>Edit Modelica.</b> Physics, detector and estimator source use Rumoca diagnostics and highlighting. Apply resets the experiment.</li><li><b>Save your work.</b> Projects save locally and can be downloaded and reopened. The estimator backend and sensor settings are saved with the project.</li></ol><p>Select sensor rates in Configuration. RGB and depth share a simulation clock; IMU, GPS and LiDAR use their selected rates. Physics waits for every sensor event before advancing. Three.js presents the world independently at a target of 30 FPS. Select a paired D435 resolution in Configuration. The camera uses pinhole optics with stereo disparity noise; the D435 has no IMU, so IMU measurements belong to the airframe. Traffic, pedestrians and lighting vary the scene. Open the site to use the browser environment.</p></details></main>`;

const el=<T extends HTMLElement=HTMLElement>(id:string)=>document.getElementById(id)! as T;
el('environment').insertAdjacentHTML('afterend','<select id="scene-detail" aria-label="Graphics quality" aria-describedby="graphics-budget"><option value="low">Low · economical</option><option value="medium">Medium · balanced</option><option value="high">High · showcase</option></select>');
document.querySelector('.toolbar')!.insertAdjacentHTML('afterend','<p id="graphics-budget" class="graphics-budget"></p>');
el('scene-detail').insertAdjacentHTML('afterend','<select id="lighting-mode" aria-label="Lighting"><option value="day">Daylight</option><option value="night">Moonlight</option><option value="cycle">Day / night cycle</option></select>');
el('environment').insertAdjacentHTML('beforeend','<option value="tokyo">Littlest Tokyo · animated</option><option value="asset-city">Asset city</option><option value="big-city">Big city · furnished interiors</option>');
el('scene-detail').title='Controls viewer resolution, textures, shadows, clouds and scene detail. Appearance also changes sensor imagery. Sensor rates follow quality defaults unless customized.';
el('environment').insertAdjacentHTML('afterend','<select id="interior-view" aria-label="Inspect Big city" title="Move the viewer camera; simulation and sensor pose stay on the flight path" hidden><option value="overview">City overview</option><option value="store">Corner Market</option><option value="apartment">Daylight Loft</option><option value="conference">Civic Workspace</option></select>');
el('scene-detail').insertAdjacentHTML('afterend','<label class="check"><input id="cutaway-3d" type="checkbox">Cutaway 3D</label>');
el('scene-detail').insertAdjacentHTML('afterend','<span id="sensor-rate-summary" class="badge"></span><label class="check"><input id="lidar-enabled" type="checkbox">64-beam LiDAR</label><label class="check"><input id="show-paths" type="checkbox" checked>3D paths</label>');
el('flight-control').insertAdjacentHTML('afterend','<select id="tour-mode" aria-label="Flight tour route"></select>');
for(const route of FLIGHT_TOURS)el<HTMLSelectElement>('tour-mode').add(new Option(route.label,route.id));
el('lidar-enabled').parentElement!.insertAdjacentHTML('afterend','<label class="check"><input id="cars-enabled" type="checkbox" checked>Cars</label><label class="check"><input id="people-enabled" type="checkbox" checked>People</label>');
el('lidar-enabled').parentElement!.insertAdjacentHTML('afterend','<label class="check"><input id="depth-cloud-enabled" type="checkbox">Raw depth cloud</label>');
document.querySelector('.scene .panel-label .right')!.innerHTML='<span id="world-fps">— FPS · target 30</span> · Three.js';
const cutawayLabel=el('cutaway-3d').closest('label')!;cutawayLabel.title='3D view only: open ceiling, darker floor, and a clear opening through walls to the selected drone';document.querySelector('.scene .panel-label')!.append(cutawayLabel);
document.querySelector('.metrics')!.insertAdjacentHTML('afterend','<details class="panel estimator-diagnostics" open><summary>Estimator diagnostics</summary><div class="diagnostic-controls"><label class="check"><input id="show-uncertainty" type="checkbox" checked>2σ position ellipsoid</label><label class="check"><input id="show-graph" type="checkbox" checked>Keyframes & loop edges</label><label class="check"><input id="show-matches" type="checkbox" checked>Feature matches</label></div><div id="tracking-state">Waiting for observations</div><div id="uncertainty-state">Covariance unavailable</div><div id="graph-state">No keyframes yet · loop edges are orange</div><div id="frame-timings"></div><small>Matches connect reference → current RGB pixels. Covariance comes from the estimator; the visual frontend approximates correlations.</small></details>');
el('deploy-state').insertAdjacentHTML('afterend','<div id="comparisons"></div><small>External comparison adapters are under development.</small>');
// The published package list is small; read it once so the download reflects this site.
void (async()=>{
  const link=el<HTMLAnchorElement>('download-fmu'),state=el('deploy-state');
  try{
    const response=await fetch(nativeManifestUrl(new URL(import.meta.env.BASE_URL,location.href)));
    if(!response.ok)throw new Error('unpublished');
    const manifest=await response.json();
    const fmu=manifest?.fmu;
    if(!fmu?.file)throw new Error('unpublished');
    link.href=new URL(`vendor/slam/${fmu.file}`,new URL(import.meta.env.BASE_URL,location.href)).href;link.download=fmu.name??'D435FastSLAMStep.fmu';link.removeAttribute('aria-disabled');
    state.textContent=`${fmu.model} · FMI-LS WebAssembly · ${(fmu.bytes/1e6).toFixed(1)} MB · source ${String(manifest.sourceSha256).slice(0,12)} · embedded host integration pending`;
  }catch{
    link.setAttribute('aria-disabled','true');link.onclick=event=>event.preventDefault();
    state.textContent='SLAM FMU not published on this site · embedded host integration pending';
  }
})();
const status=(message:string,error=false)=>{el('status').textContent=message;el('status').classList.toggle('error',error);};
const mobileDefaults=matchMedia('(max-width:760px)').matches;
let project=defaultProject(mobileDefaults);
let selected=project.graph.nodes.find(n=>n.kind==='slam')!;
const world=new World(el('world'));
document.querySelector('.visuals')!.insertAdjacentHTML('afterbegin','<div id="render-warning" class="render-warning" role="status" hidden></div>');
if(world.graphics.acceleration==='software'){
  const camera=world.calibration;
  el('render-warning').hidden=false;
  el('render-warning').textContent=`Software rendering detected: graphics are running on the CPU. Reduced preview: ${camera.width} × ${camera.height} RGB-D and 64 × 128 LiDAR, with graphics below Low. Enable browser hardware acceleration for full quality.`;
}
const graphicsDescription=()=>world.sensorProfile==='software'?'Software fallback · tiny sensor images · reduced viewer resolution · no shadows or normal maps':GRAPHICS_DESCRIPTIONS[project.sceneDetail??'high'];
el('frame-timings').insertAdjacentHTML('beforebegin','<div id="graphics-state"></div>');
el('frame-timings').insertAdjacentHTML('afterend','<button id="copy-stats" type="button">Copy stats</button>');
// One pasteable line so a phone run can report where its frame time goes.
el('copy-stats').onclick=async()=>{
  const text=(id:string)=>document.getElementById(id)?.textContent?.trim()??'';
  const line=[`SLAM Lab ${location.href}`,`sim ${text('metric-time')} · sim/wall ${text('metric-rtf')} · devicePixelRatio ${devicePixelRatio}`,
    `camera ${world.calibration.width}x${world.calibration.height} · sensor profile ${world.sensorProfile}`,
    ...['estimator-mode','frame-timings','performance-state','graphics-state','graph-state','uncertainty-state','navigation-state'].map(text),
    Object.entries(latest?.estimate.diagnostics??{}).filter(([name,value])=>typeof value==='number'&&/keyframe|generation|vocabulary|accepted|rejection/i.test(name))
      .map(([name,value])=>`${name}=${value}`).join(' ')].filter(Boolean).join(' | ');
  try{await navigator.clipboard.writeText(line);status('Stats copied');}
  catch{status(`Copy unavailable; stats: ${line}`);}
};
el('graphics-state').textContent=`Renderer: ${world.graphics.api} · ${world.graphics.renderer} · ${world.graphics.acceleration==='software'?'software rendering':world.graphics.acceleration==='hardware-reported'?'hardware driver reported':'driver acceleration unknown'}`;
el('graphics-state').insertAdjacentHTML('afterend','<small id="scene-credit"></small>');
el('scene-credit').insertAdjacentHTML('afterend','<div id="navigation-state">Navigation · GPS roof geofence demonstration</div><div id="lidar-state">LiDAR disabled</div>');
const runtime=new Runtime(world);
const loopHistories=[new LoopClosureHistory(),new LoopClosureHistory()];
const loopPanel=mountLoopClosurePanel(document.querySelector<HTMLElement>('.visuals')!);
loopPanel.render(loopHistories[0]);
runtime.onInitialFrame=(index,frame,estimate)=>{loopHistories[index].reset();loopHistories[index].observe(frame,estimate);};
const depthPreview=new DepthPreview(el<HTMLCanvasElement>('depth'));
const colorPreview=new ColorPreview();
let pendingSensorPreview:{frame:SensorFrame;estimate:Estimate}|undefined;
let finishInitialization!:()=>void;
const initializationReady=new Promise<void>(resolve=>{finishInitialization=resolve;});
let initialized=false;
const navigation=new NavigationHandoff();
let viewer:WorkerViewer|undefined;
el<HTMLInputElement>('cutaway-3d').onchange=()=>{world.viewerCutaway=el<HTMLInputElement>('cutaway-3d').checked;viewer?.cutaway(world.viewerCutaway);};
let viewerPresented=true;
el('graphics-state').insertAdjacentHTML('afterend','<div id="performance-state"></div>');
const performanceMonitor=new BrowserPerformanceMonitor({targetFps:30,onUpdate:s=>{
  syncEstimatorMode();
  const display=viewer?.latest??s;
  el('world-fps').textContent=`${display.fps.toFixed(0)} FPS · target 30${viewer?' · worker':''}`;
  el('performance-state').textContent=`RGB-D input ${s.captureHz.toFixed(1)} Hz real time · Estimation ${s.visionHz.toFixed(1)} Hz real time · LiDAR ${s.lidarHz.toFixed(1)} Hz real time · lockstep cameras ${sensorRates(project).cameraHz} Hz sim time · render submission ${display.meanRenderSubmitMs?.toFixed(1)??'—'} ms · display interval p95 ${display.p95FrameIntervalMs?.toFixed(1)??'—'} ms${s.longTasks?` · main-thread stalls ${s.longTasks.count} / ${s.longTasks.blockedMs.toFixed(0)} ms`:''}`;
}});
runtime.onSensor=()=>performanceMonitor.captured();runtime.onLidar=()=>performanceMonitor.lidarCaptured();
function syncEstimatorMode(){
  const backend=compiling||!initialized?'preparing':!ready?'unavailable':runtime.activeVehicle.slamProgram?'native':'inertial';
  const chip=el('estimator-mode');
  if(chip.dataset.backend===backend)return;
  chip.dataset.backend=backend;
  chip.textContent=backend==='native'?'Estimator: RGB-D SLAM':backend==='inertial'?'Estimator: inertial baseline':`Estimator: ${backend}`;
  chip.title=backend==='native'?'Visual odometry, mapping and loop closure enabled':'Choose Native Modelica SLAM in Configuration for visual mapping and loop closure';
  const initial=runtime.activeVehicle.slamProgram?.estimate.diagnostics;
  loopPanel.render(loopHistories[runtime.activeVehicleIndex],Number(initial?.loopVocabularyReady),
    typeof initial?.loopWords==='number'?initial.loopWords:undefined,backend==='native');
}

const code=el<HTMLTextAreaElement>('code');
const configuration=mountConfigurationPanel(document.querySelector<HTMLElement>('aside.editor')!,{project:()=>project,onRatesChange:changeSensorRates,onResolutionChange:changeCameraResolution,camera:()=>({...world.calibration,software:world.sensorProfile==='software'})});
const parameterButton=document.createElement('button');parameterButton.id='parameters';parameterButton.textContent='Parameters';parameterButton.onclick=()=>showParameters();
document.querySelector('.run-controls')!.append(parameterButton);
function showParameters(){workspacePanes.reveal('editor');parameterView.show();}
const sourceEditor=createSourceEditor(code);
const parameterPanel=mountModelParameters({
  targets:()=>modelParameterTargets(project),
  values:id=>id===NATIVE_SLAM_PARAMETERS?nativeParametersFor(project):project.modelParameters?.[id]??{},
  load:async id=>{
    const target=modelParameterTargets(project).find(target=>target.id===id);if(!target)throw Error('Unknown parameter model');
    const version=parameterTargetVersion(project,id);
    if(target.native){
      const manifest=await fetchNativeManifest(new URL(import.meta.env.BASE_URL,location.href),project.cameraResolution??'848x480');
      const fields=manifest.models.step.parameters;
      if(!fields?.length)throw Error('This native SLAM delivery has no model parameter catalog. Load the latest app to inspect its parameters.');
      return {fields,version};
    }
    return {fields:await sourceEditor.parameters(target.source,target.model,target.sources),version};
  },
  apply:changeModelParameters,
  source:id=>{
    if(id===NATIVE_SLAM_PARAMETERS){void openSource('library:models/Libraries/CogniPilot/SLAM/RGBDFastSLAMInterface.mo');return;}
    const kind=id==='LabQuadrotor'?'physics':id==='RuntimeEvaluation'?'evaluation':['SensorObservations','SensorAvailability','ActorMotion'].includes(id)?'sensor':'slam';
    configuration.showEditor();selectNode(project.graph.nodes.find(node=>node.kind===kind)!);
  }
});
const parameterView=configuration.addPanel('parameters','Model parameters',parameterPanel.element);
parameterPanel.element.addEventListener('workspace:show',()=>{void parameterPanel.show();});
const runModelLabel=document.createElement('label');runModelLabel.className='run-model';runModelLabel.textContent='Inertial baseline model';
const runModel=document.createElement('select');runModel.id='run-model';runModel.setAttribute('aria-label','Run model');
runModelLabel.append(runModel);document.querySelector('.configuration-flight')!.append(runModelLabel);
let modelDiscovery=0,modelDiscoveryTimer:ReturnType<typeof setTimeout>|undefined;
async function refreshRunModels(){
  const token=++modelDiscovery,owner=project,source=project.algorithm,companions=project.modelicaSources;
  const entryPoint=project.entryPoint??'ModelicaInertial';
  if(!Array.from(runModel.options).some(option=>option.value===entryPoint))runModel.add(new Option(entryPoint,entryPoint));
  runModel.value=entryPoint;
  try{
    const models=await sourceEditor.simulationModels(source,entryPoint,estimatorExampleSources(project));
    if(token!==modelDiscovery||owner!==project||source!==project.algorithm||companions!==project.modelicaSources)return;
    const labels:Record<string,string>={'SLAM.Examples.InertialOnly':'Inertial navigation','SLAM.Examples.SmoothedInertial':'Inertial navigation · smoothed IMU','SLAM.Examples.ResponsiveInertial':'Inertial navigation · responsive IMU'};
    runModel.replaceChildren(...models.map(name=>new Option(labels[name]??name,name)));
    if(!models.includes(entryPoint)){
      const missing=new Option(entryPoint+' · missing from source',entryPoint);missing.disabled=true;runModel.add(missing);
    }
    runModel.value=entryPoint;runModel.title='Inertial estimator examples and models defined in your main source. RGB-D SLAM uses its own native model.';
  }catch(error){if(token===modelDiscovery)runModel.title=String(error);}
}
runModel.onchange=()=>{
  ++modelDiscovery;
  project.entryPoint=runModel.value;delete project.algorithmArtifact;
  changed();status(`Selected ${project.entryPoint} · click Run to compile and start`);
};
const explorer=createSourceExplorer(id=>{void openSource(id);});
const workbench=document.createElement('div');workbench.className='editor-workbench';
const sourceDocument=document.createElement('div');sourceDocument.className='source-document';
const editorView=el('editor-view');
sourceDocument.append(...Array.from(editorView.children).filter(child=>!child.classList.contains('editor-top')));
workbench.append(explorer.element,sourceDocument);editorView.append(workbench);
function showFiles(show:boolean){
  workbench.classList.toggle('files-open',show);
  explorer.element.hidden=!show;
  el('files-toggle').setAttribute('aria-expanded',String(show));
}
showFiles(!mobileDefaults);
el('files-toggle').onclick=()=>showFiles(explorer.element.hidden);
function refreshFiles(){
  const manifest=rgbdSlamWorkspaceManifest(project.slamWorkspace?.schemaVersion??2);
  explorer.update([
    ...project.graph.nodes.filter(node=>node.kind!=='map').map(node=>({id:'experiment:'+node.id,path:node.kind==='slam'&&project.mainSourcePath?project.mainSourcePath.slice('models/'.length):'Experiment/'+({physics:'Quadrotor',sensor:'Sensors',detector:'FeatureDetector',slam:'Main',evaluation:'Evaluation',map:'Map',modelica:node.id}[node.kind])+'.mo'})),
    ...Object.keys(project.modelicaSources??{}).map(path=>({id:'library:'+path,path:path.slice('models/'.length)})),
    ...manifest.paths.map(path=>({id:path,path:'SLAM sources/'+path.slice('models/'.length)}))
  ],viewingSlamFile()?activeSlamFile:viewingLibraryFile()?'library:'+activeLibraryFile:'experiment:'+selected.id);
}
const telemetryPlot=mountTelemetryPlot(document.querySelector('.visuals')!);
const workspacePanes=mountWorkspacePanes([
  {name:'viewer',panel:document.querySelector('.scene')!,header:document.querySelector('.scene .panel-label')!,fullScreenControls:document.querySelector<HTMLElement>('.run-controls')!},
  {name:'editor',panel:document.querySelector('aside.editor')!,header:document.querySelector('.configuration-tabs')!,fullScreenControls:document.querySelector<HTMLElement>('.run-controls')!},
  {name:'plot',panel:telemetryPlot.panel,header:telemetryPlot.header,fullScreenControls:document.querySelector<HTMLElement>('.run-controls')!}
],()=>{
  viewerPresented=!document.querySelector('.scene.pane-collapsed, aside.editor.pane-fullscreen, .telemetry-plot.pane-fullscreen');
  viewer?.presentationVisible(viewerPresented);
});
const slamBuildPanel=document.createElement('div');slamBuildPanel.className='slam-source-build';slamBuildPanel.hidden=true;
slamBuildPanel.innerHTML='<button id="check-slam-build">Check WASM build</button> <button id="cancel-slam-build" hidden>Cancel build</button><pre id="slam-build-result" role="status">Checks all saved SLAM sources using Rumoca in this browser. Execution integration is pending.</pre>';
document.querySelector('.editor-selects')!.after(slamBuildPanel);
let slamBuildAbort:AbortController|undefined;
let slamBuildReceipt:SlamBuildReceipt|undefined;
el('check-slam-build').onclick=async()=>{
  if(slamBuildAbort||!project.slamWorkspace)return;
  const workspace=project.slamWorkspace,owner=project,abort=new AbortController();slamBuildAbort=abort;slamBuildReceipt=undefined;
  const result=el('slam-build-result');
  el<HTMLButtonElement>('check-slam-build').disabled=true;el('cancel-slam-build').hidden=false;
  runtime.pause();el('run').textContent='▶ Run';
  result.textContent='Preparing saved source snapshot…';
  try{
    const receipt=await checkRGBDSlamBuild(workspace,{base:new URL(import.meta.env.BASE_URL,location.href).href,signal:abort.signal,
      onProgress:progress=>{result.textContent=progress.phase==='loading'?'Loading Rumoca WASM…':`${progress.phase==='compiling'?'Compiling':'Checking executable'} ${progress.model}…`;}});
    slamBuildReceipt=receipt;
    const identity=`Source ${receipt.sourceSha256.slice(0,12)} · Rumoca ${receipt.compiler?.version??'unavailable'}${receipt.compiler?' / '+receipt.compiler.revision:''}`;
    const stale=project!==owner||project.slamWorkspace!==workspace?'\nSource changed during this build; check again.':'';
    result.textContent=receipt.status==='pass'?`WASM build check passed (${receipt.programs.length} programs). Numerical execution integration is pending.\n${identity}${stale}`
      :`WASM build failed${receipt.failedAt?.model?' · '+receipt.failedAt.model:''}\n${receipt.error}\n${identity}${stale}`;
  }catch(error){result.textContent=abort.signal.aborted?'Build cancelled.':`WASM build failed: ${String(error)}`;}
  finally{slamBuildAbort=undefined;el<HTMLButtonElement>('check-slam-build').disabled=false;el('cancel-slam-build').hidden=true;slamBuildPanel.hidden=!viewingSlamFile();}
};
el('cancel-slam-build').onclick=()=>slamBuildAbort?.abort();
window.addEventListener('pagehide',()=>slamBuildAbort?.abort());
let activeSlamFile='',activeLibraryFile='',slamFileLoad=0;
const viewingSlamFile=()=>selected.kind==='slam'&&!!activeSlamFile&&!!project.slamWorkspace;
const viewingLibraryFile=()=>selected.kind==='slam'&&!!activeLibraryFile&&Object.hasOwn(project.modelicaSources??{},activeLibraryFile);
const selectedSource=()=>viewingSlamFile()?project.slamWorkspace!.sources[activeSlamFile]:viewingLibraryFile()?project.modelicaSources![activeLibraryFile]:sourceFor(selected,project);
function experimentPath(node:GraphNode){
  if(node.kind==='slam')return project.mainSourcePath??'models/Main.mo';
  const paths:Partial<Record<GraphNode['kind'],string>>={physics:'models/Vehicles/LabQuadrotor.mo',sensor:'models/Sensors/SensorObservations.mo',detector:'models/Vision/Features/FeatureDetector.mo',evaluation:'models/Evaluation/RuntimeEvaluation.mo'};
  return paths[node.kind]??`models/Experiment/${node.id}.mo`;
}
function projectFiles():Record<string,string>{
  return {...project.modelicaSources,...project.slamWorkspace?.sources,
    ...Object.fromEntries(project.graph.nodes.filter(node=>node.kind!=='map').map(node=>[experimentPath(node),sourceFor(node,project)]))};
}
function sourceId(path:string){
  const node=project.graph.nodes.find(node=>experimentPath(node)===path);
  return node?'experiment:'+node.id:Object.hasOwn(project.modelicaSources??{},path)?'library:'+path:path;
}
const documentation=createModelicaDocs({
  sources:projectFiles,
  request:(sources,name)=>sourceEditor.documentation(sources,name),
  openSource:path=>{configuration.showEditor();void openSource(sourceId(path));}
});
configuration.addPanel('docs','Docs',documentation.element);
const assistant=createAssistantPanel({
  identity:()=>project,
  files:projectFiles,
  activePath:()=>viewingSlamFile()?activeSlamFile:viewingLibraryFile()?activeLibraryFile:experimentPath(selected),
  apply:(path,source)=>{
    if(compiling)throw new Error('Wait for the current compilation to finish.');
    const node=project.graph.nodes.find(node=>experimentPath(node)===path);
    if(node){activeSlamFile='';activeLibraryFile='';selectNode(node,false);}
    else if(Object.hasOwn(project.modelicaSources??{},path)){
      activeSlamFile='';activeLibraryFile=path;selectNode(project.graph.nodes.find(node=>node.kind==='slam')!,false);
    }else if(Object.hasOwn(project.slamWorkspace?.sources??{},path)){
      activeSlamFile=path;activeLibraryFile='';selectNode(project.graph.nodes.find(node=>node.kind==='slam')!,false);
    }else throw new Error('Unknown project file');
    code.value=source;code.dispatchEvent(new Event('input',{bubbles:true}));
  },
  diagnostics:()=>({path:viewingSlamFile()?activeSlamFile:viewingLibraryFile()?activeLibraryFile:experimentPath(selected),diagnostics:sourceEditor.getDiagnostics()}),
  compile:async()=>{
    const model=project.entryPoint??'ModelicaInertial',path=project.mainSourcePath??'models/Main.mo';
    return {scope:'selected runnable example',model,path,
      excluded:'Full SLAM workspace and other experiment sources are not checked.',
      result:await sourceEditor.checkModel(project.algorithm,model,estimatorModelicaSources(project))};
  },
  searchDocs:query=>documentation.search(query),
  readDocs:name=>documentation.read(name),
  simulation:assistantSimulationStatus,
  landmarks:environment=>sceneLandmarks(environment??project.environment),
  checkFile:async(path,model)=>{
    const sources={...estimatorModelicaSources(project),...(Object.hasOwn(project.slamWorkspace?.sources??{},path)?project.slamWorkspace?.sources:{})};delete sources[path];
    return {scope:'named Modelica component',path,model,result:await sourceEditor.checkModel(projectFiles()[path],model,sources)};
  },
  selectEnvironment:async environment=>{
    if(compiling||assistantSimulation)throw new Error('Wait for the current operation to finish.');
    pauseForAssistant();project.environment=environment as typeof project.environment;el<HTMLSelectElement>('environment').value=environment;
    changed();ready=false;disableRun(true);syncApplyButton();
    return {environment,status:'Selected for the next compilation. Modelica source and flight path are unchanged.'};
  },
  startSimulation:async signal=>{
    const revision=projectRevision;pauseForAssistant();
    const deadline=performance.now()+15_000;
    while(runtime.busy||compiling){signal.throwIfAborted();if(performance.now()>deadline)throw new Error('Simulation is busy. Retry when it finishes.');await new Promise(resolve=>setTimeout(resolve,20));}
    signal.throwIfAborted();if(projectRevision!==revision)throw new Error('Project changed. Start a new request.');
    if(dirty||!ready)await compile();
    signal.throwIfAborted();if(projectRevision!==revision||!ready||dirty)throw new Error('Selected example could not be prepared. Read diagnostics before retrying.');
    runtime.play();el('run').textContent='Ⅱ Pause';return {status:'Playback started',simulation:assistantSimulationStatus()};
  },
  pauseSimulation:pauseForAssistant,
  runSimulation:async(frames,signal)=>{
    if(assistantSimulation)throw new Error('An assistant simulation is already running.');
    try{
      return await runAssistantSimulation({
        pause:pauseForAssistant,busy:()=>runtime.busy||compiling,
        revision:()=>projectRevision,
        prepare:async()=>{
          if(dirty||!ready)await compile();
          if(!ready||dirty)throw new Error('Selected example could not be prepared. Read diagnostics before retrying.');
          assistantSimulation=true;disableRun(true);configuration.setBusy(true);
        },
        step:()=>runtime.step(),status:assistantSimulationStatus
      },frames,signal);
    }finally{
      assistantSimulation=false;configuration.setBusy(compiling);disableRun(!ready||compiling);syncApplyButton();
    }
  }
});
configuration.addPanel('assistant','Assistant',assistant.element);
window.addEventListener('pagehide',()=>assistant.dispose());
function syncApplyButton(){
  const button=el<HTMLButtonElement>('apply');
  button.disabled=compiling||viewingSlamFile();
  button.textContent=viewingSlamFile()?'Full SLAM execution pending':'Apply & reset';
}
let ready=false,dirty=false,compiling=false,localPersistenceBlocked=false,assistantSimulation=false,projectRevision=0;
let truthPath:Pose[]=[],estimatePath:Pose[]=[];
const comparisons=new ComparisonTracker();
let externalLatest:TimedPose|undefined;
let latest:{frame:SensorFrame;estimate:Estimate;truth?:Truth}|undefined;
let latestMetrics:Metrics|undefined;
function pauseForAssistant(){runtime.pause();el('run').textContent='▶ Run';}
function assistantSimulationStatus(){
  return {scope:'selected runnable example',model:project.entryPoint??'ModelicaInertial',time:runtime.time,running:runtime.running,busy:runtime.busy,ready,dirty,
    environment:project.environment,physicsPath:runtime.activeVehicleIndex===1?'models/Vehicles/OffsetQuadrotor.mo':'models/Vehicles/LabQuadrotor.mo',autopilot:runtime.autopilot,tourMode:runtime.tourMode,
    vehicleCount:runtime.vehicleCount,controlledVehicle:runtime.activeVehicleIndex+1,
    vehicles:Array.from({length:runtime.vehicleCount},(_,index)=>{const vehicle=runtime.getVehicle(index as 0|1),truth=vehicle.world.truth;return {id:index+1,time:vehicle.time,position:truth?[truth.x,truth.y,truth.z]:null,estimator:project.entryPoint??'ModelicaInertial'};}),
    sensorRates:sensorRates(project),pending:Object.fromEntries(runtime.pendingNodes),metrics:latestMetrics??null,
    truth:latest?.truth?{position:[latest.truth.x,latest.truth.y,latest.truth.z],quaternion:latest.truth.quaternion,velocity:latest.truth.velocity}:null,
    estimate:latest?{position:[latest.estimate.x,latest.estimate.y,latest.estimate.z],quaternion:latest.estimate.quaternion,confidence:latest.estimate.confidence,diagnostics:latest.estimate.diagnostics}:null};
}
let autosaveTimer:ReturnType<typeof setTimeout>|undefined;
function changed(requiresCompilation=true) {
  projectRevision++;
  runtime.pause();if(requiresCompilation)dirty=true;
  el('run').textContent='▶ Run';
  if(autosaveTimer)clearTimeout(autosaveTimer);
  autosaveTimer=setTimeout(()=>{autosaveTimer=undefined;if(localPersistenceBlocked)return;void saveProject(project).then(()=>{el('saved').textContent='Autosaved locally';}).catch(()=>{el('saved').textContent='Cannot save locally · download your project';});},350);
}
function syncProject() {
  slamBuildAbort?.abort();
  if(activeSlamFile&&!Object.hasOwn(project.slamWorkspace?.sources??{},activeSlamFile))activeSlamFile='';
  if(activeLibraryFile&&!Object.hasOwn(project.modelicaSources??{},activeLibraryFile))activeLibraryFile='';
  el<HTMLInputElement>('name').value=project.name;
  el<HTMLSelectElement>('environment').value=project.environment;
  runtime.setFlightTour(selectedFlightTour(project));el<HTMLSelectElement>('tour-mode').value=runtime.tourMode;
  el<HTMLSelectElement>('scene-detail').value=project.sceneDetail??'high';
  el('graphics-budget').textContent=graphicsDescription();
  el<HTMLInputElement>('lidar-enabled').checked=project.lidarEnabled??false;
  el<HTMLInputElement>('depth-cloud-enabled').checked=project.depthCloudEnabled??false;
  el<HTMLInputElement>('cars-enabled').checked=project.carsEnabled??true;
  el<HTMLInputElement>('people-enabled').checked=project.peopleEnabled??true;
  el<HTMLSelectElement>('lighting-mode').value=project.lightingMode??'day';
  configuration.sync(project);
  parameterPanel.sync();
  selectNode(project.graph.nodes.find(n=>n.id===selected?.id)??project.graph.nodes.find(n=>n.kind==='slam')!,false);
  return refreshRunModels();
}
function selectNode(node:GraphNode,showEditor=true) {
  ++slamFileLoad;
  if(showEditor)configuration.showEditor();
  selected=node;
  el('editor-title').textContent=node.title;
  el('language').textContent=nodeRuntimeLabel(node,project);
  const description:Record<string,string>={
    physics:'Quadrotor dynamics, controller and flight-tour setpoints.',
    sensor:'IMU/GPS observations and scene motion. Three.js renders camera images and depth noise at the simulation timestamps.',
    detector:'Image features in RGB pixel coordinates. Native feature detection integration is pending.',
    slam:'Modelica estimator and experiment composition.',
    map:'Estimated 3D point cloud display.',
    evaluation:'Trajectory and orientation errors in the initial reference frame.',
    modelica:'Editable Modelica source. Runtime integration is pending.'
  };
  el('editor-description').textContent=description[node.kind];
  code.value=selectedSource();code.readOnly=!['physics','sensor','detector','slam','evaluation','modelica'].includes(node.kind);
  if(code.readOnly&&node.kind!=='slam')code.value='// This built-in node is configured through its graph ports.\n// Connect an editable Modelica node to describe a transformation.';
  if(node.kind==='slam'){
    el('editor-title').textContent=viewingLibraryFile()?activeLibraryFile.slice('models/'.length):project.mainSourcePath?.slice('models/'.length)??'Main.mo';
    el('editor-description').textContent='Wire components in Modelica and choose the entry point with Run model. The current examples run inertial navigation; full RGB-D SLAM integration is pending.';
  }
  refreshFiles();
  slamBuildPanel.hidden=!viewingSlamFile()&&!slamBuildAbort;
  if(viewingSlamFile()){
    el('editor-title').textContent=activeSlamFile.slice('models/'.length);
    el('editor-description').textContent='Edit the full Modelica SLAM source workspace. All dependency files are saved with this project. Browser execution integration is pending; Select the inertial baseline model under Parameters, Flight; Native Modelica SLAM uses the exported model for the chosen camera resolution.';
  }
  const presets=el<HTMLSelectElement>('preset');presets.replaceChildren();
  const collection=node.kind==='detector'?visibleDetectorPresets(project):node.kind==='slam'?algorithms:{};
  for(const key of Object.keys(collection)){
    presets.add(new Option(key,key));
  }
  presets.add(new Option('Custom source','custom'));
  presets.value=node.kind==='detector'?project.detectorPreset:node.kind==='slam'?project.algorithmPreset:'custom';
  if(!presets.value)presets.value='custom';presets.disabled=compiling||viewingSlamFile()||!['slam','detector'].includes(node.kind);
  document.querySelector<HTMLElement>('.editor-selects')!.hidden=viewingSlamFile()||node.kind!=='detector';
  const library={...estimatorModelicaSources(project),[project.mainSourcePath??'models/Main.mo']:project.algorithm};
  sourceEditor.setWorkspaceSources(viewingSlamFile()?project.slamWorkspace!.sources:library,
    viewingSlamFile()?activeSlamFile:viewingLibraryFile()?activeLibraryFile:experimentPath(node));
  sourceEditor.setLanguage(code.readOnly?'plaintext':'modelica');
  sourceEditor.syncFromTextarea();
  el<HTMLButtonElement>('export-node').disabled=!['physics','sensor','detector','slam','evaluation','modelica'].includes(node.kind);
  lineCount();
  syncApplyButton();
}
function lineCount(){el('line-count').textContent=`${code.value.split('\n').length} lines · ${code.readOnly?'read-only source':'editable source'}`;}
code.addEventListener('input',()=>{
  if(viewingLibraryFile()){
    project.modelicaSources={...project.modelicaSources,[activeLibraryFile]:code.value};
    delete project.algorithmArtifact;
    if(modelDiscoveryTimer)clearTimeout(modelDiscoveryTimer);
    modelDiscoveryTimer=setTimeout(()=>{void refreshRunModels();},350);
    lineCount();changed();return;
  }
  if(viewingSlamFile()){
    project.slamWorkspace=editRGBDSlamWorkspace(project.slamWorkspace!,activeSlamFile,code.value);
    lineCount();changed(false);return;
  }
  setSource(selected,project,code.value);if(selected.kind==='slam'){
    project.algorithmPreset='custom';project.runtime='modelica';delete project.algorithmArtifact;
    if(modelDiscoveryTimer)clearTimeout(modelDiscoveryTimer);
    modelDiscoveryTimer=setTimeout(()=>{void refreshRunModels();},350);
  }if(selected.kind==='detector')project.detectorPreset='custom';el<HTMLSelectElement>('preset').value='custom';lineCount();changed();
});
async function openSource(id:string){
  if(id.startsWith('experiment:')){
    const node=project.graph.nodes.find(node=>node.id===id.slice('experiment:'.length));
    if(!node)return;
    activeSlamFile='';activeLibraryFile='';selectNode(node);
    if(mobileDefaults)showFiles(false);
    sourceEditor.editor.focus();return;
  }
  if(id.startsWith('library:')){
    const path=id.slice('library:'.length);
    if(!Object.hasOwn(project.modelicaSources??{},path))return;
    activeSlamFile='';activeLibraryFile=path;selectNode(project.graph.nodes.find(node=>node.kind==='slam')!);
    if(mobileDefaults)showFiles(false);
    sourceEditor.editor.focus();return;
  }
  const path=id,token=++slamFileLoad,owner=project;
  try{
    const workspace=owner.slamWorkspace??await createRGBDSlamWorkspace('d435-native');
    if(token!==slamFileLoad||owner!==project)return;
    if(!Object.hasOwn(workspace.sources,path))throw new Error('Unknown source file: '+path);
    if(!owner.slamWorkspace){owner.slamWorkspace=workspace;changed(false);}
    activeSlamFile=path;activeLibraryFile='';selectNode(project.graph.nodes.find(node=>node.kind==='slam')!);
    if(mobileDefaults)showFiles(false);
    sourceEditor.editor.focus();
  }catch(error){if(token===slamFileLoad&&owner===project)status(String(error),true);}
}
code.addEventListener('keydown',e=>{if(e.key==='Tab'&&!code.readOnly){e.preventDefault();const at=code.selectionStart;code.setRangeText('    ',at,code.selectionEnd,'end');code.dispatchEvent(new Event('input'));}});
el<HTMLSelectElement>('preset').onchange=async e=>{
  const value=(e.target as HTMLSelectElement).value;if(value==='custom')return;
  if(selected.kind==='detector'){project.detector=detectors[value];project.detectorPreset=value;project.detectorLanguage='modelica';delete project.detectorArtifact;}
  else {project.algorithm=algorithms[value];project.algorithmPreset=value;project.runtime='modelica';delete project.algorithmArtifact;}
  selectNode(selected);changed();
  await initializationReady;
  while(compiling||runtime.busy)await new Promise(resolve=>setTimeout(resolve,20));
  await compile();
  if(ready){runtime.play();el('run').textContent='Ⅱ Pause';}
};
el<HTMLInputElement>('name').oninput=e=>{project.name=(e.target as HTMLInputElement).value;changed();};
el<HTMLSelectElement>('environment').onchange=async e=>{
  selectMapFlightTour(project,(e.target as HTMLSelectElement).value as typeof project.environment);
  runtime.setFlightTour(selectedFlightTour(project));el<HTMLSelectElement>('tour-mode').value=runtime.tourMode;
  changed();ready=false;disableRun(true);
  await initializationReady;
  while(runtime.busy||compiling)await new Promise(resolve=>setTimeout(resolve,20));
  ready=false;
  compiling=true;configuration.setBusy(true);el<HTMLSelectElement>('environment').disabled=true;el<HTMLSelectElement>('scene-detail').disabled=true;
  try {await buildProjectScene();status('Environment updated · flight tour route selected · press Reset to prepare');}
  catch(error){status(String(error),true);}
  finally {compiling=false;configuration.setBusy(false);disableRun(true);syncApplyButton();el<HTMLButtonElement>('reset').disabled=false;el<HTMLSelectElement>('environment').disabled=false;el<HTMLSelectElement>('scene-detail').disabled=false;}
};
function syncSceneControls() {
  el('interior-view').hidden=project.environment!=='big-city';el<HTMLSelectElement>('interior-view').value='overview';
  el('scene-credit').innerHTML=project.environment==='tokyo'?'<a href="models/TOKYO-LICENSE.md" target="_blank" rel="noopener">Littlest Tokyo · Glen Fox · CC Attribution</a>':(['city','asset-city','big-city'].includes(project.environment)?'<a href="models/licenses/ACTORS.md" target="_blank" rel="noopener">Actor credits & licenses</a>':'');
  if(project.environment==='big-city')el('scene-credit').insertAdjacentHTML('beforeend',' · Original city & furnished interiors · CC0 surface textures');
  if(project.environment==='asset-city')el('scene-credit').insertAdjacentHTML('beforeend',' · <a href="models/city/ATTRIBUTION.txt" target="_blank" rel="noopener">Buildings · J-Toastie · CC BY 3.0</a>');
}
async function buildProjectScene() {
  world.build(project.environment,project.sceneDetail??'high');await world.ready;
  world.configureActors(project.carsEnabled??true,project.peopleEnabled??true);
  world.setDepthCloudEnabled(project.depthCloudEnabled??false);world.setLighting(project.lightingMode??'day');
  syncSceneControls();
}
el<HTMLSelectElement>('interior-view').onchange=e=>{setCameraView('free');world.inspectInterior((e.target as HTMLSelectElement).value as 'overview'|'store'|'apartment'|'conference');};
el<HTMLSelectElement>('scene-detail').onchange=async e=>{
  project.sceneDetail=(e.target as HTMLSelectElement).value as 'low'|'medium'|'high';changed(false);
  configuration.sync(project);
  await initializationReady;
  if(compiling){dirty=true;return;}
  compiling=true;disableRun(true);el<HTMLSelectElement>('scene-detail').disabled=true;
  configuration.setBusy(true);
  try{
    while(runtime.busy)await new Promise(resolve=>setTimeout(resolve,20));
    await runtime.setGraphicsQuality(project.sceneDetail);
    el('graphics-budget').textContent=graphicsDescription();
    status(ready?'Graphics quality updated · run preserved · press Run to continue':'Graphics quality updated · select an available experiment to run');
  }catch(error){ready=false;dirty=true;status(String(error),true);}
  finally{
    compiling=false;configuration.setBusy(false);disableRun(!ready);syncApplyButton();el<HTMLButtonElement>('reset').disabled=false;el<HTMLSelectElement>('scene-detail').disabled=false;
  }
};
async function changeCameraResolution(resolution:NonNullable<typeof project.cameraResolution>) {
  project.cameraResolution=resolution;
  changed();
  await compile();
  configuration.sync(project);
}
async function changeModelParameters(id:string,values:ParameterValues,version:string){
  await initializationReady;
  if(assistantSimulation)throw Error('Finish the current simulation before changing parameters');
  if(version!==parameterTargetVersion(project,id))throw Error('The model source changed. Refresh its parameters before applying edits.');
  configuration.setBusy(true);parameterPanel.setBusy(true);
  runtime.pause();el('run').textContent='▶ Run';disableRun(true);
  try{
    while(runtime.busy||compiling)await new Promise(resolve=>setTimeout(resolve,20));
    const previous=project.modelParameters;
    if(id===NATIVE_SLAM_PARAMETERS){
      await runtime.configureNativeParameters(values);
      project.modelParameters={...previous,[id]:{...values}};changed(false);
      const message=project.slamBackend==='native'?'Model parameters applied · current map retained · press Run to continue':'Parameters saved for RGB-D SLAM';status(message);return message;
    }
    project.modelParameters={...previous,[id]:{...values}};dirty=true;
    await compile();
    if(!ready){
      const error=String(compilationError??'Model initialization failed');
      if(previous)project.modelParameters=previous;else delete project.modelParameters;
      dirty=true;await compile();throw Error(`${error}. Previous parameter settings restored.`);
    }
    changed(false);return 'Model parameters applied · simulation reset · press Run to continue';
  }finally{configuration.setBusy(compiling);parameterPanel.setBusy(compiling);disableRun(!ready||compiling);syncApplyButton();}
}
async function changeSensorRates(rates:SensorRates|undefined) {
  if(rates)project.sensorRates={...rates};else delete project.sensorRates;
  changed(false);configuration.sync(project);
  disableRun(true);el<HTMLSelectElement>('scene-detail').disabled=true;
  try {
    await initializationReady;
    while(runtime.busy||compiling)await new Promise(resolve=>setTimeout(resolve,20));
    runtime.setSensorRates(rates);
    status(ready?'Sensor rates updated · run preserved · press Run to continue':'Sensor rates saved · select an available experiment to run');
  }catch(error){status(String(error),true);}
  finally {disableRun(!ready);syncApplyButton();el<HTMLButtonElement>('reset').disabled=false;el<HTMLSelectElement>('scene-detail').disabled=false;}
}
el<HTMLInputElement>('lidar-enabled').onchange=async e=>{project.lidarEnabled=(e.target as HTMLInputElement).checked;changed();await compile();};
el<HTMLInputElement>('depth-cloud-enabled').onchange=async e=>{project.depthCloudEnabled=(e.target as HTMLInputElement).checked;changed();await compile();};
el<HTMLInputElement>('show-paths').onchange=e=>{world.trajectories.visible=(e.target as HTMLInputElement).checked;syncViewerFlags();};
el<HTMLSelectElement>('lighting-mode').onchange=()=>{project.lightingMode=el<HTMLSelectElement>('lighting-mode').value as 'day'|'night'|'cycle';world.setLighting(project.lightingMode);changed();};
for(const id of ['cars-enabled','people-enabled'])el<HTMLInputElement>(id).onchange=()=>{
  project.carsEnabled=el<HTMLInputElement>('cars-enabled').checked;project.peopleEnabled=el<HTMLInputElement>('people-enabled').checked;
  world.configureActors(project.carsEnabled,project.peopleEnabled);
  changed();
};
el<HTMLSelectElement>('speed').onchange=e=>runtime.speed=Number((e.target as HTMLSelectElement).value);
el('flight-control').onclick=()=>setManualFlight(runtime.autopilot);
el<HTMLSelectElement>('tour-mode').onchange=e=>{
  const route=(e.target as HTMLSelectElement).value;if(!isFlightTour(route))return;
  project.flightTour=route;runtime.setFlightTour(route);changed(false);
  el<HTMLButtonElement>('reset').click();
};
el<HTMLInputElement>('show-map').onchange=e=>{world.showMap((e.target as HTMLInputElement).checked);syncViewerFlags();};
el<HTMLInputElement>('show-uncertainty').onchange=()=>{if(latest){world.setEstimate(latest.estimate);world.covariance.visible=!!latest.estimate.uncertainty&&el<HTMLInputElement>('show-uncertainty').checked;}syncViewerFlags();};
el<HTMLInputElement>('show-graph').onchange=()=>{for(const object of [world.graphEdges,world.loopEdges,world.keyframes])object.visible=el<HTMLInputElement>('show-graph').checked;syncViewerFlags();};
function syncViewerFlags(){viewer?.visibility({uncertainty:el<HTMLInputElement>('show-uncertainty').checked,graph:el<HTMLInputElement>('show-graph').checked,showMap:el<HTMLInputElement>('show-map').checked,showPaths:el<HTMLInputElement>('show-paths').checked});}
el<HTMLInputElement>('show-matches').onchange=()=>{if(latest)drawSensors(latest.frame,latest.estimate);};
el('help').onclick=()=>{el<HTMLDetailsElement>('lesson').open=!el<HTMLDetailsElement>('lesson').open;el('lesson').scrollIntoView({behavior:'smooth'});};
function disableRun(disabled:boolean){
  parameterPanel.setBusy(compiling||assistantSimulation);
  for(const id of ['run','step','reset','apply','record'])el<HTMLButtonElement>(id).disabled=disabled;
  runModel.disabled=compiling||project.slamBackend==='native';
  const count=document.getElementById('vehicle-count') as HTMLSelectElement|null;
  if(count)count.disabled=compiling||assistantSimulation;
  el<HTMLSelectElement>('active-vehicle').disabled=compiling||assistantSimulation;
}
let compilationError:unknown,recovering=false;
function defaultModelsFromText(text:string){
  let saved:unknown;try{saved=JSON.parse(text);}catch{/* Invalid JSON cannot supply settings. */}
  return withDefaultModels(saved,mobileDefaults);
}
function showRecoveryBackup(backup:ProjectRecoveryBackup,message:string){
  let notice=document.getElementById('model-recovery');
  if(!notice){notice=document.createElement('div');notice.id='model-recovery';notice.setAttribute('role','status');el('status').after(notice);}
  const summary=document.createElement('p');summary.textContent=message;
  const details=document.createElement('details'),label=document.createElement('summary'),reason=document.createElement('pre');
  label.textContent='Recovery details';reason.textContent=backup.reason;details.append(label,reason);
  const downloadBackup=document.createElement('button');downloadBackup.textContent='Download previous model';
  downloadBackup.onclick=()=>download('previous-model.slam.json',backup.text);
  notice.replaceChildren(summary,downloadBackup,details);
}
async function recoverDefaultModels(text:string,reason:unknown){
  if(compiling||recovering||assistantSimulation)return;
  recovering=true;localPersistenceBlocked=true;el<HTMLButtonElement>('save').disabled=true;
  el<HTMLButtonElement>('default-models').disabled=true;el<HTMLInputElement>('open').disabled=true;
  if(autosaveTimer)clearTimeout(autosaveTimer);autosaveTimer=undefined;
  runtime.pause();ready=false;disableRun(true);
  const backup:ProjectRecoveryBackup={id:crypto.randomUUID(),text,reason:String(reason),createdAt:new Date().toISOString()};
  let backedUp=false;
  showRecoveryBackup(backup,'Preparing current default models. Your original source is available below.');
  try{
    try{await saveProjectRecovery(backup);backedUp=true;}catch{/* Keep the saved project intact if durable backup is unavailable. */}
    while(runtime.busy)await new Promise(resolve=>setTimeout(resolve,20));
    project=defaultModelsFromText(text);projectRevision++;dirty=true;
    await syncProject();await buildProjectScene();await compile();
    if(ready&&backedUp){
      try{await saveProject(project);localPersistenceBlocked=false;el<HTMLButtonElement>('save').disabled=false;}
      catch{/* Keep recovery temporary when persistence fails. */}
    }
    const message=ready?'Loaded current default models; valid scene, camera and simulation settings were retained.':'Current default models also failed to prepare. Your original source is available below.';
    showRecoveryBackup(backup,message+(localPersistenceBlocked?' Automatic saving is disabled; download your project to keep changes.':''));
    el('saved').textContent=localPersistenceBlocked?'Original saved project preserved':'Defaults saved · original source backed up';
    if(ready)status('Ready · current default models loaded');
  }catch(error){status(String(error),true);showRecoveryBackup(backup,'Recovery failed. Your original source is available below. Automatic saving is disabled.');}
  finally{recovering=false;el<HTMLButtonElement>('default-models').disabled=false;el<HTMLInputElement>('open').disabled=false;}
}
async function compile(reusePrepared=false) {
  await initializationReady;
  if(compiling||assistantSimulation)return;
  compilationError=undefined;compiling=true;ready=false;disableRun(true);el<HTMLSelectElement>('scene-detail').disabled=true;el<HTMLSelectElement>('preset').disabled=true;runtime.pause();
  configuration.setBusy(true);
  while(runtime.busy)await new Promise(resolve=>setTimeout(resolve,20));
  loopHistories.forEach(history=>history.reset());pendingPresentation=undefined;
  loopPanel.render(loopHistories[0]);
  const snapshot=structuredClone(project),revision=JSON.stringify(project);
  try {
    if(reusePrepared)clearKeys();
    await runtime.prepare(snapshot,reusePrepared);
    if(JSON.stringify(project)!==revision){dirty=true;status('Source changed during compilation · apply your latest edits');return;}
    project.detectorArtifact=snapshot.detectorArtifact;
    project.algorithmArtifact=snapshot.algorithmArtifact;
    dirty=false;ready=true;truthPath=[];estimatePath=[];comparisons.clear();externalLatest=undefined;latest=undefined;latestMetrics=undefined;pendingSensorPreview=undefined;el('run').textContent='▶ Run';drawTrajectory();
    navigation.reset();telemetryPlot.reset();
    vehiclePaths[0]={truth:[],estimate:[]};vehiclePaths[1]={truth:[],estimate:[]};
    telemetryPlot.setVehicleCount(runtime.vehicleCount);
    el<HTMLSelectElement>('active-vehicle').value='0';el('active-vehicle').hidden=runtime.vehicleCount===1;
    el<HTMLSelectElement>('vehicle-count').value=String(project.vehicleCount??1);el<HTMLSelectElement>('slam-backend').value=project.slamBackend??'inertial';syncFlightHelp();
    syncSceneControls();
    if(!localPersistenceBlocked)try{await saveProject(project);}catch{el('saved').textContent='Cannot save locally · download your project';}
  }
  catch(error){compilationError=error;ready=false;status(String(error),true);}
  finally {
    configuration.sync(project);
    compiling=false;configuration.setBusy(false);disableRun(!ready);el<HTMLButtonElement>('apply').disabled=false;el<HTMLButtonElement>('reset').disabled=false;
    el<HTMLButtonElement>('step').disabled=!ready;
    // A failed fleet build leaves one vehicle; the selector must not offer a second.
    el('active-vehicle').hidden=runtime.vehicleCount===1;if(runtime.vehicleCount===1)el<HTMLSelectElement>('active-vehicle').value='0';
    telemetryPlot.setVehicleCount(runtime.vehicleCount);
    el<HTMLSelectElement>('scene-detail').disabled=false;
    el<HTMLSelectElement>('preset').disabled=viewingSlamFile()||!['slam','detector'].includes(selected.kind);
    syncApplyButton();syncEstimatorMode();
  }
}
el('apply').onclick=()=>{if(!viewingSlamFile())void compile();};el('reset').onclick=()=>void compile(true);
el('run').onclick=async()=>{
  if(assistantSimulation)return;
  if(runtime.running){runtime.pause();el('run').textContent='▶ Run';return;}
  if(dirty||!ready)await compile();
  if(ready){runtime.play();el('run').textContent='Ⅱ Pause';}
};
el('step').onclick=async()=>{
  if(assistantSimulation)return;
  runtime.pause();while(runtime.busy)await new Promise(resolve=>setTimeout(resolve,20));
  if(dirty||!ready)await compile();
  if(ready)try{await runtime.step();}catch(error){status(String(error),true);}
};
el('save').onclick=async()=>{if(localPersistenceBlocked)return;if(autosaveTimer)clearTimeout(autosaveTimer);autosaveTimer=undefined;try{await saveProject(project);status('Project saved in this browser');el('saved').textContent='Saved locally';}catch(error){status(String(error),true);}};
el('download').onclick=()=>download(`${project.name.replace(/[^a-z0-9_-]/gi,'_')}.slam.json`,JSON.stringify(project,null,2));
el('default-models').onclick=()=>void recoverDefaultModels(JSON.stringify(project,null,2),'Current default models requested');
el<HTMLInputElement>('open').onchange=async e=>{
  const input=e.target as HTMLInputElement,file=input.files?.[0];if(!file||compiling||recovering||assistantSimulation)return;
  const text=await file.text();input.value='';
  try{
    // Do not autosave an imported project until it has compiled successfully.
    if(autosaveTimer)clearTimeout(autosaveTimer);autosaveTimer=undefined;
    const opened=parseProject(text);
    runtime.pause();ready=false;disableRun(true);
    while(runtime.busy)await new Promise(resolve=>setTimeout(resolve,20));
    project=opened;projectRevision++;dirty=true;
    localPersistenceBlocked=true;el<HTMLButtonElement>('save').disabled=true;
    await syncProject();await buildProjectScene();await compile();
    if(ready){
      try{await saveProject(project);localPersistenceBlocked=false;el<HTMLButtonElement>('save').disabled=false;el('saved').textContent='Saved locally';}
      catch{el('saved').textContent='Cannot save locally · download your project';status('Project loaded · cannot save locally; download your project to keep changes');}
    }
    else if(compilationError!==undefined)await recoverDefaultModels(text,compilationError);
  }catch(error){await recoverDefaultModels(text,error);}
};
el('record').onclick=()=>{const vehicle=runtime.activeVehicle;vehicle.recording=!vehicle.recording;el('record').textContent=vehicle.recording?'■ Stop recording':'● Record dataset';el<HTMLButtonElement>('export-recording').disabled=vehicle.records.length===0;};
el('export-recording').onclick=()=>{
  const vehicle=runtime.activeVehicle;
  const records=vehicle.records.map(r=>({...r,estimate:{...r.estimate,points:[]},frame:{...r.frame,rgb:Array.from(r.frame.rgb),depth:Array.from(r.frame.depth)}}));
  download(`${project.name}-drone-${runtime.activeVehicleIndex+1}-dataset.json`,JSON.stringify({format:'slam-lab-dataset',version:2,project,evaluationOrigin:vehicle.evaluationOrigin,records}));
};
el<HTMLInputElement>('replay').onchange=async e=>{
  const file=(e.target as HTMLInputElement).files?.[0];if(!file)return;
  runtime.pause();while(runtime.busy)await new Promise(resolve=>setTimeout(resolve,20));
  try {
    const data=JSON.parse(await file.text());
    if(data.format!=='slam-lab-dataset'||![1,2].includes(data.version)||!Array.isArray(data.records)||data.records.length>300)throw new Error('Unsupported dataset');
    const records:RunRecord[]=data.records.map((r:any)=>({...r,frame:{...r.frame,rgb:new Uint8Array(r.frame.rgb),depth:r.frame.imageLayout?new Uint16Array(r.frame.depth):new Float32Array(r.frame.depth)}}));
    for(let i=0;i<records.length;i++) {const f=records[i].frame;validateSensorFrame(f);if(i>0&&(f.sequence<=records[i-1].frame.sequence||f.time<=records[i-1].frame.time))throw new Error('Reordered sensor frame in dataset');}
    if(dirty||!ready)await compile();await runtime.loadReplay(records,data.evaluationOrigin);truthPath=[];estimatePath=[];comparisons.clear();externalLatest=undefined;runtime.play();el('run').textContent='Ⅱ Pause';
  }catch(error){status(String(error),true);}
};
el('export-node').onclick=()=>{
  const path=viewingSlamFile()?activeSlamFile:viewingLibraryFile()?activeLibraryFile:
    selected.kind==='slam'?project.mainSourcePath:undefined;
  download(path?.split('/').at(-1)??`${selected.id}.mo`,selectedSource(),'text/plain');
};
const cameraControls=new ViewerCameraControls(),keys=new Set<string>();
el('world').tabIndex=0;el('world').setAttribute('aria-label','World camera');
el('world').addEventListener('pointerdown',()=>el('world').focus({preventScroll:true}));
const flightSettings=document.querySelector('.configuration-flight')!;
flightSettings.insertAdjacentHTML('beforeend','<label>Estimator <select id="slam-backend" aria-label="Estimator backend"><option value="inertial">Inertial baseline</option><option value="native">Native Modelica SLAM · desktop, 1.2 GB</option></select></label><label>Vehicles <select id="vehicle-count" aria-label="Number of drones"><option value="1">1 drone</option><option value="2">2 drones · independent sensors and estimation</option></select></label><label>Camera view <select id="camera-view" aria-label="Camera view"></select></label><label>Keys control <select id="keyboard-mode" aria-label="Keyboard control"><option value="drone">Drone commands</option><option value="viewer">Viewer camera</option></select></label><p id="flight-help"></p>');
el('slam-backend').closest('label')!.after(runModelLabel);
for(const view of CAMERA_VIEWS)el<HTMLSelectElement>('camera-view').add(new Option(view.label,view.id));
function clearKeys(){keys.clear();cameraControls.clear();updateCommand();}
function updateCommand(){runtime.command={forward:(+keys.has('w')-+keys.has('s'))*1.6,left:(+keys.has('a')-+keys.has('d'))*1.6,up:(+keys.has('r')-+keys.has('f'))*.8,yaw:(+keys.has('q')-+keys.has('e'))*.5};}
function syncFlightHelp(){
  const drone=el<HTMLSelectElement>('keyboard-mode').value==='drone';
  const mode=(runtime.vehicleCount===2?'Drone '+(runtime.activeVehicleIndex+1)+' · ':'')+(runtime.autopilot?'Flight tour':drone?'Manual flight':'Manual flight · viewer keys');
  const view=CAMERA_VIEWS.find(view=>view.id===world.viewerCamera.mode)!.label;
  document.querySelector('.scene-bottom')!.firstChild!.textContent=mode+' · '+view+' · V change view ';
  el('flight-help').textContent=world.viewerCamera.mode==='orbit-follow'?'Drag to orbit and scroll to zoom around the followed drone. Vehicle turns do not rotate the view. V changes the camera view.':runtime.autopilot?'Flight tour is flying the drone. Choose Manual flight in the toolbar to take control.':drone?'Click Run, then use WASD to fly, Q/E to yaw and R/F to climb or descend. V changes the camera view.':'WASD moves the camera; Q/E orbit or yaw, R/F altitude, Shift faster. Follow views move relative to the drone. V changes the camera view.';
}
function setCameraView(mode:CameraView){
  world.setCameraView(mode);el<HTMLSelectElement>('camera-view').value=mode;
  cameraControls.clear();syncFlightHelp();viewer?.camera(true);
}
function setManualFlight(manual:boolean){
  clearKeys();runtime.autopilot=!manual;
  const button=el<HTMLButtonElement>('flight-control');
  button.setAttribute('aria-pressed',String(manual));button.textContent=manual?'Manual flight':'Flight tour';
  button.title=manual?'Switch to flight tour':'Take manual control';
  el<HTMLSelectElement>('tour-mode').disabled=manual;
  if(manual)el<HTMLSelectElement>('keyboard-mode').value='drone';
  syncFlightHelp();
}
el('camera-view').onchange=()=>setCameraView(el<HTMLSelectElement>('camera-view').value as CameraView);
el('keyboard-mode').addEventListener('change',()=>{
  clearKeys();
  syncFlightHelp();
});
el('slam-backend').onchange=async()=>{
  project.slamBackend=el<HTMLSelectElement>('slam-backend').value as 'inertial'|'native';
  changed();await compile();
};
el('vehicle-count').onchange=async()=>{
  project.vehicleCount=Number(el<HTMLSelectElement>('vehicle-count').value) as 1|2;
  if(project.vehicleCount===2){project.modelicaSources??={};project.modelicaSources[offsetQuadrotorPath]??=offsetQuadrotorSource;refreshFiles();}
  changed();await compile();
};
el('active-vehicle').onchange=()=>{
  clearKeys();comparisons.clear();navigation.reset();
  try{runtime.selectVehicle(Number(el<HTMLSelectElement>('active-vehicle').value) as 0|1);}
  catch(error){el<HTMLSelectElement>('active-vehicle').value=String(runtime.activeVehicleIndex);status(String(error),true);}
  syncFlightHelp();viewer?.camera(true);
};
setCameraView('follow');setManualFlight(true);
window.addEventListener('keydown',e=>{
  const key=e.key.toLowerCase();
  if(document.body.classList.contains('app-loading')||!viewerPresented||isTextEntry(e.target)||e.ctrlKey||e.metaKey||e.altKey)return;
  if(key==='v'){
    e.preventDefault();if(!e.repeat)setCameraView(nextCameraView(world.viewerCamera.mode));return;
  }
  if(!VIEWER_KEYS.has(key))return;
  e.preventDefault();
  if(el<HTMLSelectElement>('keyboard-mode').value==='viewer'){
    cameraControls.keys.add(key);
  }else if(!runtime.autopilot){keys.add(key);updateCommand();}
});
window.addEventListener('keyup',e=>{keys.delete(e.key.toLowerCase());cameraControls.keys.delete(e.key.toLowerCase());updateCommand();});
window.addEventListener('blur',clearKeys);
// A hidden page throttles the simulation; that wall time is not counted.
let hiddenSince:number|undefined;
document.addEventListener('visibilitychange',()=>{
  if(document.hidden){clearKeys();hiddenSince=performance.now();}
  else if(hiddenSince!==undefined){runtime.excludeHiddenWall(performance.now()-hiddenSince);hiddenSince=undefined;}
});
document.addEventListener('focusin',e=>{if(isTextEntry(e.target))clearKeys();});
function drawSensors(frame:SensorFrame,estimate:Estimate) {
  const rgb=el<HTMLCanvasElement>('rgb'),ctx=rgb.getContext('2d')!;
  const k=frame.calibration;
  colorPreview.draw(ctx,frame.rgb,k.width,k.height,frame.imageLayout?.color);
  ctx.strokeStyle='#50ffc4';ctx.lineWidth=.75;
  for(const [u,v] of estimate.features??[])ctx.strokeRect(u-1.8,v-1.8,3.6,3.6);
  if(el<HTMLInputElement>('show-matches').checked) {
    ctx.strokeStyle=estimate.tracking?.accepted?'#ffbd73':'#bd9afa';ctx.lineWidth=.6;
    for(const match of estimate.tracking?.matches??[]) {
      ctx.beginPath();ctx.moveTo(match.previous[0],match.previous[1]);ctx.lineTo(match.current[0],match.current[1]);ctx.stroke();
    }
  }
  depthPreview.draw(frame.depth,k.width,k.height,k.far,frame.imageLayout?.depth);
  document.querySelector('#rgb + .sensor-foot')!.textContent=`${k.width} × ${k.height} · fx ${k.rgbFx.toFixed(1)} · ${frame.imageLayout?'RGB8':'RGBA8 dataset'}`;
  document.querySelector('#depth + .sensor-foot')!.textContent=`${k.width} × ${k.height} · ${frame.imageLayout?`Z16 · ${frame.imageLayout.depth.unitsMeters*1000} mm/unit · `:''}${k.near.toFixed(2)}–${k.far.toFixed(1)} m · black = no depth return`;
}
function drawTrajectory() {
  const canvas=el<HTMLCanvasElement>('trajectory'),ctx=canvas.getContext('2d')!;ctx.clearRect(0,0,canvas.width,canvas.height);
  const paths:[Pose[],string][]=[[truthPath,'#b4c5d4'],[estimatePath,'#50e6b2'],...Array.from(comparisons.streams.values(),s=>[comparisons.path(s),s.color] as [Pose[],string])];
  const all=paths.flatMap(([path])=>path);
  const minX=Math.min(-2,...all.map(p=>p.x)),maxX=Math.max(8,...all.map(p=>p.x));
  const minY=Math.min(-2,...all.map(p=>p.y)),maxY=Math.max(8,...all.map(p=>p.y));
  const scale=Math.min((canvas.width-24)/(maxX-minX),(canvas.height-24)/(maxY-minY));
  ctx.strokeStyle='#26333f';ctx.lineWidth=.5;
  for(let x=0;x<canvas.width;x+=20){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,canvas.height);ctx.stroke();}
  for(let y=0;y<canvas.height;y+=20){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(canvas.width,y);ctx.stroke();}
  for(const [path,color] of paths) {
    ctx.strokeStyle=color;ctx.lineWidth=1.5;ctx.beginPath();
    path.forEach((p,i)=>{const x=12+(p.x-minX)*scale,y=canvas.height-12-(p.y-minY)*scale;i?ctx.lineTo(x,y):ctx.moveTo(x,y);});ctx.stroke();
  }
  const table=document.createElement('table');
  for(const stream of comparisons.streams.values()) {
    const row=document.createElement('tr'),name=document.createElement('td'),value=document.createElement('td');
    name.textContent=stream.topic;name.style.color=stream.color;
    value.textContent=stream.pairs?`${Math.sqrt(stream.squaredDifference/stream.pairs).toFixed(3)} m RMS · ${stream.pairs} pairs`:`${stream.samples} samples · waiting for shared timestamps`;
    row.append(name,value);table.append(row);
  }
  el('comparisons').replaceChildren(table);
}
runtime.onStatus=message=>{status(message);el('flight-status').textContent=message;};
runtime.onError=error=>{ready=false;dirty=true;status(error.message+' · Run or Reset recompiles the experiment',true);el('run').textContent='▶ Run';};
const vehiclePaths:{truth:Pose[];estimate:Pose[]}[]=[{truth:[],estimate:[]},{truth:[],estimate:[]}];
runtime.onVehicleFrame=(index,{frame,estimate,truth})=>{
  performanceMonitor.visionProcessed();
  loopHistories[index].observe(frame,estimate);
  telemetryPlot.sample(index,frame.time,truth,estimate,frame.imu);
  const vehicle=runtime.getVehicle(index);
  const reference=truth?vehicle.referenceTruth(truth):undefined;
  const paths=vehiclePaths[index];
  if(reference)paths.truth.push(poseSnapshot(reference));paths.estimate.push(poseSnapshot(estimate));
  if(paths.truth.length>2000)paths.truth.shift();if(paths.estimate.length>2000)paths.estimate.shift();
};
let pendingPresentation:(()=>void)|undefined;
runtime.onFrame=(frame,estimate,truth,metrics)=>{
  const displayed=runtime.activeVehicle,displayWorld=displayed.world;
  latestMetrics=metrics;
  latest={frame,estimate,truth};
  comparisons.addLocal(frame.time,estimate);
  // Navigation and telemetry consume every measurement. Geometry, DOM and
  // viewer diagnostics consume only the newest completed frame at 30 Hz.
  const nav=navigation.update(frame.time,estimate,displayed.gps);
  pendingPresentation=()=>{
  loopPanel.render(loopHistories[runtime.activeVehicleIndex],Number(estimate.diagnostics?.loopVocabularyReady),
    typeof estimate.diagnostics?.loopWords==='number'?estimate.diagnostics.loopWords:undefined,
    estimate.diagnostics?.runtime==='Rumoca native SLAM program');
  truthPath=vehiclePaths[runtime.activeVehicleIndex].truth;estimatePath=vehiclePaths[runtime.activeVehicleIndex].estimate;
  const paths=[{id:'truth',color:0xc9d7e3,points:truthPath},{id:'slam',color:0x50e6b2,points:estimatePath},...Array.from(comparisons.streams.values()).map((stream,i)=>({id:`external-${i}`,color:Number.parseInt(stream.color.replace('#',''),16),points:comparisons.path(stream)}))];
  world.setMap(estimate.points,truth?displayed.evaluationOrigin:undefined);world.setEstimate(estimate);
  world.setTrajectories(paths);
  if(displayWorld.latestLidar)viewer?.lidar(displayWorld.latestLidar,displayWorld);
  viewer?.depthCloudEnabled(displayWorld.depthCloudEnabled);
  const depthDisplay=displayWorld.latestDepthRaster??displayWorld.latestDepthCloud;
  if(depthDisplay)viewer?.depthCloud(depthDisplay);
  viewer?.diagnostics(estimate,truth?displayed.evaluationOrigin:undefined,paths,{uncertainty:el<HTMLInputElement>('show-uncertainty').checked,graph:el<HTMLInputElement>('show-graph').checked,showMap:el<HTMLInputElement>('show-map').checked,showPaths:el<HTMLInputElement>('show-paths').checked});
  el('navigation-state').textContent=truth?`Navigation · ${nav.status} · ${nav.reason}${nav.alignment?` · alignment ${nav.alignment.pairs} fixes / ${nav.alignment.rms.toFixed(2)} m RMS`:''}`:'GPS handoff unavailable for this external/replayed dataset';
  el('lidar-state').textContent=displayWorld.latestLidar?`LiDAR · 64 × ${displayWorld.latestLidar.columns} samples · raw GPU cloud · t=${displayWorld.latestLidar.time.toFixed(3)} s · ${displayWorld.lidar.timings?.totalMs.toFixed(1)} ms`:'LiDAR disabled';
  pendingSensorPreview={frame,estimate};
  el('sim-time').textContent=`t = ${metrics.time.toFixed(2)} s`;el('metric-time').innerHTML=`${metrics.time.toFixed(2)} <em>s</em>`;
  el('metric-ate').innerHTML=metrics.ate===null?'Unavailable':`${metrics.ate.toFixed(3)} <em>m</em>`;el('metric-features').textContent=displayed.pendingNodes.size?'Pending':String(metrics.features);
  el('metric-points').textContent=metrics.points.toLocaleString();el('metric-rtf').innerHTML=`${metrics.rtf.toFixed(2)} <em>×</em>${runtime.frameClock.recentRate===undefined?'':` <em>· last 10 s ${runtime.frameClock.recentRate.toFixed(2)} ×</em>`}`;
  el('metric-time').previousElementSibling!.textContent=truth?'SIM TIME':'SENSOR TIME';
  el('metric-rtf').previousElementSibling!.textContent=truth?'SIM / WALL TIME':'SENSOR / WALL TIME';
  el('flight-status').textContent=`${truth?'Simulation':'Measured input'} · frame ${frame.sequence} · ${displayed.pendingNodes.size?'Inertial baseline · visual SLAM pending':`${(metrics.confidence*100).toFixed(0)}% tracking · ${metrics.loops} loop matches`}${metrics.dropped?` · ${metrics.dropped} skipped in browser`:''}`;
  world.covariance.visible=!!estimate.uncertainty&&el<HTMLInputElement>('show-uncertainty').checked;
  el('tracking-state').textContent=estimate.tracking?`${estimate.tracking.matches.length} proposed feature matches${estimate.tracking.referenceSequence===undefined?'':` to frame ${estimate.tracking.referenceSequence}`} · ${estimate.tracking.reason}${estimate.diagnostics?.visualUpdates===undefined?'':` · ${estimate.diagnostics.visualUpdates} visual updates`}${estimate.diagnostics?.visualRejected===undefined?'':` / ${estimate.diagnostics.visualRejected} rejected`}`:'Matching diagnostics unavailable for this node';
  el('uncertainty-state').textContent=estimate.uncertainty?uncertaintySummary(estimate):'Covariance unavailable for this node';
  el('graph-state').textContent=estimate.poseGraph?`${estimate.poseGraph.keyframes.length} keyframes · ${estimate.poseGraph.edges.length} constraints · ${metrics.loops} verified loops${estimate.diagnostics?.loopRejections===undefined?'':` · ${estimate.diagnostics.loopRejections} rejected candidates`}`:'Pose graph unavailable for this node';
  if(estimate.diagnostics?.loopWords!==undefined)el('graph-state').textContent+=` · ${estimate.diagnostics.loopWords} visual words / ${estimate.diagnostics.loopCandidates??0} candidates this frame`;
  if(estimate.diagnostics?.landmarksConfirmed!==undefined)el('graph-state').textContent+=` · ${estimate.diagnostics.landmarksConfirmed} confirmed / ${estimate.diagnostics.landmarksTentative??0} tentative landmarks · ${estimate.diagnostics.landmarksPruned??0} pruned (${estimate.diagnostics.landmarksFreeSpacePruned??0} free-space contradictions)`;
  if(estimate.diagnostics?.mappingStatus)el('graph-state').textContent+=` · ${estimate.diagnostics.mappingStatus}`;
  // Lockstep vehicles run concurrently; the frame waits for the slower one.
  // Frame wall splits into the real-time pacing wait, that vehicle's nodes and other time.
  const work=Array.from({length:runtime.vehicleCount},(_,index)=>{const timings=runtime.getVehicle(index as 0|1).lastTimings;
    return {index,timings,total:Object.values(timings).reduce((sum,ms)=>sum+ms,0)};});
  const clock=runtime.frameClock,slowest=Math.max(...work.map(vehicle=>vehicle.total));
  el('frame-timings').textContent=work.map(({index,timings,total})=>`${work.length>1?`Drone ${index+1} frame`:'Frame'} work: ${Object.entries(timings).map(([id,ms])=>`${id} ${ms.toFixed(1)} ms`).join(' · ')} · total ${total.toFixed(1)} ms`).join(' | ')
    +(clock.intervalMs>0?` | frame wall ${clock.intervalMs.toFixed(1)} ms (step ${clock.stepMs.toFixed(1)} ms) · paced wait ${clock.pacedMs.toFixed(1)} ms · other ${clock.otherMs(slowest).toFixed(1)} ms`:'');
  if(displayWorld.captureTimings&&truth)el('frame-timings').textContent+=` · camera submission ${displayWorld.captureTimings.renderSubmission.toFixed(1)} ms / readback wait ${displayWorld.captureTimings.readback.toFixed(1)} ms`;
  el('record-state').textContent=`${displayed.records.length} recorded frames${displayed.recording?' · recording':''}`;
  el('record').textContent=displayed.recording?'■ Stop recording':'● Record dataset';
  el<HTMLButtonElement>('export-recording').disabled=displayed.records.length===0;
  };
};
// A single independently paced overview; simulation awaits every graph node.
// Correct the deadline phase rather than drawing bursts after a missed slot.
let cameraWall=performance.now();
function render(now=0){
  if(performanceMonitor.shouldRender(now)){
    const started=performance.now();world.controls.update();
    const present=pendingPresentation;pendingPresentation=undefined;present?.();
    const cameraDt=(now-cameraWall)/1000;cameraWall=now;
    if(world.viewerCamera.mode==='free')cameraControls.update(world.view,world.controls,cameraDt);
    else world.viewerCamera.move(cameraControls.keys,cameraDt);
    if(viewerPresented){
      if(viewer){viewer.flushPoses();viewer.flushClouds();viewer.camera();viewer.running(runtime.running);}else world.render(runtime.running);
    }
    if(pendingSensorPreview){const {frame,estimate}=pendingSensorPreview;pendingSensorPreview=undefined;drawSensors(frame,estimate);drawTrajectory();}
    performanceMonitor.rendered(performance.now()-started);
  }
  requestAnimationFrame(render);
}requestAnimationFrame(render);
// Exposed state supports browser integration checks without privileged access.
installLiveCapture({runtime,status,
  d435Started(){
    runtime.pause();
    pendingPresentation=undefined;
    pendingSensorPreview=undefined;
    truthPath=[];estimatePath=[];
    world.setEnvironmentVisible(false);
    el('run').textContent='▶ Run';
    drawTrajectory();
  },
  d435Frame(points,position,time){
    const [x,y,z]=position??[0,0,0];
    const pose={x,y,z,quaternion:[1,0,0,0]};
    const last=estimatePath.at(-1);
    if(position&&(!last||Math.hypot(x-last.x,y-last.y,z-last.z)>.002)){
      estimatePath.push(pose);
      if(estimatePath.length>5000)estimatePath.shift();
    }
    const paths=[{id:'d435',color:0x50e6b2,points:estimatePath}];
    const estimate={...pose,points,confidence:0};
    world.setMap(points);
    world.showMap(true);
    world.setTrajectories(paths);
    viewer?.diagnostics(estimate,undefined,paths,{uncertainty:false,graph:false,showMap:true,showPaths:true});
    el('metric-points').textContent=points.length.toLocaleString();
    el('metric-ate').textContent='—';
    el('metric-features').textContent='—';
    el('metric-time').previousElementSibling!.textContent='SENSOR TIME';
    el('metric-time').textContent=time===undefined?'—':time.toFixed(2)+' s';
    el('flight-status').textContent=position?'D435 · RGB-D odometry':'D435 · live depth';
    drawTrajectory();
  },
  d435Images(camera,rgb){
    const {color,depth}=camera.header;
    colorPreview.draw(el<HTMLCanvasElement>('rgb').getContext('2d')!,rgb,color.width,color.height,
      {width:color.width,height:color.height,format:'RGB8',strideBytes:color.width*3,bytes:rgb.byteLength});
    depthPreview.draw(camera.depth,depth.width,depth.height,5);
    document.querySelector('#rgb + .sensor-foot')!.textContent=color.width+' × '+color.height+' · D435 color';
    document.querySelector('#depth + .sensor-foot')!.textContent=depth.width+' × '+depth.height+' · D435 depth · meters';
  },
  d435Stopped(){
    world.setEnvironmentVisible(true);
  },
  async prepare(resolution,variant){
    if(project.slamBackend==='native'&&project.cameraResolution===resolution&&ready&&!dirty&&runtime.nativeVariant===variant)return;
    runtime.nativeVariant=variant;project.slamBackend='native';project.cameraResolution=resolution;
    el<HTMLSelectElement>('slam-backend').value='native';
    changed();await compile();configuration.sync(project);
    if(!ready)throw new Error('The native SLAM did not finish compiling: '+(el('status').textContent||'see the status line'));
  },
  release(){if(runtime.nativeVariant!=='image'){runtime.nativeVariant='image';changed();}},
  started(){truthPath=[];estimatePath=[];comparisons.clear();externalLatest=undefined;el('run').textContent='Ⅱ Pause';},
  stopped(){el('run').textContent='▶ Run';},
});
(window as any).__slamLab={runtime,comparisons,sourceEditor,performanceMonitor,loopHistories,get initialized(){return initialized;},get viewer(){return viewer;},get project(){return project;},get slamBuildReceipt(){return slamBuildReceipt;},get latest(){return latest;},get ready(){return ready;},get compiling(){return compiling;},get externalSamples(){return Array.from(comparisons.streams.values()).reduce((sum,s)=>sum+s.samples,0);},get externalLatest(){return externalLatest;}};
void (async()=>{
  startup.stage(0,'Opening your project…');
  let savedText:string|undefined,openingError:unknown,selectedNativeDefault=false,nativeDefaultFailed=false;
  try{
    const backup=await readProjectRecovery();if(backup)showRecoveryBackup(backup,'Previous model backup available.');
    savedText=await readSavedProjectText();
    project=savedText===undefined?defaultProject(mobileDefaults):parseProject(savedText);
    selected=project.graph.nodes.find(n=>n.kind==='slam')??selected;
  }catch(error){
    openingError=error;
    if(savedText!==undefined)project=defaultModelsFromText(savedText);
    else{
      localPersistenceBlocked=true;el<HTMLButtonElement>('save').disabled=true;
      el('saved').textContent='Cannot read local project · automatic saving disabled';
      status(String(error),true);
    }
  }
  if(savedText===undefined&&!openingError){
    const device=navigator as Navigator&{deviceMemory?:number;userAgentData?:{mobile:boolean}};
    if(useNativeDefault({hardwareRenderer:world.graphics.acceleration==='hardware-reported',
      mobile:device.userAgentData?.mobile===true||/Android|iPhone|iPad|iPod/i.test(device.userAgent),
      simd:supportsWasmSimd(),memoryGB:device.deviceMemory,logicalCores:device.hardwareConcurrency})){
      try{
        await fetchNativeManifest(new URL(import.meta.env.BASE_URL,location.href),project.cameraResolution??'424x240');
        project.slamBackend='native';selectedNativeDefault=true;
      }catch{/* Unpublished native programs keep the inertial startup path. */}
    }
  }
  try{
    const modelsReady=syncProject();
    startup.stage(1,'Preparing the scene and graphics…');
    if(typeof OffscreenCanvas!=='undefined'&&typeof HTMLCanvasElement.prototype.transferControlToOffscreen==='function'){
      const candidate=new WorkerViewer(world,el('world'));
      try{await withDeadline(candidate.ready,VIEWER_READY_DEADLINE_MS,'Rendering worker');viewer=candidate;candidate.presentationVisible(viewerPresented);world.onBuild=(environment,detail,preservePresentation)=>candidate.configure(environment,detail,project.carsEnabled??true,project.peopleEnabled??true,preservePresentation);world.onPose=truth=>candidate.pose(truth);world.onOtherPose=truth=>candidate.otherPose(truth);world.onVehicleCount=count=>candidate.vehicleCount(count);world.onActors=(cars,people)=>candidate.actors(cars,people);world.onActorMotion=frame=>candidate.actorMotion(frame);world.onEnvironmentVisible=visible=>candidate.sceneVisible(visible);world.onLighting=mode=>candidate.lighting(mode);await world.enableSensorWorker();}
      catch(error){candidate.worker.terminate();candidate.canvas.remove();viewer=undefined;el('world').appendChild(world.renderer.domElement);world.controls.disconnect();world.controls.connect(world.renderer.domElement);world.onBuild=async()=>{};world.onPose=()=>{};world.onOtherPose=()=>{};world.onVehicleCount=async()=>{};world.onActors=()=>{};world.onActorMotion=()=>{};world.onEnvironmentVisible=()=>{};world.onLighting=()=>{};status(`Rendering worker unavailable: ${error} · using main-thread renderer`);}
    }
    await buildProjectScene();
    initialized=true;finishInitialization();
    startup.stage(2,'Compiling your Modelica model…');
    if(openingError&&savedText!==undefined)await recoverDefaultModels(savedText,openingError);
    else{
      // Browsers that block site storage still run the defaults; only saving is disabled.
      await withDeadline(compile(),STARTUP_COMPILE_DEADLINE_MS,'Compiling the Modelica model','a slow phone may need more time, so reloading the page may help');
      if(!ready&&selectedNativeDefault){project.slamBackend='inertial';nativeDefaultFailed=true;await compile();}
      if(!ready&&savedText!==undefined&&compilationError!==undefined)await recoverDefaultModels(savedText,compilationError);
      else if(ready)status(nativeDefaultFailed?'Ready · native SLAM could not start; inertial baseline selected':
        runtime.activeVehicle.slamProgram?'Ready · click Run to start RGB-D SLAM':
        localPersistenceBlocked?'Ready · click Run to start · this browser blocks site storage, so download your project to keep changes':
        'Ready · click Run to start the Modelica inertial experiment');
    }
    if(!ready){startup.fail(el('status').textContent);return;}
    startup.stage(3,'Preparing the editor and first view…');
    await modelsReady;
    if(viewer)await viewer.present();else world.render();
    startup.complete();
  }
  catch(error){status(String(error),true);startup.fail(error);}
})();
