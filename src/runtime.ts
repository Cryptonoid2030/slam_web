import {DataFlow} from './data-flow';
import { WorkerRpc } from './rpc';
import { ports,portTopic,validateGraph,type PortType } from './graph';
import {seededRandom,recordedCameraFrame,validateSensorObservation} from './packet';
import { World } from './world';
import {defaultSensorModelica,defaultEvaluationModelica,withActorMotion,offsetQuadrotorPath,estimatorModelicaSources,type Project} from './project';
import offsetQuadrotor from '../models/Vehicles/OffsetQuadrotor.mo?raw';
import {LockstepBarrier} from './lockstep-barrier';
import {ModelicaSlamClient,type SlamReady} from './modelica-slam-client';
import type {ActorMotionFrame} from './modelica-actor-motion';
import type { Command,Estimate,Pose,RunRecord,SensorFrame,InitialSensorFrame,Truth,SceneDetail } from './types';
import {validatePose} from './evaluation';
import type {EvaluationResult} from './modelica-runtime-math';
import type {InertialSessionMetadata} from './modelica-inertial-session';
import type {GpsSample} from './world-navigation';
import {SensorClock,sensorRates,validateSensorRates,type SensorRates} from './sensor-clock';
import {checkedLoopClosureSettings,type LoopClosureSettings} from './loop-closure-settings';
import {nativeParametersFor,NATIVE_SLAM_PARAMETERS,type ParameterValues} from './model-parameters';
import {FrameClock} from './frame-clock';
import {runtimeProjectKey} from './runtime-project-key';
import {selectedFlightTour,type FlightTour} from './flight-tour';
import type {LiveFeed} from './live-source';
import type {NativeVariant} from './native-assets';
export interface Metrics { time:number; ate:number|null; currentError:number|null; orientationError:number|null; confidence:number; points:number; features:number; rtf:number; frames:number; loops:number; dropped:number }
export interface FlowTiming {totalMs:number;bufferBytes:number|null}
export interface VehicleFrame {frame:SensorFrame;estimate:Estimate;truth:Truth|undefined;metrics:Metrics}
export class Runtime {
  readonly flow=new DataFlow();
  private physics=new WorkerRpc(new Worker(new URL('./physics.worker.ts',import.meta.url),{type:'module'}));
  readonly pendingNodes=new Map<string,string>();
  private modelicaState?:WorkerRpc;
  private slam?:ModelicaSlamClient;
  /** Published native SLAM program identity once the session is initialized. */
  slamProgram?:SlamReady;
  private modelicaMath?:WorkerRpc;
  private project?:Project;
  private order:ReturnType<typeof validateGraph>=[];
  private random=seededRandom(7);
  private gpsRandom=seededRandom(9);
  gps:GpsSample|null=null;
  tourMode:FlightTour='circuit';
  private sequence=0;
  private cameraSequence=0;
  private stepping=false;
  private resetting=false;
  private playbackEpoch=0;
  private reference?:{truth:Truth;pose:Pose};
  private origin:Pose={x:0,y:0,z:1.5,quaternion:[1,0,0,0]};
  private startWall=0;
  private startTime=0;
  private sensorClock=new SensorClock(sensorRates({}));
  private heldImu:SensorFrame['imu']={accel:[0,0,9.80665],gyro:[0,0,0]};
  private cameraLidar=false;
  private imuIntervals:NonNullable<SensorFrame['imuIntervals']>=[];
  private replay?:RunRecord[];
  /** Live phone capture: records are pulled one at a time into a sparse `replay`. */
  private live?:LiveFeed;
  /** Which published native program set the next compile loads: raster input or phone landmarks. */
  nativeVariant:NativeVariant='image';
  private externalStep=false;
  private secondary?:Runtime;
  private fleetStepping=false;
  private fleetFailure?:Error;
  private selectedVehicle:0|1=0;
  lastFrame?:VehicleFrame;
  private pendingFrame?:VehicleFrame;
  private pendingRecord?:RunRecord;
  records:RunRecord[]=[];
  recording=false;
  running=false;
  speed=1;
  /** Diagnostic switch for matched serial/overlapped browser measurements. */
  overlapSlamPrediction=true;
  dt=1/90;
  time=0;
  lastTimings:Record<string,number>={};
  /** Wall pacing of completed lockstep frames, including both vehicles. */
  readonly frameClock=new FrameClock();
  private pacingSince?:number;
  private pacedWaitMs=0;
  lastWorkerTimings:Record<string,Record<string,number>>={};
  lastTransportTimings:Record<string,FlowTiming>={};
  command:Command={forward:0,left:0,up:0,yaw:0};
  autopilot=true;
  onFrame:(frame:SensorFrame,estimate:Estimate,truth:Truth|undefined,metrics:Metrics)=>void=()=>{};
  onError:(error:Error)=>void=()=>{};
  onStatus:(status:string)=>void=()=>{};
  onSensor:()=>void=()=>{};
  onLidar:()=>void=()=>{};
  onVehicleFrame:(vehicle:0|1,data:VehicleFrame)=>void=()=>{};
  onInitialFrame:(vehicle:0|1,frame:InitialSensorFrame,estimate:Estimate)=>void=()=>{};
  constructor(readonly world:World,readonly vehicleIndex:0|1=0) {}
  get activeVehicle():Runtime{return this.getVehicle(this.selectedVehicle);}
  getVehicle(index:0|1):Runtime{
    if(index!==0&&index!==1)throw new Error('Unknown vehicle');
    if(index===1&&!this.secondary)throw new Error('The second vehicle is not enabled');
    return index===1?this.secondary!:this;
  }
  get vehicleCount():1|2{return this.secondary?2:1;}
  get activeVehicleIndex(){return this.selectedVehicle;}
  setFlightTour(route:FlightTour){
    this.tourMode=route;if(this.project)this.project.flightTour=route;
    if(this.secondary)this.secondary.setFlightTour(route);
  }
  selectVehicle(index:0|1){
    this.getVehicle(index);
    this.command={forward:0,left:0,up:0,yaw:0};this.selectedVehicle=index;this.world.cameraVehicle=index;
    const data=this.activeVehicle.lastFrame;
    if(data)this.onFrame(data.frame,data.estimate,data.truth,data.metrics);
  }
  dispose(){
    this.pause();this.physics.stop();this.modelicaState?.stop();this.modelicaMath?.stop();this.slam?.close();
    this.secondary?.dispose();this.secondary=undefined;this.world.dispose();
  }
  async setGraphicsQuality(detail:SceneDetail) {
    if(this.running||this.busy)throw new Error('Pause and finish the current lockstep frame before changing graphics quality');
    await this.world.setGraphicsQuality(detail);
    if(this.secondary)await this.secondary.setGraphicsQuality(detail);
    if(this.project)this.project.sceneDetail=detail;
    this.resetSensorClock();
  }
  setSensorRates(rates?:SensorRates){
    if(this.running||this.busy)throw new Error('Pause and finish the current lockstep frame before changing sensor rates');
    if(rates)validateSensorRates(rates);
    if(this.project){if(rates)this.project.sensorRates=structuredClone(rates);else delete this.project.sensorRates;}
    this.resetSensorClock(rates);
    this.secondary?.setSensorRates(rates);
  }
  private resetSensorClock(rates?:SensorRates){
    const configuration=sensorRates({...this.project,...(rates?{sensorRates:rates}:{})});
    this.sensorClock=new SensorClock(configuration,this.time);this.dt=1/configuration.cameraHz;
  }
  async compile(project:Project) {
    this.pause();
    if(this.busy) throw new Error('Wait for the current lockstep frame to finish');
    this.secondary?.dispose();this.secondary=undefined;this.selectedVehicle=0;this.lastFrame=undefined;
    this.fleetFailure=undefined;this.pendingFrame=undefined;this.pendingRecord=undefined;
    await this.world.setVehicleCount(1);
    const order=validateGraph(project.graph);
    await this.world.setCameraResolution(project.cameraResolution??'848x480');
    if(order.some(node=>node.kind==='modelica'))throw new Error('Custom Modelica graph-node execution is pending. Source editing, Rumoca diagnostics and project persistence are available.');
    this.onStatus('Compiling Modelica in Rumoca WASM…');
    const base=new URL(import.meta.env.BASE_URL,location.href).href;
    this.project=undefined;
    this.physics.stop();
    this.pendingNodes.clear();delete project.detectorArtifact;
    this.modelicaState?.stop();this.modelicaState=undefined;
    this.modelicaMath?.stop();this.modelicaMath=undefined;
    this.slam?.close();this.slam=undefined;this.slamProgram=undefined;
    this.physics=new WorkerRpc(new Worker(new URL('./physics.worker.ts',import.meta.url),{type:'module'}));
    const workspaceSources={...project.modelicaSources};
    delete workspaceSources['models/Vehicles/LabQuadrotor.mo'];
    const spawnSource=workspaceSources[offsetQuadrotorPath]??offsetQuadrotor;
    if(this.vehicleIndex===1)delete workspaceSources[offsetQuadrotorPath];
    const truth=await this.physics.call<Truth>('init',{base,source:this.vehicleIndex===1?project.physics+'\n'+spawnSource:project.physics,modelName:this.vehicleIndex===1?'OffsetQuadrotor':'LabQuadrotor',workspaceSources,parameters:project.modelParameters?.LabQuadrotor});
    validatePose(truth);this.origin={x:truth.x,y:truth.y,z:truth.z,quaternion:[...truth.quaternion]};
    this.modelicaMath=new WorkerRpc(new Worker(new URL('./modelica-runtime-math.worker.ts',import.meta.url),{type:'module'}));
    project.sensorModelica=withActorMotion(project.sensorModelica??defaultSensorModelica);
    // Recycle beyond the storefront block and before the training enclosure.
    const actorRoute=project.environment==='city'?{pedestrianRadius:7.1,carRadius:1.65,pedestrianCrossings:true,streetCars:true,carStreetHalfLength:33}:project.environment==='big-city'?{pedestrianRadius:5.8,carRadius:1.65}:undefined;
    await this.modelicaMath.call('init',{base,sensorSource:project.sensorModelica??defaultSensorModelica,evaluationSource:project.evaluationModelica??defaultEvaluationModelica,origin:this.origin,actorRoute,parameters:project.modelParameters},180_000);
    this.onStatus('Compiling selected Modelica model to WASM…');
    for(const node of order) if(['detector','slam'].includes(node.kind)) {
      if(node.kind==='detector'&&project.detectorLanguage==='modelica') {
        // The native SLAM programs detect features inside the compiled model.
        if(project.slamBackend!=='native')this.pendingNodes.set(node.id,'Rumoca native full-frame feature detection pending');
      } else if(node.kind==='slam'&&project.runtime==='modelica') {
        this.modelicaState=new WorkerRpc(new Worker(new URL('./modelica-state.worker.ts',import.meta.url),{type:'module'}));
        const result=await this.modelicaState.call<{artifact:InertialSessionMetadata}>('init',{base,source:project.algorithm,workspaceSources:estimatorModelicaSources(project),modelName:project.entryPoint??'ModelicaInertial',parameters:project.modelParameters?.[project.entryPoint??'ModelicaInertial']},180_000);
        project.algorithmArtifact=result.artifact;
      }
    }
    this.world.build(project.environment,project.sceneDetail??'high');await this.world.ready;this.world.configureActors(project.carsEnabled??true,project.peopleEnabled??true);this.world.setDepthCloudEnabled(project.depthCloudEnabled??false);this.world.update(truth);this.world.setLighting(project.lightingMode??'day');this.world.setMap([]);
    const calibration=this.world.calibration;
    await this.world.setDepthNoise({seed:project.seed,disparityNoisePx:calibration.depthNoiseDisparityPx!,referenceFx:calibration.depthNoiseReferenceFx!,baselineMeters:calibration.baseline,dropoutProbability:.005,unitsMeters:.001});
    this.world.setActorMotion(await this.modelicaMath.call<ActorMotionFrame>('actors',{time:truth.time}));
    this.world.setEstimate({x:0,y:0,z:0,quaternion:[1,0,0,0],confidence:0,points:[]});
    this.world.setEnvironmentVisible(true);
    this.random=seededRandom(project.seed);this.gpsRandom=seededRandom(project.seed^0x475053);this.gps=null;this.sequence=0;this.cameraSequence=0;this.time=truth.time;
    this.resetSensorClock(sensorRates(project));
    this.heldImu=(await this.sampleSensors(truth,true,false)).imu;
    this.reference=undefined;this.records=[];this.replay=undefined;this.releaseLive();
    this.project=structuredClone(project);this.order=order;this.tourMode=selectedFlightTour(project);
    if(project.slamBackend==='native'&&order.some(node=>node.kind==='slam')){
      this.onStatus('Loading native SLAM programs…');
      const client=new ModelicaSlamClient(text=>this.onStatus(text));
      try{
        const initial=await this.captureInitialFrame();
        this.slamProgram=await client.initialize(base,initial,project.loopClosureSettings,nativeParametersFor(project),this.nativeVariant);
        this.slam=client;
        this.onInitialFrame(this.vehicleIndex,initial,this.slamProgram.estimate);
      }catch(error){client.close();this.project=undefined;throw error;}
    }
    if(this.vehicleIndex===0&&project.vehicleCount===2){
      const otherWorld=new World(document.createElement('div'));
      let candidate:Runtime|undefined;
      try{
        const other=candidate=new Runtime(otherWorld,1);
        other.onStatus=message=>this.onStatus('Drone 2 · '+message);
        other.onSensor=()=>this.onSensor();other.onLidar=()=>this.onLidar();
        other.onVehicleFrame=(_index,data)=>this.onVehicleFrame(1,data);
        other.onInitialFrame=(_index,frame,estimate)=>this.onInitialFrame(1,frame,estimate);
        other.onFrame=(frame,estimate,truth,metrics)=>{if(this.selectedVehicle===1)this.onFrame(frame,estimate,truth,metrics);};
        await other.compile({...structuredClone(project),vehicleCount:1,seed:project.seed^0x44524f4e});
        if(this.world.graphics.acceleration!=='software'&&typeof OffscreenCanvas!=='undefined')await otherWorld.enableSensorWorker();
        await Promise.all([this.world.setVehicleCount(2),otherWorld.setVehicleCount(2)]);
        this.secondary=other;this.publishVehiclePoses();
      }catch(error){if(candidate)candidate.dispose();else otherWorld.dispose();await this.world.setVehicleCount(1);this.project=undefined;throw error;}
    }
    this.onStatus(this.slam?`Ready · camera + native Modelica SLAM (${this.slamProgram?.models.step.model}) · inertial baseline retained`:'Ready · camera + Modelica inertial baseline · native feature detection and SLAM pending');
  }
  async prepare(project:Project,reusePrepared=false){
    if(reusePrepared&&this.canReset(project))await this.reset(project);
    else await this.compile(project);
  }
  canReset(project:Project){
    return !!this.project&&!this.fleetFailure&&runtimeProjectKey(project)===runtimeProjectKey(this.project);
  }
  /** Reinitialize the prepared worker programs, never the compiler. */
  async reset(project:Project){
    this.pause();
    if(this.busy)throw new Error('Wait for the current lockstep frame to finish');
    if(!this.canReset(project))throw new Error('The model changed or is unavailable; compile it before resetting');
    this.resetting=true;
    try{
      await this.resetVehicle(project);
      if(this.secondary)await this.secondary.reset({...structuredClone(project),vehicleCount:1,seed:project.seed^0x44524f4e});
      this.selectedVehicle=0;this.world.cameraVehicle=0;
      this.publishVehiclePoses();
      this.onStatus('Simulation reset · compiled models reused · press Run to continue');
    }catch(error){
      // A partially reset fleet must never be stepped or reused as a valid run.
      this.project=undefined;if(this.secondary)this.secondary.project=undefined;
      throw error;
    }finally{this.resetting=false;}
  }
  private async resetVehicle(project:Project){
    const truth=await this.physics.call<Truth>('reset');validatePose(truth);
    if(truth.time!==0)throw new Error('Physics reset did not return to time zero');
    this.origin={x:truth.x,y:truth.y,z:truth.z,quaternion:[...truth.quaternion]};
    const actorRoute=project.environment==='city'?{pedestrianRadius:7.1,carRadius:1.65,pedestrianCrossings:true,streetCars:true,carStreetHalfLength:33}:project.environment==='big-city'?{pedestrianRadius:5.8,carRadius:1.65}:undefined;
    await this.modelicaMath!.call('reset',{origin:this.origin,actorRoute});
    if(this.modelicaState)await this.modelicaState.call('reset');
    this.world.build(project.environment,project.sceneDetail??'high');await this.world.ready;
    this.world.configureActors(project.carsEnabled??true,project.peopleEnabled??true);
    this.world.setDepthCloudEnabled(project.depthCloudEnabled??false);this.world.setLighting(project.lightingMode??'day');
    this.world.update(truth);this.world.setMap([]);
    const calibration=this.world.calibration;
    await this.world.setDepthNoise({seed:project.seed,disparityNoisePx:calibration.depthNoiseDisparityPx!,referenceFx:calibration.depthNoiseReferenceFx!,baselineMeters:calibration.baseline,dropoutProbability:.005,unitsMeters:.001});
    this.world.setActorMotion(await this.modelicaMath!.call<ActorMotionFrame>('actors',{time:0}));
    this.world.setEstimate({x:0,y:0,z:0,quaternion:[1,0,0,0],confidence:0,points:[]});
    this.world.setEnvironmentVisible(true);
    this.project=structuredClone(project);this.order=validateGraph(project.graph);this.tourMode=selectedFlightTour(project);
    this.random=seededRandom(project.seed);this.gpsRandom=seededRandom(project.seed^0x475053);this.gps=null;
    this.sequence=0;this.cameraSequence=0;this.time=0;this.resetSensorClock(sensorRates(project));
    this.imuIntervals=[];this.cameraLidar=false;this.heldImu=(await this.sampleSensors(truth,true,false)).imu;
    this.reference=undefined;this.records=[];this.replay=undefined;this.releaseLive();this.lastFrame=undefined;
    this.pendingFrame=undefined;this.pendingRecord=undefined;this.externalStep=false;
    this.command={forward:0,left:0,up:0,yaw:0};this.flow.stats.clear();
    this.frameClock.reset();this.pacingSince=undefined;this.pacedWaitMs=0;this.startWall=0;this.startTime=0;
    this.lastTimings={};this.lastWorkerTimings={};this.lastTransportTimings={};
    if(this.slam){
      const frame=await this.captureInitialFrame();
      const estimate=await this.slam.reset(frame,project.loopClosureSettings,nativeParametersFor(project));
      this.slamProgram={...this.slamProgram!,estimate};
      this.onInitialFrame(this.vehicleIndex,frame,estimate);
    }
  }
  /** Optional full-SLAM initialization image. It consumes no physics/sensor tick
   * and publishes no estimate. A failed capture leaves the image ID available. */
  async captureInitialFrame():Promise<InitialSensorFrame> {
    if(!this.project)throw new Error('Compile the project first');
    if(this.running||this.stepping||this.time!==0||this.sequence!==0||this.cameraSequence!==0||this.replay)
      throw new Error('Initial camera capture requires a paused, fresh time-zero experiment');
    this.stepping=true;
    try{
      const [images,scan]=await this.world.captureSensors(false);
      if(scan)throw new Error('Initial camera capture must not advance LiDAR');
      const frame:InitialSensorFrame={rgb:images.rgb,depth:images.depth,imageLayout:images.imageLayout,
        sequence:this.cameraSequence,time:this.time,dt:0,
        calibration:{...this.world.calibration,depthEncoding:'axial-z16-le'},
        imu:structuredClone(this.heldImu),imuIntervals:[]};
      validateSensorObservation(frame);
      this.cameraSequence++;
      return frame;
    }finally{this.stepping=false;}
  }
  private async sampleSensors(truth:Truth,imu:boolean,gps:boolean){
    const pairs=(random:()=>number)=>Array.from({length:3},()=>[random(),random()]);
    const neutral=[[.5,.5],[.5,.5],[.5,.5]];
    const draws:{accel:number[][];gyro:number[][];gps?:number[][]}={accel:imu?pairs(this.random):neutral,gyro:imu?pairs(this.random):neutral};
    const roof=this.world.navigationRoofs,enabled=this.world.navigationEnabled;
    if(gps&&await this.modelicaMath!.call<boolean>('availability',{truth,roof,enabled}))draws.gps=pairs(this.gpsRandom);
    return this.modelicaMath!.call<{imu:SensorFrame['imu'];gps:GpsSample|null}>('sensors',{truth,roof,enabled,draws,sampleGps:gps});
  }
  private deliveredLidar(scan:{samples:Float32Array}){this.onLidar();this.flow.record('lab/sensors/lidar/0',scan.samples.byteLength);}
  private deliver(type:PortType,port:string,value:any) {
    const started=performance.now();
    const bufferBytes=type==='frame'?value.rgb.byteLength+value.depth.byteLength:null;
    this.flow.record(port,bufferBytes);
    this.lastTransportTimings[port]={totalMs:performance.now()-started,bufferBytes};
    return value;
  }
  private publishVehiclePoses(){
    if(this.secondary&&this.world.truth&&this.secondary.world.truth){
      this.world.updateOtherVehicle(this.secondary.world.truth);
      this.secondary.world.updateOtherVehicle(this.world.truth);
    }
  }
  async step(){
    const started=performance.now(),sequence=this.sequence,paced=this.pacedWaitMs;
    this.pacedWaitMs=0;
    await this.stepFrame();
    if(this.sequence!==sequence)this.frameClock.record(started,performance.now(),this.time,paced);
  }
  private async stepFrame(){
    if(this.resetting)throw new Error('Wait for simulation reset to finish');
    if(this.fleetFailure)throw this.fleetFailure;
    if(this.fleetStepping)throw new Error('A fleet lockstep frame is already executing');
    if(!this.secondary)return this.stepVehicle();
    if(this.replay)throw new Error('Dataset replay requires one vehicle');
    this.fleetStepping=true;
    const other=this.secondary,barrier=new LockstepBarrier(2,()=>this.publishVehiclePoses());
    other.autopilot=this.autopilot;other.tourMode=this.tourMode;other.speed=this.speed;
    other.command=this.selectedVehicle===1?{...this.command}:{forward:0,left:0,up:0,yaw:0};
    const advance=(vehicle:Runtime)=>vehicle.stepVehicle(barrier).catch(error=>{barrier.abort(error instanceof Error?error:new Error(String(error)));throw error;});
    try{
      const results=await Promise.allSettled([advance(this),advance(other)]);
      const failure=results.find(result=>result.status==='rejected');
      if(failure?.status==='rejected')throw failure.reason;
      if(this.time!==other.time)throw new Error('Vehicle clocks diverged');
      if(this.pendingFrame)this.lastFrame=this.pendingFrame;
      if(other.pendingFrame)other.lastFrame=other.pendingFrame;
      this.publishFrame();other.publishFrame();
    }catch(error){
      this.fleetFailure=new Error('Recompile after the failed fleet frame: '+String(error));throw error;
    }finally{this.pendingFrame=undefined;other.pendingFrame=undefined;this.pendingRecord=undefined;other.pendingRecord=undefined;this.fleetStepping=false;}
  }
  private publishFrame(){
    const data=this.lastFrame;if(!data)return;
    if(this.pendingRecord){this.records.push(this.pendingRecord);this.pendingRecord=undefined;}
    this.onVehicleFrame(this.vehicleIndex,data);
    if(!this.secondary||this.selectedVehicle===0)this.onFrame(data.frame,data.estimate,data.truth,data.metrics);
  }
  private async stepVehicle(barrier?:LockstepBarrier) {
    if(this.stepping) throw new Error('A lockstep frame is already executing');
    if(!this.project) throw new Error('Compile the project first');
    if(this.live){
      // Block until the phone delivers the next frame (or pause/reset releases us with none).
      const record=await this.live.next();
      if(!record||!this.live)return;
      this.replay![this.sequence]=record;
      if(this.sequence>0)delete this.replay![this.sequence-1];
    }
    if(this.replay && this.sequence>=this.replay.length) {this.running=false;this.onStatus('Replay finished');return;}
    this.stepping=true;
    this.lastWorkerTimings={};this.lastTransportTimings={};
    this.externalStep=!!this.replay&&!this.replay[this.sequence]?.truth;
    this.world.setEnvironmentVisible(!this.externalStep);
    let prediction:Promise<unknown>|undefined;
    const overlap=this.overlapSlamPrediction&&this.slam&&!this.replay&&!barrier;
    try {
      const values=new Map<string,any>();
      let truth:Truth|undefined,frame!:SensorFrame,estimate!:Estimate;
      let sensorGps:GpsSample|null=null;
      let command=this.secondary&&this.selectedVehicle===1?{forward:0,left:0,up:0,yaw:0}:{...this.command};
      for(const node of this.order) {
        const nodeStart=performance.now();
        const inputs:Record<string,any>={};
        for(const edge of this.project.graph.edges.filter(e=>e.to===node.id)) inputs[edge.input]=values.get(`${edge.from}/${edge.output}`);
        let outputs:Record<string,any>={};
        switch(node.kind) {
          case 'physics':
            if(!this.externalStep){
              this.imuIntervals=[];this.cameraLidar=false;
              if(this.replay)truth=this.replay[this.sequence].truth;
              else {
                let intervalStart=this.time;const commandTime=this.time;
                for(const event of this.sensorClock.nextFrame(this.project.lidarEnabled??false)){
                  truth=await this.physics.call<Truth>('step',{time:event.time,command,autopilot:this.autopilot,indoorTour:this.tourMode==='indoor',tourMode:this.tourMode,commandTime});
                  this.world.update(truth);
                  if(barrier)await barrier.arrive();
                  // Zero-order hold: an endpoint IMU measurement applies to
                  // the next interval. Numerical integration stays in WASM.
                  if(event.imu||event.camera){
                    this.imuIntervals.push({time:event.time,dt:event.time-intervalStart,imu:this.heldImu});intervalStart=event.time;
                  }
                  if(event.imu||event.gps){
                    const observed=await this.sampleSensors(truth,event.imu,event.gps);
                    if(event.imu){this.heldImu=observed.imu;this.flow.record('lab/sensors/imu/0');}
                    if(event.gps){this.gps=observed.gps;this.flow.record('lab/navigation/gps');}
                  }
                  if(event.lidar&&!event.camera){
                    this.world.setActorMotion(await this.modelicaMath!.call<ActorMotionFrame>('actors',{time:truth.time}));
                    this.deliveredLidar(await this.world.captureLidarOnly());
                  }
                  if(event.camera)this.cameraLidar=event.lidar;
                  if(barrier)await barrier.arrive();
                  if(overlap&&event.imu&&!event.camera){
                    // Keep at most one prediction in flight. The next physics
                    // interval can run on its own worker in the meantime.
                    if(prediction)await prediction;
                    prediction=this.slam!.predict(this.cameraSequence,this.imuIntervals.at(-1)!);
                    // Observe rejection immediately, then propagate it before
                    // publishing the frame; never leave an unhandled promise.
                    void prediction.catch(()=>{});
                  }
                }
                if(prediction)await prediction;
              }
              if(!truth)throw new Error('Missing committed camera pose');
              command=truth.command??command;
              if(!this.replay)this.lastWorkerTimings[node.id]={...this.physics.lastTimings};
              this.time=truth.time;this.world.update(truth);outputs={truth};
            }
            break;
          case 'sensor': {
            const t=inputs.truth as Truth;
            if(t&&!this.externalStep)this.world.setActorMotion(await this.modelicaMath!.call<ActorMotionFrame>('actors',{time:t.time}));
            if(this.replay) frame=this.replay[this.sequence].frame;
            else {
              const [images,scan]=await this.world.captureSensors(this.cameraLidar);
              if(scan)this.deliveredLidar(scan);
              frame={...images,sequence:this.cameraSequence++,time:t.time,dt:this.dt,calibration:{...this.world.calibration,depthEncoding:'axial-z16-le'},imu:this.heldImu,imuIntervals:this.imuIntervals};
              frame.capture={sensorProfile:this.world.sensorProfile,depthNoise:{model:'independent-pixel-hash-v1',seed:this.project.seed,tick:Math.round(t.time*180),unitsMeters:.001}};
              sensorGps=this.gps;
            }
            this.time=frame.time;
            this.onSensor();
            this.gps=sensorGps;
            if(this.replay&&t&&!this.externalStep){
              const roof=this.world.navigationRoofs,enabled=this.world.navigationEnabled;
              const available=await this.modelicaMath!.call<boolean>('availability',{truth:t,roof,enabled});
              const draws={accel:[[.5,.5],[.5,.5],[.5,.5]],gyro:[[.5,.5],[.5,.5],[.5,.5]],...(available?{gps:Array.from({length:3},()=>[this.gpsRandom(),this.gpsRandom()])}:{})};
              const observed=await this.modelicaMath!.call<{gps:GpsSample|null}>('sensors',{truth:t,roof,enabled,draws});
              this.gps=observed.gps;
            }
            // Physical envelopes retain their measured clock metadata; don't
            // relabel an interpolated measurement as a local-freerun raw IMU.
            this.flow.record('lab/camera/info');
            outputs={frame};break;
          }
          case 'detector': {
            // The nominal INS baseline does not consume feature observations.
            // Emit no feature result or activity count for an unavailable node.
            continue;
          }
          case 'slam': {
            const worker=this.modelicaState!,sample=inputs.frame as SensorFrame;
            // The admitted state backend is nominal INS. Its issued interface
            // consumes IMU/time only; avoid cloning RGB/depth into that worker.
            // A future visual estimator must declare its own image inputs.
            const frame={time:sample.time,dt:sample.dt,imu:sample.imu,imuIntervals:sample.imuIntervals};
            if(this.slam){
              // The native programs consume the full RGB8/Z16 frame with its IMU intervals.
              const result=await this.slam.step(sample);
              estimate=result.estimate;this.lastWorkerTimings[node.id]={...this.slam.lastTimings,stepMsTotal:result.stepMs.reduce((a,b)=>a+b,0)};
              outputs={estimate};break;
            }
            estimate=await worker.call<Estimate>('step',{nodeId:node.id,kind:node.kind,frame,features:inputs.features});
            this.lastWorkerTimings[node.id]={...worker.lastTimings};outputs={estimate};break;
          }
          case 'modelica': throw new Error('Custom Modelica graph-node execution is pending');
          case 'map': estimate=inputs.estimate;this.world.setMap(estimate.points,this.externalStep?undefined:this.origin);this.world.setEstimate(estimate);if(this.externalStep)this.world.update({...estimate,time:frame.time,velocity:[0,0,0],accel:frame.imu.accel,gyro:frame.imu.gyro});outputs={estimate};break;
          case 'evaluation': {
            const e=inputs.estimate as Estimate,t=inputs.truth as Truth;
            if(!t){outputs={metrics:{ate:null,currentError:null,orientationError:null}};break;}
            const evaluated=await this.modelicaMath!.call<EvaluationResult>('evaluation',{truth:t,estimate:e,sampleCount:this.sequence+1});
            this.reference={truth:t,pose:evaluated.reference};
            this.lastWorkerTimings[node.id]={...this.modelicaMath!.lastTimings};
            outputs={metrics:evaluated.metrics};break;
          }
        }
        for(const [port,value] of Object.entries(outputs)) {
          const output=this.deliver(ports(node).outputs[port],portTopic(node.id,port),value);
          values.set(`${node.id}/${port}`,output);
        }
        this.lastTimings[node.id]=performance.now()-nodeStart;
      }
      if(!estimate) throw new Error('Graph must produce a SLAM estimate');
      const metricNode=this.order.find(n=>n.kind==='evaluation');
      if(truth&&!metricNode){const evaluated=await this.modelicaMath!.call<EvaluationResult>('evaluation',{truth,estimate,sampleCount:this.sequence+1,accumulate:false});this.reference={truth,pose:evaluated.reference};}
      const metrics=metricNode?values.get(`${metricNode.id}/metrics`):{ate:null,currentError:null,orientationError:null};
      this.flow.record('lab/slam/odometry');
      if(this.recording) {
        if(this.records.length<300) this.pendingRecord={frame:recordedCameraFrame(frame),truth,estimate,command};
        else { this.recording=false;this.onStatus('Recording stopped at 300 frames; export the dataset'); }
      }
      this.sequence++;
      const wall=(performance.now()-this.startWall)/1000;
      const completed={...metrics,time:frame.time,confidence:estimate.confidence,points:estimate.points.length,features:estimate.features?.length??0,rtf:wall>0?(frame.time-this.startTime)/wall:0,frames:this.sequence,loops:Number(estimate.diagnostics?.loops??0),dropped:0};
      const delivered={frame,estimate,truth,metrics:completed};
      if(barrier)this.pendingFrame=delivered;
      else{this.lastFrame=delivered;this.publishFrame();}
    } finally {if(prediction)await prediction.catch(()=>{});this.stepping=false;this.externalStep=false;}
  }
  /** Removes wall time spent with the page hidden from the sim/wall rates. */
  excludeHiddenWall(ms:number){
    if(!this.running||!(ms>0))return;
    this.startWall+=ms;if(this.secondary)this.secondary.startWall+=ms;
    this.frameClock.reset();
  }
  play() {
    if(this.resetting)throw new Error('Wait for simulation reset to finish');
    if(this.running) return;
    this.running=true;this.startWall=performance.now();this.startTime=this.time;this.frameClock.reset();
    this.pacingSince=undefined;this.pacedWaitMs=0;
    const epoch=++this.playbackEpoch;
    const active=()=>this.running&&this.playbackEpoch===epoch;
    if(this.secondary){this.secondary.startWall=this.startWall;this.secondary.startTime=this.secondary.time;}
    const loop=async()=>{
      if(!active())return;
      // A pause/resume can leave an earlier epoch finishing its last frame.
      if(this.busy){setTimeout(loop,0);return;}
      if(this.pacingSince!==undefined){this.pacedWaitMs=performance.now()-this.pacingSince;this.pacingSince=undefined;}
      const deadline=performance.now()+8;
      do{
        const before=performance.now();
        try {await this.step();}
        catch(error){
          // An older epoch still owns the same solver state. Its failure must
          // also stop a resumed run before that state is stepped again.
          this.pause();this.onError(error instanceof Error?error:new Error(String(error)));return;
        }
        if(!active())return;
        const delay=this.speed===Infinity?0:Math.max(0,this.dt*1000/this.speed-(performance.now()-before));
        // Worker replies already yield to the event loop. Batch completed
        // frames for at most 8 ms instead of adding a clamped timer per frame.
        // Every acquisition/estimator event is still awaited by step().
        if(delay>0||performance.now()>=deadline){if(delay>0)this.pacingSince=performance.now();setTimeout(loop,delay);return;}
      }while(active());
    };
    void loop();
  }
  pause() {this.running=false;this.playbackEpoch++;this.live?.wake();}
  async configureNativeParameters(values:ParameterValues){
    if(this.running||this.busy)throw Error('Pause at a completed frame before changing model parameters');
    const vehicles=this.secondary?[this,this.secondary]:[this];
    const previous=vehicles.map(vehicle=>nativeParametersFor(vehicle.project??{}));
    const results=await Promise.allSettled(vehicles.map(vehicle=>vehicle.slam?.configureParameters(values)));
    const failed=results.find(result=>result.status==='rejected');
    if(failed?.status==='rejected'){
      await Promise.allSettled(vehicles.map((vehicle,index)=>results[index].status==='fulfilled'?vehicle.slam?.configureParameters(previous[index]):undefined));throw failed.reason;
    }
    for(const vehicle of vehicles)if(vehicle.project){vehicle.project.modelParameters??={};vehicle.project.modelParameters[NATIVE_SLAM_PARAMETERS]={...values};}
  }
  async configureLoopClosure(settings:LoopClosureSettings){
    const checked=checkedLoopClosureSettings(settings);
    if(this.busy||this.secondary?.busy)throw Error('Pause at a completed frame before applying loop tuning');
    const vehicles=this.secondary?[this,this.secondary]:[this];
    for(const vehicle of vehicles)if(vehicle.slamProgram&&!vehicle.slamProgram.tuningSupported)
      throw Error('This SLAM delivery does not expose tuning inputs. Load the current default models before applying tuning.');
    const previous=vehicles.map(vehicle=>checkedLoopClosureSettings(vehicle.project?.loopClosureSettings));
    const results=await Promise.allSettled(vehicles.map(vehicle=>vehicle.slam?.configure(checked)));
    const failure=results.find(result=>result.status==='rejected');
    if(failure?.status==='rejected'){
      await Promise.allSettled(vehicles.map((vehicle,index)=>results[index].status==='fulfilled'?vehicle.slam?.configure(previous[index]):undefined));
      throw failure.reason;
    }
    for(const vehicle of vehicles)if(vehicle.project)vehicle.project.loopClosureSettings={...checked};
  }
  get busy():boolean{return this.resetting||this.stepping||this.fleetStepping||!!this.secondary?.busy;}
  get evaluationOrigin(){return structuredClone(this.origin);}
  referenceTruth(truth:Pose){
    if(!this.reference||truth.x!==this.reference.truth.x||truth.y!==this.reference.truth.y||truth.z!==this.reference.truth.z||truth.quaternion.some((v,i)=>v!==this.reference!.truth.quaternion[i]))throw new Error('Modelica reference observation is unavailable for this pose');
    return structuredClone(this.reference.pose);
  }
  async loadReplay(records:RunRecord[],origin?:Pose) {
    if(this.secondary)throw new Error('Set Vehicles to 1 drone before replaying a dataset');
    if(!this.project)throw new Error('Compile before replay');
    if(origin)validatePose(origin);
    await this.compile(this.project);if(origin){this.origin=structuredClone(origin);await this.modelicaMath!.call('origin',{origin:this.origin});}
    this.replay=records;this.onStatus(`Replay ready · ${records.length} frames · sensor time preserved`);
  }
  /** Run the native SLAM on frames pulled from a live phone feed. The feed's
   * time-zero initial frame seeds the map; later frames carry their own IMU intervals. */
  async startLive(feed:LiveFeed){
    if(this.secondary)throw new Error('Set Vehicles to 1 drone before live capture');
    if(!this.project)throw new Error('Compile before live capture');
    if(!this.slam)throw new Error('Live capture needs the Native Modelica SLAM estimator (Configuration → Flight → Estimator)');
    const initial=feed.initial;if(!initial)throw new Error('No phone frame has arrived yet');
    const expected=this.project.cameraResolution??'848x480',actual=`${initial.calibration.width}x${initial.calibration.height}`;
    if(expected!==actual)throw new Error(`Set the camera to ${actual} in Configuration; the compiled SLAM expects ${expected}`);
    this.pause();
    if(this.busy)throw new Error('Wait for the current lockstep frame to finish');
    this.resetting=true;
    try{
      const estimate=await this.slam.reset(initial,this.project.loopClosureSettings,nativeParametersFor(this.project));
      this.slamProgram={...this.slamProgram!,estimate};
      this.live?.wake();this.live=feed;this.replay=[];
      this.sequence=0;this.cameraSequence=1;this.time=0;this.lastFrame=undefined;this.reference=undefined;this.records=[];
      this.pendingFrame=undefined;this.pendingRecord=undefined;this.externalStep=false;this.flow.stats.clear();
      this.frameClock.reset();this.pacingSince=undefined;this.pacedWaitMs=0;this.startWall=0;this.startTime=0;
      this.onInitialFrame(this.vehicleIndex,initial,estimate);
      this.onStatus('Live capture ready · the SLAM follows the phone as fast as it can');
    }finally{this.resetting=false;}
  }
  stopLive(){this.pause();this.releaseLive();this.replay=undefined;}
  get liveActive(){return !!this.live;}
  private releaseLive(){this.live?.wake();this.live=undefined;}
}
