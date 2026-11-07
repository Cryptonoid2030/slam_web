import {SlamNativeProgram,type SlamNativeFetch,type SlamNativeManifest} from './slam-native-program';
import type {SlamInitialFrame,SlamIntervalFrame} from './modelica-slam-frame';
import type {Estimate} from './types';
import {extractLandmarks,sparseFrame} from './landmark-source';
import {vectorNorm} from './covariance';
import {checkedLoopClosureSettings,LOOP_CLOSURE_PARAMETERS,type LoopClosureSettings} from './loop-closure-settings';
import {checkedParameterValues,type ParameterValues} from './model-parameters';

export interface SlamNativeResult {
  estimate:Estimate;
  /** Wall time of each held interval; the final call processes the captured image. */
  stepMs:number[];
  status:Record<string,number|boolean>;
}
type CameraInputs=Pick<SlamIntervalFrame,'rgbCalibration'|'depthCalibration'|'depthUnits'|'opticalToBody'|'cameraOriginBody'|'disparityNoise'|'noiseReferenceFx'|'baseline'>;
interface ImageObservation {features:number[][];current:number[][];previous:number[][];matchCount:number|undefined}
type ImuInterval=SlamIntervalFrame['intervals'][number];

/** Drives the compiler-exported D435FastSLAMInitialize and D435FastSLAMStep
 * programs: one Initialize evaluation on the time-zero image, then one Step
 * evaluation per held IMU interval with the camera image of that interval's
 * epoch. All estimation, mapping and loop closure run inside the programs. */
export class ModelicaSlamSession {
  private epoch=-1;
  private frames=0;
  private visualUpdates=0;
  /** End time of the last evaluated interval; the model checks continuity to 1e-12. */
  private clock=0;
  private predicted:{epoch:number;intervals:ImuInterval[];stepMs:number[]}={epoch:-1,intervals:[],stepMs:[]};
  private initial!:Estimate;
  private constructor(readonly manifest:SlamNativeManifest,private readonly step:SlamNativeProgram,private readonly makeInitialize:()=>Promise<SlamNativeProgram>){}

  static async create(manifest:SlamNativeManifest,fetchFile:SlamNativeFetch,initial:SlamInitialFrame,progress:(text:string)=>void=()=>{},settings?:LoopClosureSettings,parameters?:ParameterValues){
    progress('Loading native SLAM initialization program');
    const makeInitialize=await SlamNativeProgram.prepare(manifest.models.initialize,fetchFile);
    const initialize=await makeInitialize();
    progress('Loading native SLAM step program');
    const step=await SlamNativeProgram.load(manifest.models.step,fetchFile);
    const session=new ModelicaSlamSession(manifest,step,makeInitialize);
    session.initialize(initial,initialize,progress,settings,parameters);
    return session;
  }

  async reset(initial:SlamInitialFrame,settings?:LoopClosureSettings,parameters?:ParameterValues){
    await this.step.reset();
    this.initialize(initial,await this.makeInitialize(),()=>{},settings,parameters);
    return this.initialEstimate;
  }

  private initialize(initial:SlamInitialFrame,initialize:SlamNativeProgram,progress:(text:string)=>void,settings?:LoopClosureSettings,parameters?:ParameterValues){
    const session=this,step=this.step;
    this.frames=0;this.visualUpdates=0;this.predicted={epoch:-1,intervals:[],stepMs:[]};
    const tuning=checkedLoopClosureSettings(settings);
    validateSettings(initialize,tuning);validateSettings(step,tuning);
    writeSettings(initialize,tuning);writeSettings(step,tuning);
    if(parameters){writeModelParameters(initialize,parameters);writeModelParameters(step,parameters);}
    progress('Initializing native SLAM on the first camera image');
    session.writeCamera(initialize,initial);
    session.writeImage(initialize,initial);
    writeVector(initialize,'accel',initial.imu.accel);writeVector(initialize,'gyro',initial.imu.gyro);
    if(initialize.hasInput('h'))initialize.real('h')[0]=0;
    if(initialize.hasInput('intervalTime'))initialize.real('intervalTime')[0]=initial.frameTime;
    const started=performance.now();
    const status=initialize.step(initial.frameTime);
    if(status!==0)throw new Error(`Native SLAM initialization returned status ${status}`);
    initialize.transferTo(step);
    session.epoch=initial.imageEpoch;session.clock=initial.frameTime;
    session.initial=session.estimateFrom(initialize,[]);
    progress(`Native SLAM initialized in ${(performance.now()-started).toFixed(0)} ms`);
  }

  get initialEstimate():Estimate {return structuredClone(this.initial);}
  get tuningSupported(){return LOOP_CLOSURE_PARAMETERS.every(field=>this.step.hasInput(field.key));}
  configure(settings:LoopClosureSettings){const checked=checkedLoopClosureSettings(settings);validateSettings(this.step,checked);writeSettings(this.step,checked);}
  configureParameters(values:ParameterValues){writeModelParameters(this.step,values);}

  /** Overlap a held-IMU prediction with the next physics interval. Images are
   * never consumed here: only the final camera call requests observation. */
  predict(imageEpoch:number,interval:ImuInterval){
    if(imageEpoch<=this.epoch||this.predicted.intervals.length&&imageEpoch!==this.predicted.epoch)
      throw new Error('Native SLAM prediction image epoch does not continue the pending frame');
    if(!this.step.hasInput('imageRequested'))throw new Error('Native SLAM must expose the imageRequested acquisition gate');
    const end=this.writeInterval(interval);
    this.step.integer('imageEpoch')[0]=BigInt(imageEpoch);
    this.step.boolean('imageRequested')[0]=0;
    const started=performance.now(),status=this.step.step(end),elapsed=performance.now()-started;
    this.clock=end;
    if(status!==0)throw new Error(`Native SLAM prediction returned status ${status} at t=${end.toFixed(4)} s`);
    this.predicted.epoch=imageEpoch;this.predicted.intervals.push(structuredClone(interval));this.predicted.stepMs.push(elapsed);
  }

  private writeInterval(interval:ImuInterval){
    if(!(interval.dt>0&&interval.dt<=.2))throw new Error('Native SLAM accepts IMU intervals of at most 0.2 s');
    const end=this.clock+interval.dt;
    if(Math.abs(end-interval.time)>1e-6)throw new Error(`IMU interval at ${interval.time} s does not continue the native SLAM clock at ${this.clock} s`);
    writeVector(this.step,'accel',interval.imu.accel);writeVector(this.step,'gyro',interval.imu.gyro);
    this.step.real('h')[0]=interval.dt;this.step.real('intervalTime')[0]=end;
    if(this.step.hasInput('zeroVelocity'))this.step.real('zeroVelocity')[0]=interval.stationary?1:0;
    return end;
  }

  advance(frame:SlamIntervalFrame):SlamNativeResult {
    if(frame.imageEpoch<=this.epoch)throw new Error('Native SLAM requires a newer camera image epoch for each frame');
    const predicted=this.predicted;
    if(predicted.intervals.length){
      if(predicted.epoch!==frame.imageEpoch||predicted.intervals.length>=frame.intervals.length)
        throw new Error('Native SLAM camera frame does not complete its pending predictions');
      for(const [index,interval] of predicted.intervals.entries()){
        const sample=frame.intervals[index];
        if(interval.time!==sample.time||interval.dt!==sample.dt||
          interval.imu.accel.some((value,i)=>value!==sample.imu.accel[i])||
          interval.imu.gyro.some((value,i)=>value!==sample.imu.gyro[i]))
          throw new Error('Native SLAM camera frame does not match its predicted IMU prefix');
      }
    }
    this.writeCamera(this.step,frame);
    this.writeImage(this.step,frame);
    const stepMs:number[]=predicted.stepMs.slice();let imageStatus:Record<string,number|boolean>|undefined,image:ImageObservation|undefined;
    if(!this.step.hasInput('imageRequested'))throw new Error('Native SLAM must expose the imageRequested acquisition gate');
    for(const [index,interval] of frame.intervals.entries()){
      if(index<predicted.intervals.length)continue;
      // Feed the model's own continuous clock: previous end plus h, exactly.
      const end=this.writeInterval(interval);
      // This raster was captured at frameTime, after the final held IMU
      // interval. Earlier intervals predict only; they cannot consume a
      // camera measurement from their future.
      const imageOn=index===frame.intervals.length-1;
      if(imageOn&&Math.abs(end-frame.frameTime)>1e-6)throw new Error('Camera capture does not coincide with the final IMU interval');
      this.step.boolean('imageRequested')[0]=imageOn?1:0;
      const started=performance.now();
      const status=this.step.step(end);
      this.clock=end;
      stepMs.push(performance.now()-started);
      if(status!==0)throw new Error(`Native SLAM step returned status ${status} at t=${interval.time.toFixed(4)} s (image epoch ${frame.imageEpoch})`);
      if(imageOn){
        imageStatus=this.step.derived();image=this.imageFrom(this.step);
        for(const name of ['selectionValid','predictionAccepted','observationAccepted','captureAccepted']){
          const value=this.step.output(name)?.[0];if(value!==undefined)imageStatus[name]=value;
        }
      }
    }
    this.epoch=frame.imageEpoch;this.frames++;
    this.predicted={epoch:-1,intervals:[],stepMs:[]};
    if(imageStatus?.observationAccepted===1)this.visualUpdates++;
    const status={...this.step.derived(),...imageStatus};
    return {estimate:this.estimateFrom(this.step,stepMs,status,image),stepMs,status};
  }

  private writeCamera(program:SlamNativeProgram,frame:CameraInputs){
    const scalars:[string,number][]=[['depthUnits',frame.depthUnits],['disparityNoise',frame.disparityNoise],['noiseReferenceFx',frame.noiseReferenceFx],['baseline',frame.baseline]];
    for(const [name,value] of scalars)if(program.hasInput(name))program.real(name)[0]=value;
    for(const [name,values] of [['rgbCalibration',frame.rgbCalibration],['depthCalibration',frame.depthCalibration],['opticalToBody',frame.opticalToBody],['cameraOriginBody',frame.cameraOriginBody]] as const)
      if(program.hasInput(name))writeVector(program,name,values);
  }
  private writeImage(program:SlamNativeProgram,frame:SlamInitialFrame|SlamIntervalFrame){
    const {color,depth}=frame.imageLayout;
    const rgb=program.real('rgb'),z=program.real('depth');
    if(rgb.length!==color.width*color.height*3||z.length!==depth.width*depth.height)
      throw new Error(`Native SLAM program expects ${rgb.length/3} RGB and ${z.length} depth samples; the camera delivers ${color.width}×${color.height} and ${depth.width}×${depth.height}`);
    const rowBytes=color.width*3;
    if(color.strideBytes===rowBytes)rgb.set(frame.rgb.subarray(0,rgb.length));
    else for(let row=0;row<color.height;row++)rgb.set(frame.rgb.subarray(row*color.strideBytes,row*color.strideBytes+rowBytes),row*rowBytes);
    const stride=depth.strideBytes/2;
    if(stride===depth.width)z.set(frame.depth.subarray(0,z.length));
    else for(let row=0;row<depth.height;row++)z.set(frame.depth.subarray(row*stride,row*stride+depth.width),row*depth.width);
    if(frame.landmarks){
      // Landmark-input programs take the phone's corners; image programs have no such inputs.
      if(!program.hasInput('landmarkPixels'))throw new Error('This native SLAM program does not accept landmark inputs');
      program.real('landmarkPixels').set(frame.landmarks.pixels);program.real('landmarkScore').set(frame.landmarks.score);
      program.real('landmarkCount')[0]=frame.landmarks.count;
    }else if(program.hasInput('landmarkPixels')){
      // A landmark program fed a full image (synthetic world, recorded dataset): select the corners here, exactly as the phone does.
      const landmarks=sparseFrame(extractLandmarks(frame.rgb,frame.depth,color.width,color.height,{depthUnitsMeters:frame.depthUnits}),color.width,color.height);
      program.real('landmarkPixels').set(landmarks.pixels);program.real('landmarkScore').set(landmarks.score);program.real('landmarkCount')[0]=landmarks.count;
    }
    program.integer('imageEpoch')[0]=BigInt(frame.imageEpoch);
  }
  private imageFrom(program:SlamNativeProgram):ImageObservation {
    const mask=program.output('trackingEnabled');
    return {features:selectedRows(program.output('features'),program.output('featureEnabled'),3),
      current:selectedRows(program.output('trackingCurrentPixel'),mask,2),previous:selectedRows(program.output('trackingReferencePixel'),mask,2),
      matchCount:program.output('matchCount')?.[0]};
  }
  private estimateFrom(program:SlamNativeProgram,stepMs:number[],status?:Record<string,number|boolean>,image=this.imageFrom(program)):Estimate {
    const position=program.stateRun('estimator.localization.estimator.position')?.view??program.y(0,3);
    const rotation=program.stateRun('estimator.localization.estimator.rotation')?.view??program.y(6,9);
    const [x,y,z]=Array.from(position as Float64Array);
    for(const value of [x,y,z])if(!Number.isFinite(value))throw new Error('Native SLAM produced a non-finite position');
    const quaternion=program.output('nextQuaternion');
    const derived=status??program.derived();
    const {features,current,previous,matchCount}=image;
    const diagnostics:Record<string,number|string>={runtime:'Rumoca native SLAM program',model:program.layout.model,imageEpoch:this.epoch,frames:this.frames,
      stepCalls:stepMs.length,stepMsMax:stepMs.length?Math.max(...stepMs):0,stepMsTotal:stepMs.reduce((a,b)=>a+b,0)};
    for(const [name,value] of Object.entries(derived))diagnostics[name]=typeof value==='boolean'?(value?1:0):value;
    const candidateCount=derived.loopCandidateCount??program.output('loopCandidateCount')?.[0];
    if(typeof candidateCount==='number'&&Number.isSafeInteger(candidateCount)&&candidateCount>=0)
      diagnostics.loopCandidates=candidateCount;
    if(matchCount!==undefined)diagnostics.matchCount=matchCount;
    const accepted=derived.observationAccepted===true||derived.observationAccepted===1;
    const {points,confidence:mapConfidence}=this.mapPoints(program);
    const confidence=matchCount!==undefined&&features.length?Math.min(1,matchCount/features.length):mapConfidence;
    diagnostics.mapPoints=points.length;
    const poseGraph=this.poseGraph(program,derived);
    diagnostics.keyframes=poseGraph.keyframes.length;
    diagnostics.loopEdges=poseGraph.edges.filter(edge=>edge.kind==='loop').length;
    diagnostics.loops=diagnostics.loopEdges;
    diagnostics.visualUpdates=this.visualUpdates;
    diagnostics.odometryEdges=poseGraph.edges.filter(edge=>edge.kind==='odometry').length;
    const words=program.stateRun('vocabulary.count')?.view,ready=program.stateRun('vocabulary.ready')?.view;
    if(words)diagnostics.loopWords=Number(words[0]);
    if(ready)diagnostics.loopVocabularyReady=Number(ready[0]);
    const covariance=program.stateRun('estimator.localization.estimator.covariance')?.view;
    const block=(start:number)=>Array.from({length:9},(_,index)=>Number(covariance![(start+Math.floor(index/3))*15+start+index%3]));
    for(const [name,state] of [['accelBiasNorm','accelBias'],['gyroBiasNorm','gyroBias']] as const){
      const norm=vectorNorm(program.stateRun(`estimator.localization.estimator.${state}`)?.view);
      if(norm!==undefined)diagnostics[name]=norm;
    }
    const uncertainty=covariance?.length===225?{positionCovariance:block(0),attitudeCovariance:block(6)}:undefined;
    const scalar=(name:string)=>derived[name]??program.output(name)?.[0];
    let loopClosure:Estimate['loopClosure'];
    if(scalar('loopAttempted')!==undefined&&typeof candidateCount==='number'&&Number.isSafeInteger(candidateCount)&&candidateCount>=0&&candidateCount<=4){
      const value=(name:string,rank:number)=>program.output(name)?.[rank]??derived[`${name}[${rank+1}]`];
      const candidates:NonNullable<Estimate['loopClosure']>['candidates']=[];
      for(let rank=0;rank<candidateCount;rank++){
        const referenceId=Number(value('loopCandidateReferenceId',rank)),currentId=Number(value('loopCandidateCurrentId',rank));
        const score=Number(value('loopCandidateScore',rank)),reason=Number(value('loopCandidateReason',rank));
        if(!Number.isSafeInteger(referenceId)||referenceId<1||!Number.isSafeInteger(currentId)||currentId<1||!Number.isFinite(score)||score<0||score>1||!Number.isSafeInteger(reason))continue;
        const fitAvailable=[0,8,9,11].includes(reason),matchedCount=Number(value('loopCandidateMatchedCount',rank));
        const inlierCount=Number(value('loopCandidateInliers',rank)),rms=Number(value('loopCandidateRms',rank));
        candidates.push({referenceId,currentId,score,reason,verified:Number(value('loopCandidateVerified',rank))===1,
          ...((reason>=6||reason===0)&&Number.isSafeInteger(matchedCount)&&matchedCount>=0?{matchedCount}:{}),
          ...(fitAvailable&&Number.isSafeInteger(inlierCount)&&inlierCount>=0&&Number.isFinite(rms)&&rms>=0?{inlierCount,rms}:{})});
      }
      loopClosure={attempted:!!scalar('loopAttempted'),retrievalReason:Number(scalar('loopRetrievalReason')),graphReason:Number(scalar('loopGraphReason')),candidateCount,candidates};
    }
    return {x,y,z,quaternion:quaternion?Array.from(quaternion):quaternionFromRotation(rotation as Float64Array),
      confidence,points,features,diagnostics,poseGraph,uncertainty,loopClosure,
      tracking:{matches:current.map((value,index)=>({current:value,previous:previous[index]})),accepted,
        reason:accepted?'visual observation accepted':derived.captureAccepted===1?'visual reference captured':`visual observation refused; publication reason ${diagnostics.publicationReason??'n/a'}`}};
  }
  private poseGraph(program:SlamNativeProgram,diagnostics:Record<string,number|boolean>={}):NonNullable<Estimate['poseGraph']> {
    const run=(name:string)=>program.stateRun(name)?.view;
    const enabled=run('estimator.poses.enabled'),ids=run('estimator.poses.ids');
    const positions=run('estimator.poses.positions'),rotations=run('estimator.poses.rotations');
    const catalog='estimator.localization.catalog.';
    const occupied=run(catalog+'occupied'),catalogIds=run(catalog+'ids'),times=run(catalog+'imageTimes'),epochs=run(catalog+'epochs');
    const images=new Map<number,{time:number;imageEpoch:number}>();
    if(occupied&&catalogIds&&times&&epochs)for(let slot=0;slot<occupied.length;slot++)if(Number(occupied[slot])){
      const id=Number(catalogIds[slot]),time=Number(times[slot]),imageEpoch=Number(epochs[slot]);
      if(Number.isSafeInteger(id)&&id>0&&Number.isFinite(time)&&time>=0&&Number.isSafeInteger(imageEpoch)&&imageEpoch>=0)
        images.set(id,{time,imageEpoch});
    }
    const keyframes:NonNullable<Estimate['poseGraph']>['keyframes']=[],edges:NonNullable<Estimate['poseGraph']>['edges']=[];
    const index=new Map<number,number>();
    if(!enabled||!ids||!positions||!rotations)return {keyframes,edges};
    for(let slot=0;slot<enabled.length;slot++)if(Number(enabled[slot])){
      index.set(Number(ids[slot]),keyframes.length);
      keyframes.push({id:Number(ids[slot]),...images.get(Number(ids[slot])),position:Array.from(positions.slice(slot*3,slot*3+3),Number),
        quaternion:quaternionFromRotation(Array.from(rotations.slice(slot*9,slot*9+9),Number))});
    }
    const prefix='estimator.localization.graph.edges.';
    const active=run(prefix+'enabled'),kinds=run(prefix+'kind'),references=run(prefix+'referenceId'),currents=run(prefix+'currentId');
    const edgeIds=run(prefix+'id');
    const scores=program.output('loopCandidateScore')??program.output('loopScore');
    const referenceIds=program.output('loopCandidateReferenceId')??program.output('loopReferenceId');
    const currentIds=program.output('loopCandidateCurrentId')??program.output('loopCurrentId');
    if(active&&kinds&&references&&currents)for(let slot=0;slot<active.length;slot++)if(Number(active[slot])){
      const from=index.get(Number(references[slot])),to=index.get(Number(currents[slot])),kind=Number(kinds[slot]);
      if(from!==undefined&&to!==undefined&&(kind===1||kind===2)){
        let similarity:number|undefined;
        // Four retained verified pairs; exporters may use y runs or typed cells.
        if(kind===2)for(let rank=0;rank<(scores?.length??4);rank++){
          const reference=referenceIds?.[rank]??diagnostics[`loopCandidateReferenceId[${rank+1}]`]??diagnostics[`loopReferenceId[${rank+1}]`];
          const current=currentIds?.[rank]??diagnostics[`loopCandidateCurrentId[${rank+1}]`]??diagnostics[`loopCurrentId[${rank+1}]`];
          const score=scores?.[rank]??diagnostics[`loopCandidateScore[${rank+1}]`]??diagnostics[`loopScore[${rank+1}]`];
          if(reference===Number(references[slot])&&current===Number(currents[slot])&&
            typeof score==='number'&&Number.isFinite(score)&&score>=0&&score<=1)similarity=score;
        }
        edges.push({from,to,kind:kind===2?'loop':'odometry',...(edgeIds?{id:Number(edgeIds[slot])}:{}),...(similarity===undefined?{}:{similarity})});
      }
    }
    return {keyframes,edges};
  }
  /** Occupied landmark positions of the carried map (estimator.localization.map),
   * with the mean landmark confidence scaled by the model's maximum. */
  private mapPoints(program:SlamNativeProgram):{points:number[][];confidence:number} {
    const point=program.stateRun('estimator.localization.map.point'),occupied=program.stateRun('estimator.localization.map.occupied');
    const landmarkConfidence=program.stateRun('estimator.localization.map.confidence');
    if(!point||!occupied||point.lane!=='f64')return {points:[],confidence:0};
    const values=point.view as Float64Array,mask=occupied.view,count=mask.length;
    if(values.length!==count*3)throw new Error('Native SLAM map runs disagree on the landmark count');
    const points:number[][]=[];let total=0;
    for(let i=0;i<count;i++){
      if(!Number(mask[i]))continue;
      const row=[values[3*i],values[3*i+1],values[3*i+2]];
      if(!row.every(Number.isFinite))throw new Error('Native SLAM map holds a non-finite landmark');
      points.push(row);total+=landmarkConfidence?Number((landmarkConfidence.view as Float64Array)[i]):0;
    }
    return {points,confidence:points.length&&landmarkConfidence?Math.min(1,total/points.length/8):0};
  }
}

function writeModelParameters(program:SlamNativeProgram,overrides:ParameterValues){
  const fields=program.layout.parameters;
  if(!fields){if(!Object.keys(overrides).length)return;throw Error('This native delivery has no Modelica parameter metadata');}
  const values={...Object.fromEntries(fields.map(field=>[field.name,field.defaultValue])),...overrides} as ParameterValues;
  checkedParameterValues(fields,values);
  const writes=fields.map(field=>{
    const slot=field.slot??field.name,index=field.index??0;
    const view=field.type==='Boolean'?program.boolean(slot):field.type==='Integer'?program.integer(slot):program.real(slot);
    if(!Number.isSafeInteger(index)||index<0||index>=view.length)throw Error(`Invalid native parameter slot: ${field.name}`);
    return {view,index,value:values[field.name],integer:field.type==='Integer'};
  });
  for(const {view,index,value,integer}of writes){if(integer)(view as BigInt64Array)[index]=BigInt(value);else (view as Float64Array|Uint8Array)[index]=Number(value);}
}

function validateSettings(program:SlamNativeProgram,settings:LoopClosureSettings){
  for(const field of LOOP_CLOSURE_PARAMETERS){
    if(!program.hasInput(field.key)){
      if(settings[field.key]!==field.value)throw Error(`This SLAM build does not expose ${field.label}. Load the current default models to tune it.`);
    }else if(program.layout.inputs[field.key].lane!==(field.key==='loopMinimumInliers'?'i64':typeof field.value==='boolean'?'u8':'f64'))throw Error(`Unexpected native parameter type: ${field.key}`);
  }
}
function writeSettings(program:SlamNativeProgram,settings:LoopClosureSettings){
  for(const field of LOOP_CLOSURE_PARAMETERS)if(program.hasInput(field.key)){
    if(field.key==='loopMinimumInliers')program.integer(field.key)[0]=BigInt(settings[field.key]);
    else if(typeof field.value==='boolean')program.boolean(field.key)[0]=settings[field.key]===true?1:0;
    else program.real(field.key)[0]=settings[field.key] as number;
  }
}

function writeVector(program:SlamNativeProgram,name:string,values:ArrayLike<number>){
  const view=program.real(name);
  if(view.length!==values.length)throw new Error(`Native SLAM input ${name} takes ${view.length} values`);
  for(let i=0;i<values.length;i++){if(!Number.isFinite(values[i]))throw new Error(`Native SLAM input ${name} must be finite`);view[i]=values[i];}
}
function selectedRows(data:Float64Array|undefined,mask:Float64Array|undefined,width:number):number[][] {
  if(!data||!mask)return [];
  const rows:number[][]=[];
  for(let row=0;row<mask.length&&(row+1)*width<=data.length;row++)if(mask[row]!==0)rows.push(Array.from(data.subarray(row*width,(row+1)*width)));
  return rows;
}
/** Unit quaternion [w,x,y,z] of a row-major rotation matrix. */
export function quaternionFromRotation(r:ArrayLike<number>):number[] {
  const [m00,m01,m02,m10,m11,m12,m20,m21,m22]=Array.from(r);
  const trace=m00+m11+m22;let w:number,x:number,y:number,z:number;
  if(trace>0){const s=Math.sqrt(trace+1)*2;w=s/4;x=(m21-m12)/s;y=(m02-m20)/s;z=(m10-m01)/s;}
  else if(m00>m11&&m00>m22){const s=Math.sqrt(1+m00-m11-m22)*2;w=(m21-m12)/s;x=s/4;y=(m01+m10)/s;z=(m02+m20)/s;}
  else if(m11>m22){const s=Math.sqrt(1+m11-m00-m22)*2;w=(m02-m20)/s;x=(m01+m10)/s;y=s/4;z=(m12+m21)/s;}
  else{const s=Math.sqrt(1+m22-m00-m11)*2;w=(m10-m01)/s;x=(m02+m20)/s;y=(m12+m21)/s;z=s/4;}
  const norm=Math.hypot(w,x,y,z)||1;
  return [w/norm,x/norm,y/norm,z/norm];
}
