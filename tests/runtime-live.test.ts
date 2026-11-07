import {afterEach,expect,it,vi} from 'vitest';
import {Runtime} from '../src/runtime';
import {FrameClock} from '../src/frame-clock';
import {LiveFeed,recordFor} from '../src/live-source';
import type {InitialSensorFrame} from '../src/types';

vi.mock('../src/model-parameters',async importOriginal=>({...await importOriginal<typeof import('../src/model-parameters')>(),nativeParametersFor:()=>({})}));
afterEach(()=>{vi.restoreAllMocks();});

const initialFrame=(width=640,height=480)=>({time:0,dt:0,sequence:0,imuIntervals:[],imu:{accel:[0,0,9.8],gyro:[0,0,0]},calibration:{width,height}}) as unknown as InitialSensorFrame;
function runtime(over:Record<string,unknown>={}){
  const slam={reset:vi.fn(async()=>({x:0,y:0,z:0,quaternion:[1,0,0,0],confidence:1,points:[]}))};
  return Object.assign(Object.create(Runtime.prototype),{
    frameClock:new FrameClock(),playbackEpoch:0,running:false,stepping:false,resetting:false,fleetStepping:false,time:3,sequence:7,cameraSequence:7,dt:.01,speed:Infinity,
    flow:{stats:new Map([['x',1]])},slam,slamProgram:{models:{step:{model:'m'}}},project:{cameraResolution:'640x480',loopClosureSettings:undefined},
    onStatus:vi.fn(),onInitialFrame:vi.fn(),onError:vi.fn(),...over}) as any;   // tests poke private state
}

it('refuses to start without a project, the native estimator, a first frame, or a matching camera mode',async()=>{
  const feed=new LiveFeed();
  await expect(runtime({project:undefined}).startLive(feed)).rejects.toThrow(/Compile/);
  await expect(runtime({slam:undefined}).startLive(feed)).rejects.toThrow(/Native Modelica SLAM/);
  await expect(runtime().startLive(feed)).rejects.toThrow(/first|No phone frame/);
  feed.initial=initialFrame(424,240);
  await expect(runtime().startLive(feed)).rejects.toThrow(/424x240.*640x480/);
});

it('reseeds the SLAM from the phone frame and resets the run clock',async()=>{
  const r=runtime(),feed=new LiveFeed();feed.initial=initialFrame();
  await r.startLive(feed);
  expect(r.slam.reset).toHaveBeenCalledWith(feed.initial,undefined,{});
  expect(r.onInitialFrame).toHaveBeenCalledOnce();
  expect(r.liveActive).toBe(true);expect(r.sequence).toBe(0);expect(r.time).toBe(0);expect(r.replay).toEqual([]);expect(r.resetting).toBe(false);
  r.stopLive();expect(r.liveActive).toBe(false);expect(r.replay).toBeUndefined();
});

it('lets pause release a step that is waiting for the next phone frame',async()=>{
  const r=runtime(),feed=new LiveFeed();feed.initial=initialFrame();await r.startLive(feed);
  const waiting=(r as any).stepVehicle();
  await Promise.resolve();
  r.pause();
  await expect(waiting).resolves.toBeUndefined();
  expect(r.stepping).toBe(false);expect(r.sequence).toBe(0);
});

it('does not step when the feed is released by a reset',async()=>{
  const r=runtime(),feed=new LiveFeed();feed.initial=initialFrame();await r.startLive(feed);
  const waiting=(r as any).stepVehicle();
  await Promise.resolve();
  (r as any).releaseLive();
  await expect(waiting).resolves.toBeUndefined();
  expect(r.sequence).toBe(0);void recordFor;
});
