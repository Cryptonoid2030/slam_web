import {describe,expect,it} from 'vitest';
import {DEFAULT_LIVE_CONFIG,ImuBuffer,LiveFrameBuilder,phoneImuToFlu,resolutionMode,type LiveConfig} from '../src/live-source';
import {phoneConfig} from '../src/live-client';
import {emptyLandmarks,encodeLandmarkPacket,extractLandmarks,parseLandmarkPacket,selectionThreshold,type LandmarkHeader} from '../src/landmark-source';

const config:LiveConfig={...DEFAULT_LIVE_CONFIG,resolution:'424x240',mode:'landmarks'};
const mode=resolutionMode(config.resolution);
function scene(width:number,height:number){
  const rgb=new Uint8Array(width*height*3),depth=new Uint16Array(width*height).fill(1800);
  let seed=7;const rand=()=>{seed=(seed*1664525+1013904223)>>>0;return seed/2**32;};
  const blobs=Array.from({length:60},()=>({x:8+rand()*(width-16),y:8+rand()*(height-16),r:2+rand()*4,v:rand()<.5?25:230}));
  for(let y=0;y<height;y++)for(let x=0;x<width;x++){
    let v=120+15*Math.sin(x/13);for(const b of blobs)if(Math.hypot(x-b.x,y-b.y)<b.r)v=b.v;
    rgb.set([v,v,v],3*(y*width+x));
  }
  return {rgb,depth};
}
const imu=()=>{
  const buffer=new ImuBuffer();
  // CoreMotion at rest, face up reads z = -1 g; 100 Hz
  for(let i=0;i<400;i++)buffer.push(phoneImuToFlu([1000+i*.01,0,0,-1,0,0,0]));
  return buffer;
};
const header=(t:number,count:number,target=mode):LandmarkHeader=>({type:'landmarks',t,count,color:{width:1920,height:1440,fx:1450,fy:1450,cx:960,cy:720},target:{width:target.width,height:target.height}});

describe('landmark live frames',()=>{
  const {rgb,depth}=scene(mode.width,mode.height);
  const set=extractLandmarks(rgb,depth,mode.width,mode.height,{depthUnitsMeters:config.depthUnitsMeters});
  it('builds an initial frame and interval frames that carry landmarks and a sparse canvas',()=>{
    const builder=new LiveFrameBuilder(imu(),config);
    const first=builder.buildLandmarks(header(1000.5,set.count),set);
    expect(first.dt).toBe(0);expect(first.landmarks?.count).toBe(set.count);
    expect(first.rgb.length).toBe(mode.width*mode.height*3);
    const second=builder.buildLandmarks(header(1001.2,set.count),set);
    expect(second.dt).toBeCloseTo(.7,6);expect(second.imuIntervals!.length).toBeGreaterThan(1);
    expect(second.landmarks!.pixels.length).toBe(700);
    let painted=0;for(const z of second.depth as Uint16Array)if(z)painted++;
    expect(painted).toBe(set.count);
  });
  it('refuses landmarks selected for another resolution',()=>{
    const builder=new LiveFrameBuilder(imu(),config);
    expect(()=>builder.buildLandmarks(header(1000.5,set.count,{width:640,height:480} as typeof mode),set)).toThrow(/640x480/);
  });
  it('survives the wire format on the way in',()=>{
    const parsed=parseLandmarkPacket(encodeLandmarkPacket(header(1000.5,set.count),set));
    const builder=new LiveFrameBuilder(imu(),config);
    expect(builder.buildLandmarks(parsed.header,parsed.landmarks).landmarks!.count).toBe(set.count);
  });
  it('accepts an empty landmark set',()=>{
    const builder=new LiveFrameBuilder(imu(),config),none=emptyLandmarks(0);
    expect(builder.buildLandmarks(header(1000.5,0),none).landmarks!.count).toBe(0);
  });
});

describe('phone config',()=>{
  it('tells the phone the grid, units and the width-dependent FAST threshold',()=>{
    const message=phoneConfig(config);
    expect(message).toMatchObject({type:'config',mode:'landmarks',resolution:'424x240',width:424,height:240,landmarkCapacity:350,depthUnitsMeters:.001});
    expect(message.fastThreshold).toBe(selectionThreshold(424));
    expect(phoneConfig({...config,mode:'raw'}).mode).toBe('raw');
  });
});

describe('RateMeter',()=>{
  it('reports events per second and decays when they stop',async()=>{
    const {RateMeter}=await import('../src/live-client');
    const meter=new RateMeter();
    for(let i=0;i<=4;i++)meter.mark(i*500);            // 2 per second
    expect(meter.perSecond(2000)).toBeCloseTo(2,5);
    expect(meter.perSecond(4000)).toBeLessThan(1.5);   // stall lowers it
    expect(meter.perSecond(20000)).toBe(0);            // window empties
    meter.reset();expect(meter.perSecond(0)).toBe(0);
  });
});

describe('frame interval',()=>{
  it('keeps the original window at the default interval and scales it for faster capture',async()=>{
    const {frameSpacing,FrameSelector}=await import('../src/live-source');
    expect(frameSpacing(.7).minGap).toBeCloseTo(.595,6);expect(frameSpacing(.7).maxGap).toBeCloseTo(.945,6);
    const fast=frameSpacing(.3);expect(fast.minGap).toBeCloseTo(.255,6);expect(fast.maxGap).toBeCloseTo(.405,6);
    expect(frameSpacing(.05).minGap).toBeCloseTo(.17,6);expect(frameSpacing(5).maxGap).toBeLessThan(.95+1e-9);
    // A phone sending every 0.3 s with jitter keeps nearly every frame at the fast setting, but would drop most at the default.
    const run=(spacing:{minGap:number;maxGap:number})=>{
      const selector=new FrameSelector<{t:number}>(spacing.minGap,spacing.maxGap);let kept=0;
      for(let i=0;i<40;i++){if(selector.offer({t:i*.3+(i%3)*.01}))kept++;}return kept;
    };
    expect(run(frameSpacing(.3))).toBeGreaterThan(30);
    expect(run(frameSpacing(.7))).toBeLessThan(20);
  });
});

describe('landmark limit',()=>{
  const {rgb,depth}=scene(mode.width,mode.height);
  const set=extractLandmarks(rgb,depth,mode.width,mode.height,{depthUnitsMeters:config.depthUnitsMeters});
  it('keeps only the strongest N landmarks, strongest first, and paints only those',()=>{
    expect(set.count).toBeGreaterThan(40);
    const builder=new LiveFrameBuilder(imu(),config);
    expect(builder.landmarkLimit).toBe(350);builder.landmarkLimit=25;
    const frame=builder.buildLandmarks(header(1000.5,set.count),set);
    expect(frame.landmarks!.count).toBe(25);
    for(let i=0;i<25;i++)expect(frame.landmarks!.score[i]).toBe(set.score[i]);
    let painted=0;for(const z of frame.depth as Uint16Array)if(z)painted++;
    expect(painted).toBe(25);
    builder.landmarkLimit=1000; // above what the set holds: nothing is dropped
    expect(builder.buildLandmarks(header(1001.2,set.count),set).landmarks!.count).toBe(set.count);
  });
  it('clamps what it asks the phone for',()=>{
    expect(phoneConfig(config,40).landmarkCapacity).toBe(40);
    expect(phoneConfig(config,9999).landmarkCapacity).toBe(350);
    expect(phoneConfig(config,0).landmarkCapacity).toBe(1);
  });
});

describe('rest detection',()=>{
  const sample=(t:number,noise:number,gyro=0)=>({t,accel:[noise*Math.sin(t*97),noise*Math.cos(t*61),9.80665+noise*Math.sin(t*53)] as [number,number,number],gyro:[gyro,0,0] as [number,number,number]});
  it('calls a motionless device at rest and a held or moving one not',async()=>{
    const {isAtRest}=await import('../src/live-source');
    const still=Array.from({length:40},(_,i)=>sample(i*.01,.01));
    expect(isAtRest(still)).toBe(true);
    expect(isAtRest(Array.from({length:40},(_,i)=>sample(i*.01,.2)))).toBe(false);          // hand tremor level
    expect(isAtRest(Array.from({length:40},(_,i)=>sample(i*.01,.01,.1)))).toBe(false);      // rotating
    expect(isAtRest(still.slice(0,4))).toBe(false);                                         // too few samples
    expect(isAtRest(still.map(s=>({...s,accel:[s.accel[0]+3,s.accel[1],s.accel[2]] as [number,number,number]})))).toBe(false); // not gravity
  });
  it('marks every interval of a resting frame and none when detection is off',()=>{
    const buffer=new ImuBuffer();
    for(let i=0;i<=100;i++)buffer.push({t:i*.01,accel:[0,0,9.80665],gyro:[0,0,0]});
    expect(buffer.intervals(0,.7,0,.2,true).every(x=>x.stationary===true)).toBe(true);
    expect(buffer.intervals(0,.7,0,.2).some(x=>x.stationary)).toBe(false);
  });
});
