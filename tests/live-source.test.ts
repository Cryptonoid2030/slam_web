import {describe,expect,it} from 'vitest';
import {
  DEFAULT_LIVE_CONFIG,FrameSelector,ImuBuffer,LiveFeed,LiveFrameBuilder,LiveGapError,PHONE_MAGIC,STANDARD_GRAVITY,
  cropFor,depthToZ16,liveCalibration,parsePhoneCamera,phoneImuToFlu,recordFor,
  type PhoneCamera,type PhoneCameraHeader,
} from '../src/live-source';
import {imuIntervals} from '../src/imu-intervals';

function header(t:number,over:Partial<PhoneCameraHeader>={}):PhoneCameraHeader {
  return {type:'camera',t,color:{width:1920,height:1440,fx:1450,fy:1450,cx:960,cy:720},depth:{width:8,height:6},
    jpegBytes:4,depthBytes:8*6*4,confidenceBytes:8*6,...over};
}
function message(h:PhoneCameraHeader,depth:Float32Array,confidence:Uint8Array){
  const json=new TextEncoder().encode(JSON.stringify(h)),jpeg=new Uint8Array([1,2,3,4]);
  const bytes=new Uint8Array(8+json.length+jpeg.length+depth.byteLength+confidence.length),view=new DataView(bytes.buffer);
  view.setUint32(0,PHONE_MAGIC,true);view.setUint32(4,json.length,true);
  bytes.set(json,8);bytes.set(jpeg,8+json.length);
  bytes.set(new Uint8Array(depth.buffer),8+json.length+jpeg.length);bytes.set(confidence,8+json.length+jpeg.length+depth.byteLength);
  return bytes;
}
function camera(t:number,depthValue=2,confidence=2):PhoneCamera {
  const depth=new Float32Array(48).fill(depthValue);
  return parsePhoneCamera(message(header(t),depth,new Uint8Array(48).fill(confidence)));
}
function steadyImu(from:number,to:number,hz=100){
  const buffer=new ImuBuffer();
  for(let t=from;t<=to+1e-9;t+=1/hz)buffer.push(phoneImuToFlu([t,-1,0,0,.1,0,0]));
  return buffer;
}

describe('phone camera message',()=>{
  it('round-trips depth, confidence and JPEG bytes',()=>{
    const depth=Float32Array.from({length:48},(_,i)=>i/10),confidence=Uint8Array.from({length:48},(_,i)=>i%3);
    const parsed=parsePhoneCamera(message(header(12.5),depth,confidence));
    expect(parsed.header.t).toBe(12.5);expect(Array.from(parsed.jpeg)).toEqual([1,2,3,4]);
    expect(Array.from(parsed.depth)).toEqual(Array.from(depth));expect(Array.from(parsed.confidence)).toEqual(Array.from(confidence));
  });
  it('rejects truncated, mis-sized and wrong-magic payloads',()=>{
    const good=message(header(1),new Float32Array(48),new Uint8Array(48));
    expect(()=>parsePhoneCamera(good.subarray(0,good.length-1))).toThrow(/length mismatch/);
    expect(()=>parsePhoneCamera(message(header(1,{depthBytes:40}),new Float32Array(10),new Uint8Array(48)))).toThrow(/mismatch/);
    const bad=good.slice();bad[0]^=0xff;expect(()=>parsePhoneCamera(bad)).toThrow(/Unsupported/);
  });
});

describe('IMU conversion',()=>{
  it('reads +g on body up for a landscape-right phone held upright',()=>{
    const s=phoneImuToFlu([0,-1,0,0,0,0,0]);
    expect(s.accel[0]).toBeCloseTo(0);expect(s.accel[1]).toBeCloseTo(0);expect(s.accel[2]).toBeCloseTo(STANDARD_GRAVITY);
  });
  it('maps a phone lying face up (camera down) to -g along body forward',()=>{
    const s=phoneImuToFlu([0,0,0,-1,0,0,0]);
    expect(s.accel[0]).toBeCloseTo(-STANDARD_GRAVITY);expect(s.accel[1]).toBeCloseTo(0);expect(s.accel[2]).toBeCloseTo(0);
  });
  it('maps rotation rate with the same proper rotation (det = +1)',()=>{
    expect(phoneImuToFlu([0,0,0,0,1,0,0]).gyro).toEqual([-0,0,1]);   // device x -> body up
    expect(phoneImuToFlu([0,0,0,0,0,1,0]).gyro).toEqual([-0,1,0]);   // device y -> body left
    expect(phoneImuToFlu([0,0,0,0,0,0,1]).gyro).toEqual([-1,0,0]);   // device z -> -forward
  });
  it('rejects malformed rows',()=>{expect(()=>phoneImuToFlu([0,1,2])).toThrow();expect(()=>phoneImuToFlu([0,NaN,0,0,0,0,0])).toThrow();});
});

describe('ImuBuffer intervals',()=>{
  it('splits a 0.95 s span into five held intervals that satisfy the lab contract',()=>{
    const buffer=steadyImu(0,2),intervals=buffer.intervals(.5,1.45,.5,.2);
    expect(intervals).toHaveLength(5);
    expect(intervals.reduce((a,i)=>a+i.dt,0)).toBeCloseTo(.95,9);
    expect(intervals.at(-1)!.time).toBeCloseTo(.95,9);
    for(const i of intervals){expect(i.dt).toBeLessThanOrEqual(.2);expect(i.imu.accel[2]).toBeCloseTo(STANDARD_GRAVITY);expect(i.imu.gyro[0]).toBeCloseTo(0);expect(i.imu.gyro[2]).toBeCloseTo(.1);}
    expect(()=>imuIntervals({time:.95,dt:.95,imu:intervals[0].imu,imuIntervals:intervals})).not.toThrow();
  });
  it('averages samples inside an interval and holds the last value across a data gap',()=>{
    const buffer=new ImuBuffer();
    buffer.push(phoneImuToFlu([.01,0,0,-1,0,0,0]));buffer.push(phoneImuToFlu([.02,0,0,-3,0,0,0]));
    const [first,second]=buffer.intervals(0,.4,0,.2);
    expect(first.imu.accel[0]).toBeCloseTo(-2*STANDARD_GRAVITY);       // mean of -1 g and -3 g on device z
    expect(second.imu.accel[0]).toBeCloseTo(-3*STANDARD_GRAVITY);      // no samples: hold the latest
  });
  it('drops duplicate and reordered samples and prunes old history',()=>{
    const buffer=new ImuBuffer();
    expect(buffer.push(phoneImuToFlu([1,0,0,0,0,0,0]))).toBe(true);
    expect(buffer.push(phoneImuToFlu([1,0,0,0,0,0,0]))).toBe(false);expect(buffer.push(phoneImuToFlu([.5,0,0,0,0,0,0]))).toBe(false);
    for(let i=2;i<10;i++)buffer.push(phoneImuToFlu([i,0,0,0,0,0,0]));
    buffer.prune(7.5);expect(buffer.heldAt(7.5).t).toBe(7);expect(buffer.size).toBe(3);
  });
  it('refuses spans that need more than 36 intervals',()=>{expect(()=>steadyImu(0,10).intervals(0,8,0,.2)).toThrow(/36/);});
});

describe('FrameSelector',()=>{
  it('keeps frames 0.6–0.95 s apart from a jittery 4 Hz stream',()=>{
    const selector=new FrameSelector<{t:number}>(),kept:number[]=[];let seed=7;
    const random=()=>{seed=(seed*1664525+1013904223)>>>0;return seed/4294967296;};
    for(let i=0,t=0;i<200;i++,t+=.25+(random()-.5)*.06){const item=selector.offer({t});if(item)kept.push(item.t);}
    expect(kept.length).toBeGreaterThan(40);
    for(let i=1;i<kept.length;i++){expect(kept[i]-kept[i-1]).toBeGreaterThanOrEqual(.6);expect(kept[i]-kept[i-1]).toBeLessThanOrEqual(.95);}
  });
  it('reports an unrecoverable gap and accepts one that is still under the 1 s limit',()=>{
    const bad=new FrameSelector<{t:number}>();bad.offer({t:0});expect(()=>bad.offer({t:1.5})).toThrow(LiveGapError);
    const ok=new FrameSelector<{t:number}>();ok.offer({t:0});expect(ok.offer({t:.97})?.t).toBe(.97);
  });
});

describe('calibration and depth resampling',()=>{
  it('maps the 4:3 phone image onto 640x480 without cropping',()=>{
    const {calibration,crop}=liveCalibration(header(0));
    expect(crop).toEqual({x:0,y:0,w:1920,h:1440});
    expect(calibration.fx).toBeCloseTo(1450/3);expect(calibration.cx).toBeCloseTo(320);expect(calibration.cy).toBeCloseTo(240);
    expect(calibration.rgbFx).toBe(calibration.fx);expect(calibration.depthEncoding).toBe('axial-z16-le');
  });
  it('center-crops 4:3 to 16:9 and shifts the principal point',()=>{
    const crop=cropFor(1920,1440,424,240);expect(crop.w).toBe(1920);expect(crop.h).toBeCloseTo(1920*240/424);expect(crop.y).toBeCloseTo((1440-crop.h)/2);
    const {calibration}=liveCalibration(header(0),{...DEFAULT_LIVE_CONFIG,resolution:'424x240'});
    expect(calibration.width).toBe(424);expect(calibration.cy).toBeCloseTo((720-crop.y)*424/1920);
  });
  it('writes Z16 millimeters and zeroes low confidence and out-of-range returns',()=>{
    const depth=new Float32Array(48).fill(2),confidence=new Uint8Array(48).fill(2);
    depth[0]=.05;depth[1]=9;depth[2]=NaN;confidence[3]=0;confidence[4]=1;
    const cam=parsePhoneCamera(message(header(0),depth,confidence)),{crop}=liveCalibration(cam.header);
    const z=depthToZ16(cam,crop,640,480);
    // 640x480 over an 8x6 grid: each source cell is 80x80 pixels
    expect(z[40*640+40]).toBe(0);expect(z[40*640+120]).toBe(0);expect(z[40*640+200]).toBe(0);expect(z[40*640+280]).toBe(0);
    expect(z[40*640+360]).toBe(2000);expect(z[300*640+300]).toBe(2000);
    expect(depthToZ16(cam,crop,640,480,{...DEFAULT_LIVE_CONFIG,minConfidence:2})[40*640+360]).toBe(0);
  });
});

describe('LiveFrameBuilder',()=>{
  const rgb=()=>new Uint8Array(640*480*3).fill(128);
  it('starts with a time-zero initial frame, then emits contiguous interval frames',()=>{
    const imu=steadyImu(0,3),builder=new LiveFrameBuilder(imu);
    const initial=builder.build(camera(1.0),rgb());
    expect(initial.time).toBe(0);expect(initial.dt).toBe(0);expect(initial.sequence).toBe(0);expect((initial as any).imuIntervals).toEqual([]);
    expect(initial.imu.accel[2]).toBeCloseTo(STANDARD_GRAVITY);
    const second=builder.build(camera(1.7),rgb()) as any,third=builder.build(camera(2.6),rgb()) as any;
    expect(second.sequence).toBe(1);expect(second.time).toBeCloseTo(.7);expect(second.dt).toBeCloseTo(.7);expect(second.imuIntervals).toHaveLength(4);
    expect(third.time).toBeCloseTo(1.6);expect(third.dt).toBeCloseTo(.9);expect(third.imuIntervals.at(-1).time).toBeCloseTo(1.6);expect(third.imuIntervals).toHaveLength(5);
    expect(third.imageLayout.depth.unitsMeters).toBe(.001);expect(third.depth).toBeInstanceOf(Uint16Array);
  });
  it('rejects non-increasing camera time and a mismatched color buffer',()=>{
    const builder=new LiveFrameBuilder(steadyImu(0,3));builder.build(camera(1),rgb());
    expect(()=>builder.build(camera(.9),rgb())).toThrow(LiveGapError);
    expect(()=>builder.build(camera(1.5),new Uint8Array(10))).toThrow(/live resolution/);
  });
  it('wraps frames as replay records with no ground truth',()=>{
    const builder=new LiveFrameBuilder(steadyImu(0,3));builder.build(camera(1),rgb());
    const record=recordFor(builder.build(camera(1.7),rgb()) as any);
    expect(record.truth).toBeUndefined();expect(record.estimate.points).toEqual([]);
  });
});

describe('LiveFeed',()=>{
  const record=()=>recordFor({rgb:new Uint8Array(10),depth:new Uint16Array(5)} as any);
  it('serves queued records in order and tracks bytes',async()=>{
    const feed=new LiveFeed();feed.push(record());feed.push(record());
    expect(feed.length).toBe(2);expect(feed.queuedBytes).toBe(40);
    await feed.next();expect(feed.queuedBytes).toBe(20);await feed.next();expect(feed.queuedBytes).toBe(0);
  });
  it('hands a pushed record straight to a waiting consumer and releases it on wake',async()=>{
    const feed=new LiveFeed(),pending=feed.next();feed.push(record());expect(await pending).toBeDefined();expect(feed.length).toBe(0);
    const idle=feed.next();feed.wake();expect(await idle).toBeUndefined();
  });
});
