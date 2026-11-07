import {describe,expect,it} from 'vitest';
import {emptyLandmarks,encodeLandmarkPacket,evenRound,extractLandmarks,fastScores,LANDMARK_CAPACITY,parseLandmarkPacket,PATCH_SIZE,
  scoreFloor,selectFeatures,selectionThreshold,sparseFrame,validateLandmarks,type LandmarkHeader} from '../src/landmark-source';

// Deterministic textured raster: blobs on a gradient give FAST corners.
function scene(width:number,height:number){
  const rgb=new Uint8Array(width*height*3),depth=new Uint16Array(width*height);
  let seed=12345;const rand=()=>{seed=(seed*1664525+1013904223)>>>0;return seed/2**32;};
  const blobs=Array.from({length:80},()=>({x:8+rand()*(width-16),y:8+rand()*(height-16),r:2+rand()*5,v:rand()<.5?30:225}));
  for(let y=0;y<height;y++)for(let x=0;x<width;x++){
    let v=110+20*Math.sin(x/17)+15*Math.cos(y/11);
    for(const b of blobs)if(Math.hypot(x-b.x,y-b.y)<b.r)v=b.v;
    const i=y*width+x;rgb[3*i]=v;rgb[3*i+1]=v*.9;rgb[3*i+2]=v*.8;depth[i]=1500+x;
  }
  return {rgb,depth};
}
const header=(count:number,width:number,height:number):LandmarkHeader=>({type:'landmarks',t:1.25,count,
  color:{width:1920,height:1440,fx:1500,fy:1500,cx:960,cy:720},target:{width,height}});

describe('landmark selection',()=>{
  const width=160,height=120,{rgb,depth}=scene(width,height);
  const options={depthUnitsMeters:.001};
  it('uses the width-dependent threshold of the native interface',()=>{
    expect(selectionThreshold(848)).toBe(18);expect(selectionThreshold(1280)).toBe(18);
    expect(selectionThreshold(424)).toBe(9);expect(selectionThreshold(100)).toBe(8);
  });
  it('rounds half to even like the Modelica selector',()=>{
    expect(evenRound(2.5)).toBe(2);expect(evenRound(3.5)).toBe(4);expect(evenRound(-1.5)).toBe(-2);expect(evenRound(2.4)).toBe(2);
  });
  it('selects separated, ranked, depth-valid corners inside the border',()=>{
    const set=extractLandmarks(rgb,depth,width,height,options);
    expect(set.count).toBeGreaterThan(10);expect(set.count).toBeLessThanOrEqual(LANDMARK_CAPACITY);
    for(let i=0;i<set.count;i++){
      expect(set.x[i]).toBeGreaterThanOrEqual(3);expect(set.x[i]).toBeLessThanOrEqual(width-4);
      expect(set.y[i]).toBeGreaterThanOrEqual(3);expect(set.y[i]).toBeLessThanOrEqual(height-4);
      expect(set.score[i]).toBeGreaterThanOrEqual(selectionThreshold(width));
      if(i)expect(set.score[i]).toBeLessThanOrEqual(set.score[i-1]+1e-3);
      for(let j=0;j<i;j++)expect(Math.max(Math.abs(set.x[i]-set.x[j]),Math.abs(set.y[i]-set.y[j]))).toBeGreaterThan(3);
    }
  });
  it('drops corners without a usable depth return',()=>{
    const none=new Uint16Array(depth.length);
    expect(extractLandmarks(rgb,none,width,height,options).count).toBe(0);
    const near=new Uint16Array(depth.length).fill(100); // 0.1 m is inside the 0.28 m minimum
    expect(extractLandmarks(rgb,near,width,height,options).count).toBe(0);
  });
  it('caps the landmark count',()=>{
    expect(extractLandmarks(rgb,depth,width,height,{...options,capacity:5}).count).toBeLessThanOrEqual(5);
  });
  it('selects the same corners with and without the cardinal pre-test',()=>{
    const full=fastScores(rgb,width,height),floor=scoreFloor(selectionThreshold(width));
    const pruned=fastScores(rgb,width,height,floor);
    const pick=(scores:Float64Array)=>selectFeatures(scores,width,height,{depth,depthUnitsMeters:.001});
    expect(pick(pruned)).toEqual(pick(full));
    // Every pruned pixel truly scores below the floor.
    for(let i=0;i<full.length;i++)if(pruned[i]===0&&full[i]!==0)expect(full[i]).toBeLessThan(floor);
    for(let i=0;i<full.length;i++)if(pruned[i]!==0)expect(pruned[i]).toBe(full[i]);
  });
  it('scores a flat raster as zero',()=>{
    expect(Math.max(...fastScores(new Uint8Array(width*height*3).fill(100),width,height))).toBe(0);
  });
});

describe('landmark packets',()=>{
  const width=160,height=120,{rgb,depth}=scene(width,height);
  const set=extractLandmarks(rgb,depth,width,height,{depthUnitsMeters:.001});
  it('round-trips through the wire format',()=>{
    const bytes=encodeLandmarkPacket(header(set.count,width,height),set);
    expect(bytes.length).toBeLessThan(40_000);
    const parsed=parseLandmarkPacket(bytes);
    expect(parsed.header.count).toBe(set.count);
    expect(Array.from(parsed.landmarks.x)).toEqual(Array.from(set.x));
    expect(Array.from(parsed.landmarks.z)).toEqual(Array.from(set.z));
    expect(Array.from(parsed.landmarks.patch)).toEqual(Array.from(set.patch));
    expect(Array.from(parsed.landmarks.score)).toEqual(Array.from(set.score));
  });
  it('rejects malformed packets',()=>{
    const bytes=encodeLandmarkPacket(header(set.count,width,height),set);
    expect(()=>parseLandmarkPacket(bytes.subarray(0,bytes.length-1))).toThrow(/length/);
    const bad=bytes.slice();bad[0]^=1;expect(()=>parseLandmarkPacket(bad)).toThrow(/Unsupported/);
    expect(()=>parseLandmarkPacket(new Uint8Array(3))).toThrow(/Truncated/);
    const outside=emptyLandmarks(1);outside.x[0]=1;outside.y[0]=50;
    expect(()=>validateLandmarks(outside,width,height)).toThrow(/border/);
    const over=emptyLandmarks(1);over.x[0]=10;over.y[0]=10;over.patch[0]=766;
    expect(()=>validateLandmarks(over,width,height)).toThrow(/765/);
  });
  it('rebuilds a sparse canvas whose patches and depth equal the source',()=>{
    const sparse=sparseFrame(set,width,height);
    expect(sparse.count).toBe(set.count);
    for(let i=0;i<set.count;i++){
      const x=set.x[i],y=set.y[i];
      expect(sparse.depth[y*width+x]).toBe(depth[y*width+x]);
      expect(sparse.pixels[2*i]).toBe(x);expect(sparse.pixels[2*i+1]).toBe(y);
      for(let row=0;row<7;row++)for(let column=0;column<7;column++){
        const p=((y+row-3)*width+x+column-3)*3;
        expect(sparse.rgb[p]+sparse.rgb[p+1]+sparse.rgb[p+2]).toBe(rgb[p]+rgb[p+1]+rgb[p+2]);
        expect(sparse.rgb[p]).toBeLessThanOrEqual(255);
      }
    }
    // Nothing outside the patches is painted.
    let painted=0;for(let i=0;i<sparse.depth.length;i++)if(sparse.depth[i])painted++;
    expect(painted).toBe(set.count);
    expect(set.patch.length).toBe(set.count*PATCH_SIZE);
  });
});
