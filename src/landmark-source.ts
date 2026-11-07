/** Landmark packets: the phone selects corners and sends only those, never the
 * raster. This module holds the wire format (IPL1), a reference implementation
 * of the selection the phone performs (FAST-9 + depth qualification + ranked
 * raster selection, mirroring models/Vision/Features), and the "sparse canvas"
 * that turns a landmark set back into the RGB/Z16 frame the native SLAM expects:
 * every pixel is zero except each landmark's 7x7 patch and its depth sample, so
 * the unchanged descriptor stage reads exactly what the full image would give.
 * No DOM or network access. */
export const LANDMARK_MAGIC=0x314c5049; // 'IPL1' little-endian
export const LANDMARK_CAPACITY=350;
export const PATCH_WIDTH=7;
export const PATCH_SIZE=PATCH_WIDTH*PATCH_WIDTH;
export const PATCH_RADIUS=3;
export const MAX_PATCH_SUM=3*255;
/** Same constants the native step applies (RGBDFastInertialLocalizationStep). */
export const SELECTION_NEAR_METERS=0.28;
export const SELECTION_FAR_METERS=10.0;
/** FAST threshold the native program derives from the image width (RGBDFastSLAMInterface): max(8, 18·min(1, width/848)). */
export function selectionThreshold(width:number){return Math.max(8,18*Math.min(1,width/848));}
export const SELECTION_RANK_SCALE=1e8;
export const SELECTION_SUPPRESSION_RADIUS=3;
export const SELECTION_BORDER=3;

export interface LandmarkSet {
  count:number;
  /** Zero-based pixel columns/rows on the target grid. */
  x:Uint16Array;y:Uint16Array;
  /** Z16 depth sample at the landmark (units given by the live config). */
  z:Uint16Array;
  /** FAST score, larger is stronger. */
  score:Float32Array;
  /** Per landmark, 49 sums r+g+b of the 7x7 patch, row-major, 0..765. */
  patch:Uint16Array;
}
export interface LandmarkHeader {
  type:'landmarks';t:number;count:number;
  /** Intrinsics of the source (uncropped) color image, as in the raw camera header. */
  color:{width:number;height:number;fx:number;fy:number;cx:number;cy:number};
  target:{width:number;height:number};
  /** Optional ARKit camera-to-world, carried for diagnostics only. */
  pose?:number[];
}
export interface LandmarkPacket {header:LandmarkHeader;landmarks:LandmarkSet}

export function landmarkPayloadBytes(count:number){return count*(2+2+2+4+2*PATCH_SIZE);}

export function emptyLandmarks(count:number):LandmarkSet {
  return {count,x:new Uint16Array(count),y:new Uint16Array(count),z:new Uint16Array(count),
    score:new Float32Array(count),patch:new Uint16Array(count*PATCH_SIZE)};
}

/** Wire format: magic, JSON header length, JSON header, then count-sized
 * little-endian arrays: x u16, y u16, z u16, score f32, patch u16[49]. */
export function encodeLandmarkPacket(header:LandmarkHeader,set:LandmarkSet):Uint8Array {
  if(header.count!==set.count)throw new Error('Landmark header count disagrees with the payload');
  const json=new TextEncoder().encode(JSON.stringify(header));
  const body=landmarkPayloadBytes(set.count),out=new Uint8Array(8+json.length+body),view=new DataView(out.buffer);
  view.setUint32(0,LANDMARK_MAGIC,true);view.setUint32(4,json.length,true);out.set(json,8);
  let o=8+json.length;
  for(let i=0;i<set.count;i++,o+=2)view.setUint16(o,set.x[i],true);
  for(let i=0;i<set.count;i++,o+=2)view.setUint16(o,set.y[i],true);
  for(let i=0;i<set.count;i++,o+=2)view.setUint16(o,set.z[i],true);
  for(let i=0;i<set.count;i++,o+=4)view.setFloat32(o,set.score[i],true);
  for(let i=0;i<set.count*PATCH_SIZE;i++,o+=2)view.setUint16(o,set.patch[i],true);
  return out;
}

export function parseLandmarkPacket(bytes:Uint8Array):LandmarkPacket {
  if(bytes.length<8)throw new Error('Truncated landmark message');
  const view=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);
  if(view.getUint32(0,true)!==LANDMARK_MAGIC)throw new Error('Unsupported landmark message');
  const headerLength=view.getUint32(4,true);
  if(8+headerLength>bytes.length)throw new Error('Truncated landmark header');
  const header=JSON.parse(new TextDecoder().decode(bytes.subarray(8,8+headerLength))) as LandmarkHeader;
  if(header?.type!=='landmarks'||!Number.isFinite(header.t)||header.t<0)throw new Error('Invalid landmark header');
  const {color,target,count}=header;
  if(!Number.isInteger(count)||count<0||count>LANDMARK_CAPACITY)throw new Error('Invalid landmark count');
  for(const v of [color?.width,color?.height,target?.width,target?.height])
    if(!Number.isInteger(v)||v<=0||v>8192)throw new Error('Invalid landmark dimensions');
  for(const v of [color.fx,color.fy,color.cx,color.cy])if(!Number.isFinite(v))throw new Error('Invalid landmark intrinsics');
  if(color.fx<=0||color.fy<=0)throw new Error('Invalid landmark intrinsics');
  if(8+headerLength+landmarkPayloadBytes(count)!==bytes.length)throw new Error('Landmark payload length mismatch');
  const set=emptyLandmarks(count);
  let o=8+headerLength;
  for(let i=0;i<count;i++,o+=2)set.x[i]=view.getUint16(o,true);
  for(let i=0;i<count;i++,o+=2)set.y[i]=view.getUint16(o,true);
  for(let i=0;i<count;i++,o+=2)set.z[i]=view.getUint16(o,true);
  for(let i=0;i<count;i++,o+=4)set.score[i]=view.getFloat32(o,true);
  for(let i=0;i<count*PATCH_SIZE;i++,o+=2)set.patch[i]=view.getUint16(o,true);
  validateLandmarks(set,target.width,target.height);
  return {header,landmarks:set};
}

/** Rejects landmarks the descriptor stage would silently drop or misread. */
export function validateLandmarks(set:LandmarkSet,width:number,height:number){
  for(let i=0;i<set.count;i++){
    if(set.x[i]<PATCH_RADIUS||set.x[i]>width-PATCH_RADIUS-1||set.y[i]<PATCH_RADIUS||set.y[i]>height-PATCH_RADIUS-1)
      throw new Error(`Landmark ${i} lies outside the descriptor border`);
    if(!Number.isFinite(set.score[i]))throw new Error(`Landmark ${i} has a non-finite score`);
  }
  for(let i=0;i<set.count*PATCH_SIZE;i++)if(set.patch[i]>MAX_PATCH_SUM)throw new Error('Landmark patch sum exceeds 765');
}

export interface SparseFrame {
  rgb:Uint8Array;depth:Uint16Array;
  /** The landmark inputs of the native program, padded to capacity. */
  pixels:Float64Array;score:Float64Array;count:number;
}
/** Splits a channel sum (0..765) into three bytes that add back to it. */
function splitSum(sum:number):[number,number,number]{
  const r=Math.min(255,sum),g=Math.min(255,sum-r);return [r,g,sum-r-g];
}
/** Paints each landmark's patch and depth sample onto zeroed full-size rasters. */
export function sparseFrame(set:LandmarkSet,width:number,height:number):SparseFrame {
  validateLandmarks(set,width,height);
  const rgb=new Uint8Array(width*height*3),depth=new Uint16Array(width*height);
  const pixels=new Float64Array(LANDMARK_CAPACITY*2),score=new Float64Array(LANDMARK_CAPACITY);
  for(let i=0;i<set.count;i++){
    const x=set.x[i],y=set.y[i];
    for(let row=0;row<PATCH_WIDTH;row++)for(let column=0;column<PATCH_WIDTH;column++){
      const [r,g,b]=splitSum(set.patch[i*PATCH_SIZE+row*PATCH_WIDTH+column]);
      const p=((y+row-PATCH_RADIUS)*width+x+column-PATCH_RADIUS)*3;
      rgb[p]=r;rgb[p+1]=g;rgb[p+2]=b;
    }
    depth[y*width+x]=set.z[i];
    pixels[2*i]=x;pixels[2*i+1]=y;score[i]=set.score[i];
  }
  return {rgb,depth,pixels,score,count:set.count};
}

/** Round half to even, as the Modelica selector ranks scores. */
export function evenRound(value:number){const low=Math.floor(value),fraction=value-low;return fraction<.5?low:fraction>.5?low+1:low%2===0?low:low+1;}

const CIRCLE:readonly (readonly [number,number])[]=[[0,-3],[1,-3],[2,-2],[3,-1],[3,0],[3,1],[2,2],[1,3],[0,3],[-1,3],[-2,2],[-3,1],[-3,0],[-3,-1],[-2,-2],[-1,-3]];
/** FAST-9 scores of an RGB8 raster (gray = (r+g+b)/3, summed in source order). Pixels that cannot
 * reach `floorScore` are left at zero, as the native program does: every 9-sample arc contains two
 * adjacent cardinal samples (top, right, bottom, left, with wraparound), so without such a pair
 * differing from the center by at least the floor the score is below it. Zero keeps every score. */
export function fastScores(rgb:Uint8Array,width:number,height:number,floorScore=0):Float64Array {
  const gray=new Float64Array(width*height),scores=new Float64Array(width*height);
  for(let i=0;i<width*height;i++)gray[i]=((rgb[3*i]+rgb[3*i+1])+rgb[3*i+2])/3;
  const differences=new Float64Array(16);
  for(let y=3;y<height-3;y++)for(let x=3;x<width-3;x++){
    const center=gray[y*width+x];
    for(let k=0;k<16;k++)differences[k]=gray[(y+CIRCLE[k][1])*width+x+CIRCLE[k][0]]-center;
    if(floorScore>0){
      let possible=false,previousBright=false,previousDark=false,firstBright=false,firstDark=false;
      for(let c=0;c<4;c++){
        const diff=differences[4*c],bright=diff>=floorScore,dark=-diff>=floorScore;
        if(c===0){firstBright=bright;firstDark=dark;}
        else possible=possible||(previousBright&&bright)||(previousDark&&dark);
        previousBright=bright;previousDark=dark;
      }
      possible=possible||(previousBright&&firstBright)||(previousDark&&firstDark);
      if(!possible)continue;
    }
    let response=0;
    for(let start=0;start<16;start++){
      let low=differences[start],high=low;
      for(let offset=1;offset<9;offset++){const v=differences[(start+offset)%16];low=low<v?low:v;high=high>v?high:v;}
      const dark=-high,arc=low>dark?low:dark;response=response>arc?response:arc;
    }
    scores[y*width+x]=response;
  }
  return scores;
}
/** The conservative floor the native program applies before scoring (FastSelectionScoreFloor). */
export function scoreFloor(threshold:number){
  return threshold>0&&threshold<=255?Math.max(0,(Math.floor(threshold*SELECTION_RANK_SCALE)-2)/SELECTION_RANK_SCALE):0;
}

export interface SelectionOptions {
  threshold?:number;capacity?:number;nearMeters?:number;farMeters?:number;depthUnitsMeters:number;
}
/** True where the calibrated depth sample at a pixel is usable (RGBDCalibratedPoint, identical RGB and depth grids). */
export function depthUsable(z16:number,units:number,near=SELECTION_NEAR_METERS,far=SELECTION_FAR_METERS){
  const sample=z16*units;return sample>near&&sample<far-0.05;
}

/** The selector the native FAST program runs: depth-qualified scores, ranked
 * by rounded score with raster-order ties, square suppression, early stop below threshold. */
export function selectFeatures(scores:Float64Array,width:number,height:number,options:SelectionOptions&{depth:Uint16Array}):number[] {
  const threshold=options.threshold??selectionThreshold(width),cap=options.capacity??LANDMARK_CAPACITY;
  const near=options.nearMeters??SELECTION_NEAR_METERS,far=options.farMeters??SELECTION_FAR_METERS;
  const qualified=new Float64Array(scores);
  for(let i=0;i<qualified.length;i++)if(qualified[i]>0&&!depthUsable(options.depth[i],options.depthUnitsMeters,near,far))qualified[i]=0;
  const guard=Math.floor(threshold*SELECTION_RANK_SCALE)-1,border=SELECTION_BORDER;
  const ranks=new Map<number,number>(),candidates:number[]=[];
  for(let y=border;y<height-border;y++)for(let x=border;x<width-border;x++){
    const index=y*width+x,rank=evenRound(qualified[index]*SELECTION_RANK_SCALE);
    if(rank>=guard){candidates.push(index);ranks.set(index,rank);}
  }
  candidates.sort((a,b)=>ranks.get(b)!-ranks.get(a)!||a-b);
  const occupied=new Uint8Array(width*height),picked:number[]=[];
  for(const index of candidates){
    if(qualified[index]<threshold)break;
    if(picked.length>=cap)break;
    if(occupied[index])continue;
    picked.push(index);
    const x=index%width,y=Math.floor(index/width),r=SELECTION_SUPPRESSION_RADIUS;
    for(let yy=Math.max(border,y-r);yy<=Math.min(height-border-1,y+r);yy++)
      for(let xx=Math.max(border,x-r);xx<=Math.min(width-border-1,x+r);xx++)occupied[yy*width+xx]=1;
  }
  return picked;
}

/** What the phone does per frame, in the same order: score, qualify by depth, select, describe. */
export function extractLandmarks(rgb:Uint8Array,depth:Uint16Array,width:number,height:number,options:SelectionOptions):LandmarkSet {
  const scores=fastScores(rgb,width,height,scoreFloor(options.threshold??selectionThreshold(width)));
  const picked=selectFeatures(scores,width,height,{...options,depth});
  const set=emptyLandmarks(picked.length);
  picked.forEach((index,i)=>{
    const x=index%width,y=Math.floor(index/width);
    set.x[i]=x;set.y[i]=y;set.z[i]=depth[index];set.score[i]=scores[index];
    for(let row=0;row<PATCH_WIDTH;row++)for(let column=0;column<PATCH_WIDTH;column++){
      const p=((y+row-PATCH_RADIUS)*width+x+column-PATCH_RADIUS)*3;
      set.patch[i*PATCH_SIZE+row*PATCH_WIDTH+column]=rgb[p]+rgb[p+1]+rgb[p+2];
    }
  });
  return set;
}
