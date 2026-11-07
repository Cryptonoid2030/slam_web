import {NATIVE_VARIANT_DIRECTORY,nativeManifestUrl,type NativeVariant} from './native-assets';
import type {ModelParameter} from './model-parameters';
/** Host for one compiler-exported native Modelica program (Rumoca native_step
 * export). The module owns the algorithm: it imports one linear memory holding
 * Y, P, scratch and typed lanes, exports `eval_assignments`, and declares its
 * carried state as previous/next tables. The host only writes declared inputs,
 * calls the entry, moves next state into previous state and reads outputs. */
export interface SlamNativeSlot {offset:number;count:number;lane:'f64'|'i64'|'u8'}
export interface SlamNativeLayout {
  parameters?:ModelParameter[];
  model:string;pages:number;y_bytes:number;p_offset:number;scratch_offset:number;typed_offset:number;typed_bytes:number;
  y_count:number;p_count:number;
  state:{count:number;previous_hash:string;next_hash:string};
  /** Flat [from, to, bytes] runs copying next state over previous state. */
  carry:number[];
  inputs:Record<string,SlamNativeSlot>;
  /** Derived scalar outputs in typed lanes, absolute byte offsets. */
  outputs:{name:string;lane:'f64'|'i64'|'u8';offset:number}[];
  y_names:string[];
  model_outputs:{name:string;index:number;count:number}[];
  state_runs:{name:string;lane:'f64'|'i64'|'u8';previous:number;next:number;count:number}[];
  files:Record<string,{file:string;sha256:string;bytes:number;encoding:'gzip'|'identity'}>;
}
export interface SlamNativeManifest {
  schemaVersion:1;sourceSha256:string;models:Record<string,SlamNativeLayout>;
  profiles?:Record<string,{sourceSha256:string;models:Record<string,SlamNativeLayout>}>;
}
export type SlamNativeFetch=(file:string,encoding:'gzip'|'identity')=>Promise<Uint8Array>;

const MATH=['abs','sin','cos','tan','asin','acos','atan','sinh','cosh','tanh','exp','log','log10','pow','atan2'] as const;
const LANE_BYTES={f64:8,i64:8,u8:1} as const;

export class SlamNativeProgram {
  private needsCarry=false;
  private constructor(readonly layout:SlamNativeLayout,readonly memory:WebAssembly.Memory,
    private readonly entry:(y:number,p:number,time:number,scratch:number,typed:number)=>number,
    private readonly state:{in:Uint32Array;out:Uint32Array;lane:Uint8Array},
    private readonly fetchFile:SlamNativeFetch){}

  static allocateMemory(pages:number){
    try{return new WebAssembly.Memory({initial:pages,maximum:pages});}
    catch(error){throw new Error(`This browser cannot reserve the ${(pages*65536/1e9).toFixed(2)} GB the native SLAM program declares (${error instanceof Error?error.message:String(error)})`);}
  }

  static async load(layout:SlamNativeLayout,fetchFile:SlamNativeFetch){
    return (await this.prepare(layout,fetchFile))();
  }

  /** Compile once; later state initialization reuses this exact module. */
  static async prepare(layout:SlamNativeLayout,fetchFile:SlamNativeFetch){
    const asset=layout.files.module;
    if(!asset)throw new Error(`Native program ${layout.model} lacks module`);
    const bytes=await fetchFile(asset.file,asset.encoding);
    const module=await WebAssembly.compile(bytes.buffer.slice(bytes.byteOffset,bytes.byteOffset+bytes.byteLength) as ArrayBuffer);
    return ()=>this.instantiate(layout,fetchFile,module);
  }

  private static async instantiate(layout:SlamNativeLayout,fetchFile:SlamNativeFetch,module:WebAssembly.Module){
    const file=(key:string)=>{const entry=layout.files[key];if(!entry)throw new Error(`Native program ${layout.model} lacks ${key}`);return fetchFile(entry.file,entry.encoding);};
    const [y,p,lanes,stateIn,stateOut,stateLane]=await Promise.all(['y','p','lanes','state_in','state_out','state_lane'].map(file));
    const stateInit=layout.files.state_init?await file('state_init'):undefined;
    if(y.length!==layout.y_bytes)throw new Error('Native program Y image size mismatch');
    if(p.length!==layout.p_count*8)throw new Error('Native program P image size mismatch');
    if(lanes.length!==layout.typed_bytes&&lanes.length!==Math.ceil(layout.typed_bytes/8)*8)throw new Error('Native program typed lane image size mismatch');
    if(stateIn.length!==layout.state.count*4||stateOut.length!==layout.state.count*4||stateLane.length!==layout.state.count)throw new Error('Native program state tables do not match the declared state count');
    const memory=SlamNativeProgram.allocateMemory(layout.pages);
    const env:Record<string,WebAssembly.ImportValue>={memory};
    for(const item of WebAssembly.Module.imports(module)){
      if(item.module!=='env')throw new Error(`Native program imports unexpected module ${item.module}`);
      if(item.name==='memory')continue;
      if(item.kind!=='function'||!(MATH as readonly string[]).includes(item.name))throw new Error(`Native program imports unexpected symbol ${item.name}`);
      env[item.name]=Math[item.name as typeof MATH[number]];
    }
    const instance=await WebAssembly.instantiate(module,{env});
    const entry=instance.exports.eval_assignments;
    if(typeof entry!=='function'||entry.length!==5)throw new Error('Native program does not export eval_assignments(y,p,time,scratch,typed)');
    const bytes=new Uint8Array(memory.buffer);
    bytes.set(y,0);bytes.set(p,layout.p_offset);bytes.set(lanes.subarray(0,layout.typed_bytes),layout.typed_offset);
    const tables={in:aligned(stateIn,Uint32Array),out:aligned(stateOut,Uint32Array),lane:new Uint8Array(stateLane)};
    const program=new SlamNativeProgram(layout,memory,entry as never,tables,fetchFile);
    if(stateInit){
      const values=aligned(stateInit,Float64Array);
      if(values.length!==layout.state.count)throw new Error('Native program initial state size mismatch');
      const view=new DataView(memory.buffer);
      for(let i=0;i<layout.state.count;i++){
        const at=tables.in[i],value=values[i];
        if(tables.lane[i]===0)view.setFloat64(at,value,true);
        else if(tables.lane[i]===1)view.setBigInt64(at,BigInt(value),true);
        else view.setUint8(at,value!==0?1:0);
      }
    }
    return program;
  }

  async reset(){
    const file=(key:string)=>{const asset=this.layout.files[key];return this.fetchFile(asset.file,asset.encoding);};
    const [y,p,lanes,stateInit]=await Promise.all([file('y'),file('p'),file('lanes'),this.layout.files.state_init?file('state_init'):undefined]);
    if(y.length!==this.layout.y_bytes||p.length!==this.layout.p_count*8||
      (lanes.length!==this.layout.typed_bytes&&lanes.length!==Math.ceil(this.layout.typed_bytes/8)*8))throw new Error('Native reset image size mismatch');
    const values=stateInit?aligned(stateInit,Float64Array):undefined;
    if(values&&values.length!==this.layout.state.count)throw new Error('Native program initial state size mismatch');
    const bytes=new Uint8Array(this.memory.buffer);bytes.fill(0);
    bytes.set(y);bytes.set(p,this.layout.p_offset);bytes.set(lanes.subarray(0,this.layout.typed_bytes),this.layout.typed_offset);
    if(values){
      const view=new DataView(this.memory.buffer);
      for(let i=0;i<this.layout.state.count;i++){
        const at=this.state.in[i],value=values[i];
        if(this.state.lane[i]===0)view.setFloat64(at,value,true);
        else if(this.state.lane[i]===1)view.setBigInt64(at,BigInt(value),true);
        else view.setUint8(at,value!==0?1:0);
      }
    }
    this.needsCarry=false;
  }

  /** Declared Real input as a live view over linear memory. */
  real(name:string):Float64Array {
    const slot=this.slot(name,'f64');return new Float64Array(this.memory.buffer,slot.offset,slot.count);
  }
  integer(name:string):BigInt64Array {
    const slot=this.slot(name,'i64');return new BigInt64Array(this.memory.buffer,slot.offset,slot.count);
  }
  boolean(name:string):Uint8Array {
    const slot=this.slot(name,'u8');return new Uint8Array(this.memory.buffer,slot.offset,slot.count);
  }
  hasInput(name:string){return Object.hasOwn(this.layout.inputs,name);}
  private slot(name:string,lane:'f64'|'i64'|'u8'){
    const slot=this.layout.inputs[name];
    if(!slot)throw new Error(`Native program ${this.layout.model} declares no host input ${name}`);
    if(slot.lane!==lane)throw new Error(`Native program input ${name} is ${slot.lane}, not ${lane}`);
    if(slot.offset%LANE_BYTES[lane]!==0||slot.offset+slot.count*LANE_BYTES[lane]>this.memory.buffer.byteLength)throw new Error(`Native program input ${name} is outside memory`);
    return slot;
  }
  /** Y scalars by index, as a live view. */
  y(index:number,count:number):Float64Array {
    if(!Number.isSafeInteger(index)||index<0||index+count>this.layout.y_count)throw new Error('Native program Y range is outside the declared layout');
    return new Float64Array(this.memory.buffer,index*8,count);
  }
  /** Declared model output living in Y, when the export names it. */
  output(name:string):Float64Array|undefined {
    const entry=this.layout.model_outputs.find(item=>item.name===name);
    return entry?this.y(entry.index,entry.count):undefined;
  }
  /** Carried state run by base name (next side), when the export names it. */
  stateRun(name:string):{lane:'f64'|'i64'|'u8';view:Float64Array|BigInt64Array|Uint8Array}|undefined {
    const run=this.layout.state_runs.find(item=>item.name===name);
    if(!run)return undefined;
    if(run.lane==='f64')return {lane:run.lane,view:new Float64Array(this.memory.buffer,run.next,run.count)};
    if(run.lane==='i64')return {lane:run.lane,view:new BigInt64Array(this.memory.buffer,run.next,run.count)};
    return {lane:run.lane,view:new Uint8Array(this.memory.buffer,run.next,run.count)};
  }
  /** Derived scalar outputs in typed lanes. */
  derived():Record<string,number|boolean> {
    const view=new DataView(this.memory.buffer),result:Record<string,number|boolean>={};
    for(const {name,lane,offset} of this.layout.outputs){
      result[name]=lane==='f64'?view.getFloat64(offset,true):lane==='i64'?Number(view.getBigInt64(offset,true)):view.getUint8(offset)!==0;
    }
    return result;
  }
  /** Copy this program's next state into another program's previous state. */
  transferTo(target:SlamNativeProgram){
    if(this.layout.state.next_hash!==target.layout.state.previous_hash)throw new Error('Native program state name sets differ between stages');
    const source=new Uint8Array(this.memory.buffer),destination=new Uint8Array(target.memory.buffer);
    const {count}=this.layout.state,out=this.state.out,into=target.state.in,lane=target.state.lane;
    for(let i=0;i<count;i++){
      const width=lane[i]===2?1:8,from=out[i],to=into[i];
      for(let b=0;b<width;b++)destination[to+b]=source[from+b];
    }
    target.needsCarry=false;
  }
  /** Evaluate once at `time`. Carries state from the previous call first. */
  step(time:number):number {
    if(!Number.isFinite(time))throw new Error('Native program time must be finite');
    if(this.needsCarry){
      const live=new Uint8Array(this.memory.buffer),carry=this.layout.carry;
      for(let i=0;i<carry.length;i+=3)live.copyWithin(carry[i+1],carry[i],carry[i]+carry[i+2]);
    }
    const status=this.entry(0,this.layout.p_offset,time,this.layout.scratch_offset,this.layout.typed_bytes?this.layout.typed_offset:0);
    this.needsCarry=true;
    return status;
  }
}

function aligned<T extends Uint32Array|Float64Array>(bytes:Uint8Array,Type:{new(buffer:ArrayBuffer):T;BYTES_PER_ELEMENT:number}):T {
  if(bytes.byteLength%Type.BYTES_PER_ELEMENT)throw new Error('Native program table is not element aligned');
  const copy=new ArrayBuffer(bytes.byteLength);new Uint8Array(copy).set(bytes);
  return new Type(copy);
}

/** Fetch and decode one published asset from the same origin. */
export async function fetchNativeAsset(base:URL,file:string,encoding:'gzip'|'identity',variant:NativeVariant='image'):Promise<Uint8Array> {
  const url=new URL(`${NATIVE_VARIANT_DIRECTORY[variant]}/${file}`,base);
  if(url.origin!==base.origin)throw new Error('Native SLAM assets must be same origin');
  const response=await fetch(url);
  if(!response.ok)throw new Error(`Native SLAM asset ${file} is unavailable (HTTP ${response.status})`);
  const bytes=new Uint8Array(await response.arrayBuffer());
  // Some static servers announce .gz files as content encoding and deliver them
  // decoded; only inflate what still carries the gzip signature.
  if(encoding==='gzip'&&bytes.length>=2&&bytes[0]===0x1f&&bytes[1]===0x8b){
    const inflated=new Blob([bytes]).stream().pipeThrough(new DecompressionStream('gzip'));
    return new Uint8Array(await new Response(inflated).arrayBuffer());
  }
  return bytes;
}
export async function fetchNativeManifest(base:URL,resolution?:string,variant:NativeVariant='image'):Promise<SlamNativeManifest> {
  const url=nativeManifestUrl(base,variant);
  if(url.origin!==base.origin)throw new Error('Native SLAM assets must be same origin');
  const response=await fetch(url);
  if(!response.ok)throw new Error('Native SLAM programs are not published on this site; build the assets from a compiler export first');
  const manifest=await response.json();
  if(manifest?.schemaVersion!==1||!manifest.models?.initialize||!manifest.models?.step)throw new Error('Unsupported native SLAM manifest');
  if(resolution){
    const profile=manifest.profiles?.[resolution];
    if(profile){
      if(!profile.models?.initialize||!profile.models?.step||typeof profile.sourceSha256!=='string')throw new Error('Unsupported native SLAM resolution profile');
      return {schemaVersion:1,...profile};
    }
    const [width,height]=resolution.split('x').map(Number);
    if(manifest.models.step.inputs?.rgb?.count!==width*height*3||manifest.models.step.inputs?.depth?.count!==width*height)
      throw new Error(`Native SLAM programs for ${resolution} are not published on this site; export that camera profile first`);
  }
  return manifest;
}
