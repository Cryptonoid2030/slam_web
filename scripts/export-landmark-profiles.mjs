// Export the landmark-input native SLAM programs (see generate-landmark-models.mjs).
// Build Rumoca's native_step example separately, then supply that executable and the Reset
// state. The Reset values are source-owned and identical to the pinned image programs' state_init.bin
// (the State record is shared), so the exporter for the image programs' package can supply them:
//   gunzip -c public/vendor/slam/<hash>-state_init.bin.gz > reset.bin
//   node scripts/export-landmark-profiles.mjs --binary native_step --reset-bin reset.bin --state-hash 8ad289e707119f89
import {readFile,writeFile,mkdir,access} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {spawn} from 'node:child_process';
import {createWriteStream} from 'node:fs';
import {homedir} from 'node:os';
import {join,resolve} from 'node:path';
import {parseArgs} from 'node:util';
import {D435_PAIRED_RESOLUTIONS} from '../src/camera-resolutions.mjs';
import {rgbdSlamNativeSourceManifest as native} from '../src/modelica-slam-source-manifest.mjs';
import {generateLandmarkModels} from './generate-landmark-models.mjs';

const {values}=parseArgs({options:{binary:{type:'string'},'reset-bin':{type:'string'},'state-hash':{type:'string'},'source-root':{type:'string'},output:{type:'string'},resolution:{type:'string'},revision:{type:'string'},'private-bytes':{type:'string'}}});
if(!values.binary||!values['reset-bin']||!/^[a-f0-9]{16}$/i.test(values['state-hash']??''))
  throw new Error('Supply --binary PATH_TO_NATIVE_STEP, --reset-bin STATE_INIT.bin and the 16-digit --state-hash');
const root=resolve(values.output??join(homedir(),'scratch/slam_web/build/slam-landmark-profiles'));
const modes=D435_PAIRED_RESOLUTIONS.filter(mode=>!values.resolution||mode.id===values.resolution);
if(!modes.length)throw new Error('Unsupported paired D435 resolution');
const reset=resolve(values['reset-bin']);
await access(reset);await mkdir(root,{recursive:true});
const sha=bytes=>createHash('sha256').update(bytes).digest('hex');
const binary=resolve(values.binary),binarySha256=sha(await readFile(binary)),resetSha256=sha(await readFile(reset));
const sourceRoot=resolve(values['source-root']??join(import.meta.dirname,'..'));
const image=(await Promise.all(native.paths.map(path=>readFile(join(sourceRoot,path),'utf8')))).join(native.separator);
const base=image+'\n'+await generateLandmarkModels(sourceRoot);
for(const mode of modes){
  const directory=join(root,'profiles',mode.id);await mkdir(directory,{recursive:true});
  const models={initialize:`D435_${mode.id}LandmarkInitialize`,step:`D435_${mode.id}LandmarkStep`};
  const source=base+Object.entries(models).map(([key,name])=>`\nmodel ${name}\n  extends RGBDLandmarkSLAM${key==='initialize'?'Initialize':'Step'}(imageHeight=${mode.height},imageWidth=${mode.width},channelCount=3,depthUnits=0.001);\nend ${name};\n`).join('');
  const sourcePath=join(directory,'source.mo'),sourceSha256=sha(source);
  await writeFile(sourcePath,source);
  const commands=[];
  for(const [key,model] of Object.entries(models)){
    const args=[sourcePath,model,'1',values['private-bytes']??'134217728','1171914752',join(directory,key==='initialize'?'init-named':'step-named'),'--carried-state'];
    if(key==='initialize')args.push('--state-bin',reset,values['state-hash'].toLowerCase());
    commands.push({binary,args});console.log(`Exporting ${mode.id} ${key}`);
    const log=createWriteStream(join(directory,`${key}.log`));
    try{await new Promise((accept,reject)=>{
      const child=spawn(binary,args,{stdio:['ignore','pipe','pipe']});
      child.stdout.pipe(log,{end:false});child.stderr.pipe(log,{end:false});
      child.on('error',reject);child.on('close',code=>code===0?accept():reject(new Error(`${model} export failed (${code}); see ${directory}/${key}.log`)));
    });}finally{log.end();}
  }
  await writeFile(join(directory,'profile.json'),JSON.stringify({resolution:mode.id,kind:'landmark',sourceSha256,baseSourceSha256:sha(image),binarySha256,resetSha256,revision:values.revision,models,commands},null,2)+'\n');
  console.log(`Exported ${mode.id}, source ${sourceSha256.slice(0,12)}`);
}
