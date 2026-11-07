// Publish landmark-input native SLAM programs as static assets under public/vendor/slam-landmark.
// Run export-landmark-profiles.mjs first. The model parameter catalog is the image programs' (the
// interface is shared), so it is copied from the published image manifest when that is installed.
import {mkdir,readFile,writeFile,rm} from 'node:fs/promises';
import {createHash} from 'node:crypto';
import {homedir} from 'node:os';
import {join,resolve} from 'node:path';
import {parseArgs} from 'node:util';
import {publishedAssets,readExport} from './slam-native-manifest.mjs';
import {D435_PAIRED_RESOLUTIONS} from '../src/camera-resolutions.mjs';
import {LANDMARK_CAPACITY} from './generate-landmark-models.mjs';

const {values}=parseArgs({options:{exports:{type:'string'},destination:{type:'string'},image:{type:'string'}}});
const root=resolve(values.exports??join(homedir(),'scratch/slam_web/build/slam-landmark-profiles'));
const destination=resolve(values.destination??'public/vendor/slam-landmark');
const imageManifest=await readFile(resolve(values.image??'public/vendor/slam/manifest.json'),'utf8').then(JSON.parse,()=>undefined);
const manifest={schemaVersion:1,kind:'landmark',exportedFrom:'rumoca native_step',profiles:{}};
const blobs=new Map();
for(const mode of D435_PAIRED_RESOLUTIONS){
  const directory=join(root,'profiles',mode.id);
  let profile;
  try{profile=JSON.parse(await readFile(join(directory,'profile.json'),'utf8'));}
  catch(error){if(error.code==='ENOENT')continue;throw error;}
  const actualSha=createHash('sha256').update(await readFile(join(directory,'source.mo'))).digest('hex');
  if(profile.resolution!==mode.id||profile.sourceSha256!==actualSha)throw new Error(`Stale landmark profile ${mode.id}; export it again`);
  const exports={
    initialize:await readExport(profile.models.initialize,join(directory,'init-named')),
    step:await readExport(profile.models.step,join(directory,'step-named')),
  };
  const catalog=imageManifest?.profiles?.[mode.id]?.models;
  for(const [key,item] of Object.entries(exports)){
    const inputs=item.layout.inputs;
    if(inputs.rgb?.count!==mode.width*mode.height*3||inputs.depth?.count!==mode.width*mode.height)throw new Error(`Landmark profile ${mode.id} has mismatched image dimensions`);
    if(inputs.landmarkPixels?.count!==LANDMARK_CAPACITY*2||inputs.landmarkScore?.count!==LANDMARK_CAPACITY||inputs.landmarkCount?.count!==1)
      throw new Error(`Landmark profile ${mode.id} ${key} does not expose the landmark inputs`);
    if(catalog?.[key]?.parameters)item.layout.parameters=catalog[key].parameters;
  }
  const result=publishedAssets(exports,{sourceSha256:profile.sourceSha256});
  manifest.profiles[mode.id]={sourceSha256:profile.sourceSha256,models:result.manifest.models};
  for(const [name,blob] of result.blobs)if(!blobs.has(name))blobs.set(name,blob);
}
const ids=Object.keys(manifest.profiles);
if(!ids.length)throw new Error(`No landmark exports under ${root}; run scripts/export-landmark-profiles.mjs`);
// Like the image manifest, the top level carries one default profile.
manifest.sourceSha256=manifest.profiles[ids[0]].sourceSha256;
manifest.models=manifest.profiles[ids[0]].models;
await rm(destination,{recursive:true,force:true});
await mkdir(destination,{recursive:true});
let total=0;
for(const [name,{bytes}] of blobs){await writeFile(join(destination,name),bytes);total+=bytes.length;}
await writeFile(join(destination,'manifest.json'),JSON.stringify(manifest));
console.log(`Landmark SLAM assets ready: ${ids.join(', ')}; ${blobs.size} files, ${(total/1e6).toFixed(1)} MB compressed.`);
