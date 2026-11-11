import {mkdir,readFile,mkdtemp,stat,rename,rm,cp} from 'node:fs/promises';
import {createReadStream,createWriteStream} from 'node:fs';
import {createHash} from 'node:crypto';
import {Readable} from 'node:stream';
import {pipeline} from 'node:stream/promises';
import {execFile} from 'node:child_process';
import {promisify} from 'node:util';
import {homedir} from 'node:os';
import {join,dirname,resolve} from 'node:path';
import {D435_PAIRED_RESOLUTIONS} from '../src/camera-resolutions.mjs';

const run=promisify(execFile);
async function verify(path,pin){
  if((await stat(path)).size!==pin.bytes)throw new Error('Native SLAM package size mismatch');
  const hash=createHash('sha256');for await(const chunk of createReadStream(path))hash.update(chunk);
  if(hash.digest('hex')!==pin.sha256)throw new Error('Native SLAM package hash mismatch');
}

/** Install an immutable native delivery when local compiler exports are absent.
 * The archive and current Modelica composition must both match the pin. */
export async function installPublishedNativeAssets(destination,sourceSha256){
  const pin=JSON.parse(await readFile(new URL('../slam-native-package.json',import.meta.url),'utf8'));
  if(pin.sourceSha256!==sourceSha256)throw new Error('Published native SLAM source differs from this checkout; export the current models locally');
  const cacheRoot=process.env.SLAM_NATIVE_CACHE_ROOT||join(homedir(),'scratch/slam_web/downloads/native-slam');
  const cache=join(resolve(cacheRoot),pin.sha256);
  await mkdir(cache,{recursive:true});
  const archive=process.env.SLAM_NATIVE_PACKAGE??join(cache,pin.asset);
  try{await verify(archive,pin);}
  catch(error){
    if(error.code!=='ENOENT'||process.env.SLAM_NATIVE_PACKAGE)throw error;
    const staging=await mkdtemp(join(cache,'download-'));
    try{
      const url=`https://github.com/${pin.repository}/releases/download/${pin.tag}/${pin.asset}`;
      console.log(`Downloading pinned native SLAM programs (${(pin.bytes/1e6).toFixed(1)} MB)`);
      const response=await fetch(url);
      if(!response.ok||!response.body)throw new Error(`Native SLAM download failed (HTTP ${response.status})`);
      const partial=join(staging,pin.asset);
      await pipeline(Readable.fromWeb(response.body),createWriteStream(partial));
      await verify(partial,pin);await rename(partial,archive);
    }finally{await rm(staging,{recursive:true,force:true});}
  }
  const unpacked=await mkdtemp(join(cache,'unpack-'));
  try{
    const {stdout}=await run('tar',['--list','--file',archive]);
    for(const entry of stdout.trim().split(/\r?\n/))
      if(entry!=='.'&&entry!=='./'&&!/^(?:\.\/)?(?:manifest\.json|[a-f0-9]{16}-[A-Za-z0-9_.-]+)$/.test(entry))throw new Error('Unexpected native SLAM package entry');
    await run('tar',['--extract','--file',archive,'--directory',unpacked,'--no-same-owner','--no-same-permissions']);
    const manifest=JSON.parse(await readFile(join(unpacked,'manifest.json'),'utf8'));
    if(manifest.schemaVersion!==1||(manifest.baseSourceSha256??manifest.sourceSha256)!==sourceSha256)throw new Error('Native SLAM package manifest mismatch');
    for(const mode of D435_PAIRED_RESOLUTIONS)for(const key of ['initialize','step']){
      const inputs=manifest.profiles?.[mode.id]?.models?.[key]?.inputs;
      if(inputs?.rgb?.count!==mode.width*mode.height*3||inputs?.depth?.count!==mode.width*mode.height)throw new Error(`Native SLAM package lacks ${mode.id} ${key}`);
    }
    await mkdir(dirname(destination),{recursive:true});await rm(destination,{recursive:true,force:true});
    await cp(unpacked,destination,{recursive:true});
    console.log(`Pinned native SLAM assets ready: Rumoca ${pin.rumocaRevision.slice(0,12)}, all five D435 modes`);
  }finally{await rm(unpacked,{recursive:true,force:true});}
}
