// Generates models/SLAM/D435LandmarkSLAM.mo: landmark-input entry points for the native SLAM.
//
// The FAST SLAM programs find their own corners from the whole RGB raster. Downstream of that
// selection, matching, registration, the filter and loop closure only see 7x7 descriptors and 3D
// points. These entry points take the selected corner pixels as inputs instead of scoring the raster,
// so a client that has already picked its landmarks (the iPhone) never has to send the image.
// Everything else is copied verbatim from the pinned sources; nothing in the existing files changes.
//
//   node scripts/generate-landmark-models.mjs          # writes the file
//   node scripts/generate-landmark-models.mjs --check  # fails if the file is stale
import {readFile,writeFile} from 'node:fs/promises';
import {join,resolve} from 'node:path';

export const LANDMARK_CAPACITY=350;
export const LANDMARK_OUTPUT='models/SLAM/RGBDLandmarkSLAM.mo';

const root=resolve(import.meta.dirname,'..');
const sources={
  localizationStep:'models/Estimation/Localization/RGBDFastInertialLocalizationStep.mo',
  localizationInitialize:'models/Estimation/Localization/RGBDFastInertialLocalizationInitialize.mo',
  slamStep:'models/SLAM/RGBDFastSLAMStep.mo',
  slamInitialize:'models/SLAM/RGBDFastSLAMInitialize.mo',
};

/** The text from the line that starts with `start` through the line that starts with `end`. */
function section(text,start,end){
  const from=text.search(new RegExp(`^${start}`,'m'));
  if(from<0)throw new Error(`Landmark generator: "${start}" not found`);
  const rest=text.slice(from),stop=rest.search(new RegExp(`^${end}`,'m'));
  if(stop<0)throw new Error(`Landmark generator: "${end}" not found`);
  const close=rest.indexOf('\n',stop);
  return rest.slice(0,close<0?rest.length:close);
}
function replaceOnce(text,pattern,replacement,what){
  const index=typeof pattern==='string'?text.indexOf(pattern):text.search(pattern);
  if(index<0)throw new Error(`Landmark generator: ${what} not found`);
  if(typeof pattern==='string')return text.slice(0,index)+replacement+text.slice(index+pattern.length);
  return text.replace(pattern,replacement);
}
const landmarkInputs=`  input Real landmarkPixels[featureCapacity,2] = zeros(featureCapacity,2) "Zero-based RGB pixels of the landmarks the client selected";
  input Real landmarkScore[featureCapacity] = zeros(featureCapacity) "Selection score of each landmark; larger is stronger";
  input Real landmarkCount = 0.0 "Number of leading landmark rows that are valid";
`;
const landmarkArguments='landmarkPixels=landmarkPixels,landmarkScore=landmarkScore,landmarkCount=landmarkCount,';

/** A localization function with its raster scoring and selection replaced by the supplied landmarks. */
function landmarkLocalization(text,oldName,newName){
  let out=section(text,`(pure )?function ${oldName}`,`end ${oldName};`).split(oldName).join(newName);
  out=replaceOnce(out,/^protected$/m,`${landmarkInputs}protected`,`${newName} protected section`);
  const block=/^ {2}imageOn := SLAMExactRealEqual\(frameEnabled,1\.0\);\n[\s\S]*?^ {2}selectedCount := if selectionValid > 0\.5 then selectionCount else 0\.0;\n/m;
  if(!block.test(out))throw new Error(`Landmark generator: ${newName} selection block not found`);
  return out.replace(block,`  imageOn := SLAMExactRealEqual(frameEnabled,1.0);
  // The client already selected the landmarks; they replace the raster selection below.
  // The FAST scoring and depth qualification are deliberately kept (their result is unused): Rumoca's
  // native emission makes the step reject every prediction when RGBDDepthQualifiedScores is not called
  // here (found by bisecting: the same program with that call restored runs correctly). On the phone's
  // sparse canvas they cost little. Revisit when the compiler is fixed.
  scores := FastFrameScores(rgb,imageOn,FastSelectionScoreFloor(absoluteThreshold,1e8));
  if depthQualifiedSelection then
    scores := RGBDDepthQualifiedScores(depth,scores,rgbCalibration,depthCalibration,
      0.28,10.0,disparityNoise,noiseReferenceFx,baseline,imageOn,depthUnits=depthUnits);
  end if;
  for k in 1:featureCapacity loop
    selectedFeatures[k,1] := landmarkPixels[k,1];
    selectedFeatures[k,2] := landmarkPixels[k,2];
    selectedFeatures[k,3] := landmarkScore[k];
  end for;
  selectionCount := if imageOn then landmarkCount else 0.0;
  selectionStatus := if scores[1] >= 0.0 then 1.0 else 0.0;
  selectionConfiguration := selectedFeatureLimit >= 1.0 and selectedFeatureLimit <= featureCapacity
    and SLAMExactRealEqual(selectedFeatureLimit,floor(selectedFeatureLimit));
  selectionValid := if selectionConfiguration and selectionStatus > 0.5
    and selectionCount >= 0.0 and selectionCount <= featureCapacity then 1.0 else 0.0;
  selectedCount := if selectionValid > 0.5 then selectionCount else 0.0;
`);
}

/** A SLAM function that calls the landmark localization instead of the FAST one. */
function landmarkSlamFunction(text,oldName,newName,oldCall,newCall){
  let out=section(text,`function ${oldName}`,`end ${oldName};`).split(oldName).join(newName);
  out=replaceOnce(out,/^protected$/m,`${landmarkInputs}protected`,`${newName} protected section`);
  return replaceOnce(out,`${oldCall}(`,`${newCall}(\n    ${landmarkArguments}`,`${newName} call`);
}

/** A public model that forwards the landmark inputs. */
function landmarkModel(text,oldName,newName,oldFunction,newFunction){
  let out=section(text,`model ${oldName}`,`end ${oldName};`).split(oldName).join(newName);
  out=replaceOnce(out,'extends RGBDFastSLAMInterface',`extends RGBDFastSLAMInterface`,`${newName} extends`);
  out=replaceOnce(out,/^equation$/m,`  input Real landmarkPixels[featureCapacity,2] = zeros(featureCapacity,2);
  input Real landmarkScore[featureCapacity] = zeros(featureCapacity);
  input Real landmarkCount = 0.0;
equation`,`${newName} equation`);
  return replaceOnce(out,`${oldFunction}(`,`${newFunction}(\n      ${landmarkArguments}`,`${newName} call`);
}

export async function generateLandmarkModels(directory=root){
  const read=async key=>readFile(join(directory,sources[key]),'utf8');
  const [localizationStep,localizationInitialize,slamStep,slamInitialize]=await Promise.all(['localizationStep','localizationInitialize','slamStep','slamInitialize'].map(read));
  const parts=[
    `// Generated by scripts/generate-landmark-models.mjs from the FAST SLAM sources; do not edit.
// Landmark-input entry points: the client selects the corners, the SLAM does the rest.
`,
    landmarkLocalization(localizationStep,'AdvanceFastRGBDLocalization','AdvanceLandmarkRGBDLocalization'),
    landmarkLocalization(localizationInitialize,'InitializeFastRGBDLocalization','InitializeLandmarkRGBDLocalization'),
    landmarkSlamFunction(slamStep,'AdvanceFastSLAM','AdvanceLandmarkSLAM','AdvanceFastRGBDLocalization','AdvanceLandmarkRGBDLocalization'),
    landmarkSlamFunction(slamInitialize,'InitializeFastSLAM','InitializeLandmarkSLAM','InitializeFastRGBDLocalization','InitializeLandmarkRGBDLocalization'),
    landmarkModel(slamStep,'RGBDFastSLAMStep','RGBDLandmarkSLAMStep','AdvanceFastSLAM','AdvanceLandmarkSLAM'),
    landmarkModel(slamInitialize,'RGBDFastSLAMInitialize','RGBDLandmarkSLAMInitialize','InitializeFastSLAM','InitializeLandmarkSLAM'),
  ];
  return parts.join('\n')+'\n';
}

if(import.meta.url===`file://${process.argv[1]}`){
  const text=await generateLandmarkModels();
  const target=join(root,LANDMARK_OUTPUT);
  if(process.argv.includes('--check')){
    const current=await readFile(target,'utf8').catch(()=>'');
    if(current!==text){console.error(`${LANDMARK_OUTPUT} is stale; run node scripts/generate-landmark-models.mjs`);process.exit(1);}
  }else{
    await writeFile(target,text);
    console.log(`Wrote ${LANDMARK_OUTPUT} (${text.length} bytes)`);
  }
}
