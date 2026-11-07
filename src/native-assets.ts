import pinText from '../slam-native-package.json?raw';
const delivery:string=JSON.parse(pinText).sha256;
/** Invalidate the small manifest when a new native delivery is pinned. */
export type NativeVariant='image'|'landmark';
/** Published program sets: `image` takes the camera raster; `landmark` takes the phone's selected corners. */
export const NATIVE_VARIANT_DIRECTORY:Record<NativeVariant,string>={image:'vendor/slam',landmark:'vendor/slam-landmark'};
export function nativeManifestUrl(base:URL,variant:NativeVariant='image'){
  const url=new URL(`${NATIVE_VARIANT_DIRECTORY[variant]}/manifest.json`,base);
  url.searchParams.set('delivery',delivery);return url;
}
