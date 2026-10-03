/** Radar 0deg direction viewed from above, relative to the vehicle front. */
export const RADAR_INSTALLATION_DIRECTIONS=[
  {key:'front',label:'车头 · 0°',relativeDeg:0},
  {key:'right',label:'车右 · −90°',relativeDeg:-90},
  {key:'rear',label:'车尾 · 180°',relativeDeg:180},
  {key:'left',label:'车左 · +90°',relativeDeg:90}
];
const numeric=value=>String(value??'').trim()!==''&&Number.isFinite(Number(value))?Number(value):null;
const normalize=value=>Number((((value%360)+360)%360).toFixed(8))%360;

/** Convert an installation draft to existing zero_deg at nominal lower yaw 90deg. */
export function radarZeroFromRelative(value){
  const angle=numeric(value);return angle===null?null:normalize(90+angle);
}

/** Interpret a parameter value; this does not confirm the physical installation. */
export function radarInstallationFromZero(value){
  const zero=numeric(value);if(zero===null)return null;
  const wrapped=normalize(zero-90),relativeDeg=Number((wrapped>180?wrapped-360:wrapped).toFixed(8));
  const preset=RADAR_INSTALLATION_DIRECTIONS.find(item=>Math.abs(item.relativeDeg-relativeDeg)<1e-8);
  return {direction:preset?.key??'custom',relativeDeg};
}

/** Static corner heading to the lower reference; upper scans face 90deg further CCW. */
export function radarLowerReferenceFromWorld(start,value){
  const angle=numeric(value);return angle===null||!['upper','lower'].includes(start)?null:normalize(angle-(start==='upper'?90:0));
}

/** Interpret a lower-reference readback at a chosen start, without physical confirmation. */
export function radarWorldFromLowerReference(start,value){
  const angle=numeric(value);return angle===null||!['upper','lower'].includes(start)?null:normalize(angle+(start==='upper'?90:0));
}
