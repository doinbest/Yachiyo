export const ROUTE_SPEED_VERSION=1;
export const DEFAULT_ROUTE_SPEED=200;
const validSpeed=value=>Number.isInteger(value)&&value>=10&&value<=5000;

/** Migrate browser startup preferences once; offline editor rows are untouched. */
export function migrateRoutePreferences(source={}){
  const prefs={...source},old=Number(prefs.routeSpeed);
  prefs.routeSpeed=validSpeed(old)?old:DEFAULT_ROUTE_SPEED;
  if(prefs.routeSpeedVersion!==ROUTE_SPEED_VERSION){
    if(validSpeed(old)&&old>DEFAULT_ROUTE_SPEED){prefs.previousRouteSpeed=old;prefs.routeSpeed=DEFAULT_ROUTE_SPEED;}
    prefs.routeSpeedVersion=ROUTE_SPEED_VERSION;
  }
  return prefs;
}

/** Local editing affects the next explicit start only; this helper has no TX API. */
export function bindRouteSpeed({prefs,inputs,save=()=>{},onChange=()=>{}}){
  function render(){for(const input of inputs){input.value=String(prefs.routeSpeed);input.dispatchEvent?.(new Event('input'));}}
  function set(speed){
    speed=Number(speed);if(!validSpeed(speed))throw new RangeError('速度范围10～5000 mm/s，须为整数');
    prefs.routeSpeed=speed;render();save();onChange(speed);
  }
  for(const input of inputs)input.addEventListener('change',()=>{const speed=Number(input.value);if(validSpeed(speed))set(speed);});
  render();return {set,get value(){return prefs.routeSpeed;}};
}
