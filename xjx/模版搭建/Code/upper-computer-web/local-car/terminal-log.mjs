import {parseGrabStatus} from './protocol.mjs?v=vision-pause-20260929';

/** Display-only reduction. Raw entries remain available to export and diagnostics. */
export function visibleTerminalLogs(entries,hideRepeated=true,chinese=false) {
  const sent=new Set();
  let previous=null;
  return entries.filter(entry=>{
    const text=entry.text.replace(/^arm>\s*/, '');
    if(entry.direction==='TX'){sent.add(entry.text.trim());previous=null;}
    if(chinese&&entry.direction==='RX'&&(!text.trim()||sent.has(text.trim())))return false;
    if(text==='STM32 mechanical arm console ready'){previous=null;sent.clear();}
    if(!hideRepeated)return true;
    if(entry.direction!=='RX'||!text.startsWith('OK grab state='))return true;
    const status=parseGrabStatus(text);
    if(!status)return true;
    const signature=JSON.stringify(['state','mode','recovery_used','reason','missing','stop_requested','stop_confirmed','model','protocol','vision','age_source','axis','step','home_flags','motor_flags'].map(key=>status[key]??null));
    if(signature===previous)return false;
    previous=signature;
    return true;
  });
}

/** At most one pending frame; hidden pages only accumulate raw data, not callbacks. */
export function createLogRenderScheduler(render,requestFrame,isHidden){
  let pending=false;
  return ()=>{
    if(pending||isHidden())return;
    pending=true;requestFrame(()=>{pending=false;if(!isHidden())render();});
  };
}
