const kinds=new Set(['host_status','host_config','host_map','host_path','host_preview','host_cloud']);
const transferKinds=new Set(['host_config','host_map','host_path','host_preview','host_cloud']);
const u32=value=>Number.isInteger(value)&&value>=0&&value<=0xffffffff;
const identityFields=['session','config','scan','map','plan'];

const deepFreeze=value=>{
  if(value&&typeof value==='object'&&!Object.isFrozen(value)){
    for(const child of Object.values(value))deepFreeze(child);
    Object.freeze(value);
  }
  return value;
};

export function parseRadarHost(line){
  if(typeof line!=='string')return null;
  line=line.trim().replace(/^(?:arm>\s*)+/, '');
  if(!line.startsWith('@RADAR '))return null;
  try{
    const frame=JSON.parse(line.slice(7));
    if(frame?.v!==1||frame.profile!=='f1_host'||!kinds.has(frame.k)||!identityFields.every(key=>u32(frame[key])))return null;
    return frame;
  }catch{return null;}
}

function decodeBase64(text){
  if(typeof text!=='string'||!/^(?:[A-Za-z0-9+/]{4})*(?:[A-Za-z0-9+/]{2}==|[A-Za-z0-9+/]{3}=)?$/.test(text))throw new Error('invalid_base64');
  let raw;try{raw=atob(text);}catch{throw new Error('invalid_base64');}
  return Uint8Array.from(raw,character=>character.charCodeAt(0));
}

function crc32(bytes){
  let crc=0xffffffff;
  for(const byte of bytes){crc^=byte;for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}
  return (crc^0xffffffff)>>>0;
}

const sameIdentity=(left,right)=>identityFields.every(key=>left[key]===right[key]);
const primaryId=frame=>frame.k==='host_config'?frame.config:frame.k==='host_map'?frame.map:frame.k==='host_path'?frame.plan:frame.k==='host_preview'?frame.scan:frame.scan;
const number=(text,name,minimum=0,maximum=0xffffffff)=>{
  const value=Number(text);if(!Number.isInteger(value)||value<minimum||value>maximum)throw new Error(`invalid_${name}`);return value;
};
const mask=text=>{
  if(typeof text!=='string'||!/^[0-9A-Fa-f]{1,8}$/.test(text))throw new Error('invalid_mask');return parseInt(text,16)>>>0;
};

function commonSnapshot(frame,now){
  return {k:frame.k,...Object.fromEntries(identityFields.map(key=>[key,frame[key]])),source:'device',receivedAt:now,complete:true,historical:false};
}

function decodeConfig(ordered,frame,now){
  const result=commonSnapshot(frame,now);let seen=0;
  for(const page of ordered){
    if(page.offset!==seen++||typeof page.data!=='string')throw new Error('config_offset');
    const fields=page.data.split(','),key=fields.shift();
    if(key==='source'){if(!['active','draft'].includes(fields[0]))throw new Error('config_source');result.source=fields[0];}
    else if(['width','height','x1','x2','x3','x4','y1','y2','y3','y4','radar_x','radar_y','angle_min','angle_max','distance_min','distance_max','energy_min','energy_max','threshold'].includes(key))result[key]=number(fields[0],key);
    else if(key==='fixed')result.fixed_mask=mask(fields[0]);
    else if(key==='manual')result.manual_mask=mask(fields[0]);
    else if(key==='protected')result.protected_mask=mask(fields[0]);
    else if(key==='planning'){if(!['mission','point'].includes(fields[0]))throw new Error('planning_mode');result.planning_mode=fields[0];}
    else if(key==='mission')result.mission=fields.map(value=>number(value,'mission',1,5));
    else if(key==='start'||key==='goal')result[key]={row:number(fields[0],key,1,5),column:number(fields[1],key,1,5),anchor:number(fields[2],key,0,2)};
    else throw new Error('config_key');
  }
  if(seen!==frame.total)throw new Error('config_total');
  return result;
}

function decodeMap(ordered,frame,now){
  const result=commonSnapshot(frame,now),counts=Array(25);let seen=0,stats=false,masks=false,path=false;
  for(const page of ordered){
    if(page.offset!==seen++||typeof page.data!=='string')throw new Error('map_offset');
    const fields=page.data.split(','),type=fields.shift();
    if(type==='s'){result.raw_points=number(fields[0],'raw',0,65535);result.filtered_points=number(fields[1],'filtered',0,65535);result.outside_points=number(fields[2],'outside',0,65535);stats=true;}
    else if(type==='m'){result.scanned_mask=mask(fields[0]);result.manual_mask=mask(fields[1]);result.effective_mask=mask(fields[2]);result.mask=result.effective_mask;masks=true;}
    else if(type==='p'){result.path_valid=number(fields[0],'path_valid',0,1)===1;result.failed_from_task=number(fields[1],'failed_from',0,255);result.failed_to_task=number(fields[2],'failed_to',0,255);path=true;}
    else if(type==='c'){const index=number(fields[0],'cell',0,24);if(counts[index]!==undefined)throw new Error('duplicate_cell');counts[index]=number(fields[1],'count',0,65535);}
    else throw new Error('map_record');
  }
  if(seen!==frame.total||!stats||!masks||!path||Object.keys(counts).length!==25)throw new Error('map_incomplete');
  result.counts=counts;return result;
}

function decodePath(ordered,frame,now){
  const result=commonSnapshot(frame,now),points=[];
  if(frame.total===0){
    const fields=ordered[0]?.data?.split(',')??[];
    if(ordered.length!==1||ordered[0].offset!==0||fields.shift()!=='n')throw new Error('path_empty');
    result.path_valid=number(fields[0],'path_valid',0,1)===1;result.failed_from_task=number(fields[1],'failed_from',0,255);result.failed_to_task=number(fields[2],'failed_to',0,255);
  }else{
    for(let index=0;index<ordered.length;index++){
      const page=ordered[index],fields=page.data?.split(',')??[];
      if(page.offset!==index||fields.shift()!=='w')throw new Error('path_offset');
      points.push([number(fields[0],'x',0,65535),number(fields[1],'y',0,65535),number(fields[2],'distance',0,65535),number(fields[3],'row',1,5),number(fields[4],'column',1,5),number(fields[5],'direction',0,255),number(fields[6],'turn',0,255)]);
    }
    if(points.length!==frame.total)throw new Error('path_total');result.path_valid=true;
  }
  result.points=points;return result;
}

function decodePacked(ordered,frame,now,bytesPerPoint){
  const chunks=[];let pointOffset=0,byteLength=0,expectedCrc=null;
  for(const page of ordered){
    if(page.offset!==pointOffset||typeof page.data!=='string')throw new Error('packed_offset');
    if(typeof page.crc32!=='string'||!/^[0-9A-Fa-f]{8}$/.test(page.crc32))throw new Error('packed_crc');
    const crc=page.crc32.toUpperCase();if(expectedCrc!==null&&crc!==expectedCrc)throw new Error('mixed_crc');expectedCrc=crc;
    const bytes=decodeBase64(page.data);if(bytes.length%bytesPerPoint)throw new Error('partial_point');
    chunks.push(bytes);pointOffset+=bytes.length/bytesPerPoint;byteLength+=bytes.length;
  }
  if(pointOffset!==frame.total)throw new Error('packed_total');
  const bytes=new Uint8Array(byteLength);let at=0;for(const chunk of chunks){bytes.set(chunk,at);at+=chunk.length;}
  if(crc32(bytes).toString(16).padStart(8,'0').toUpperCase()!==expectedCrc)throw new Error('crc_mismatch');
  const result=commonSnapshot(frame,now),points=[];
  for(let offset=0;offset<bytes.length;offset+=bytesPerPoint){
    const angle=bytes[offset]|bytes[offset+1]<<8,distance=bytes[offset+2]|bytes[offset+3]<<8;
    points.push(bytesPerPoint===5?{angle_deg:angle/10,distance_mm:distance,energy:bytes[offset+4]}:{angle_deg:angle/10,distance_mm:distance});
  }
  result.crc32=expectedCrc;result.points=points;return result;
}

function decodeTransfer(ordered,frame,now){
  if(frame.k==='host_config')return decodeConfig(ordered,frame,now);
  if(frame.k==='host_map')return decodeMap(ordered,frame,now);
  if(frame.k==='host_path')return decodePath(ordered,frame,now);
  return decodePacked(ordered,frame,now,frame.k==='host_cloud'?5:4);
}

export class RadarHostModel{
  constructor(){this.snapshots={};this.pending=new Map();this.connected=false;this.status=null;this.session=null;this.lastError=null;this.lastEvent=null;}
  get pendingKinds(){return [...this.pending.keys()];}
  connection(value){
    const connected=typeof value==='object'?value?.state==='connected'||value?.connected===true:!!value;
    this.connected=connected;
    if(!connected){this.pending.clear();this.status=null;for(const [key,snapshot] of Object.entries(this.snapshots))this.snapshots[key]=deepFreeze({...snapshot,historical:true});}
    return this.connected;
  }
  accept(line,now=Date.now()){
    const frame=parseRadarHost(line);if(!frame)return false;this.lastError=null;
    if(this.session!==null&&this.session!==frame.session){this.pending.clear();for(const [key,snapshot] of Object.entries(this.snapshots))this.snapshots[key]=deepFreeze({...snapshot,historical:true});}
    this.session=frame.session;
    if(frame.k==='host_status'){
      this.status=deepFreeze({...frame,receivedAt:now,historical:!this.connected});this.lastEvent={accepted:true,frame,status:this.status};return true;
    }
    if(this.status?.session===frame.session&&['host_map','host_path','host_preview','host_cloud'].includes(frame.k)&&frame.scan<this.status.scan){
      this.lastError='stale_scan';this.lastEvent={accepted:true,frame,error:this.lastError};return true;
    }
    if(!transferKinds.has(frame.k)||!Number.isInteger(frame.page)||!Number.isInteger(frame.pages)||frame.pages<1||frame.pages>4096||frame.page<0||frame.page>=frame.pages||!u32(frame.offset)||!u32(frame.total)){
      this.lastError='invalid_page';this.lastEvent={accepted:true,frame,error:this.lastError};return true;
    }
    const identity={...Object.fromEntries(identityFields.map(key=>[key,frame[key]])),pages:frame.pages,total:frame.total,primary:primaryId(frame)};
    let transfer=this.pending.get(frame.k);
    if(transfer&&!sameIdentity(transfer.identity,identity)){
      const newer=frame.session!==transfer.identity.session||frame.scan>transfer.identity.scan||frame.page===0&&identity.primary!==transfer.identity.primary;
      if(!newer){this.lastError='mixed_identity';this.lastEvent={accepted:true,frame,error:this.lastError};return true;}
      this.pending.delete(frame.k);transfer=null;
    }
    if(!transfer){transfer={identity,pages:new Map()};this.pending.set(frame.k,transfer);}
    if(transfer.identity.pages!==frame.pages||transfer.identity.total!==frame.total){this.lastError='mixed_shape';this.lastEvent={accepted:true,frame,error:this.lastError};return true;}
    const prior=transfer.pages.get(frame.page),serialized=JSON.stringify(frame);
    if(prior&&prior.serialized!==serialized){this.lastError='duplicate_conflict';this.lastEvent={accepted:true,frame,error:this.lastError};return true;}
    transfer.pages.set(frame.page,{frame,serialized});
    if(transfer.pages.size!==frame.pages){this.lastEvent={accepted:true,frame,pending:true};return true;}
    const ordered=Array.from({length:frame.pages},(_,index)=>transfer.pages.get(index)?.frame);
    if(ordered.some(page=>!page)){this.lastEvent={accepted:true,frame,pending:true};return true;}
    try{
      const snapshot=deepFreeze(decodeTransfer(ordered,frame,now));
      const key=frame.k.slice(5);this.snapshots[key]=snapshot;this.pending.delete(frame.k);
      this.lastEvent={accepted:true,frame,snapshot,key};
    }catch(error){this.lastError=error.message;this.pending.delete(frame.k);this.lastEvent={accepted:true,frame,error:this.lastError};}
    return true;
  }
}
