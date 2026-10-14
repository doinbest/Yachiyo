const kinds=new Set(['host_status','host_result_end','host_bundle','host_bundle_end','host_config','host_map','host_path','host_preview','host_cloud','host_stable']);
const transferKinds=new Set(['host_bundle','host_config','host_map','host_path','host_preview','host_cloud','host_stable']);
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
  // Actual wireless loss can join two JSON bodies; JSON.parse silently keeps
  // the last duplicate key and would misclassify that splice as a valid page.
  if((line.match(/"k"\s*:/g)??[]).length!==1)return null;
  try{
    const frame=JSON.parse(line.slice(7));
    if(frame?.v===2&&['r','e'].includes(frame.k)&&Array.isArray(frame.i)&&frame.i.length===5&&frame.i.every(u32)){
      if(frame.profile!==undefined&&!['f1_host','competition'].includes(frame.profile))return null;
      return {v:1,profile:frame.profile??'f1_host',k:frame.k==='r'?'host_bundle':'host_bundle_end',
        ...Object.fromEntries(identityFields.map((key,index)=>[key,frame.i[index]])),
        page:frame.p,pages:frame.n,offset:frame.p*(frame.profile?72:84),chunkSize:frame.profile?72:84,total:frame.t,data:frame.d,live:frame.l,wire:frame};
    }
    if(frame?.v!==1||!['f1_host','competition'].includes(frame.profile)||!kinds.has(frame.k)||!identityFields.every(key=>u32(frame[key])))return null;
    if(frame.k==='host_preview'&&(!u32(frame.live_id)||!frame.live_id))return null;
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

const sameIdentity=(left,right)=>left.primary===right.primary&&identityFields.every(key=>left[key]===right[key]);
const primaryId=frame=>frame.k==='host_config'?frame.config:frame.k==='host_map'?frame.map:frame.k==='host_path'?frame.plan:frame.k==='host_preview'?frame.live_id:frame.scan;
const newerPreview=(next,previous)=>{const delta=(next-previous)>>>0;return delta>0&&delta<0x80000000;};
const number=(text,name,minimum=0,maximum=0xffffffff)=>{
  const value=Number(text);if(!Number.isInteger(value)||value<minimum||value>maximum)throw new Error(`invalid_${name}`);return value;
};
const mask=text=>{
  if(typeof text!=='string'||!/^[0-9A-Fa-f]{1,8}$/.test(text))throw new Error('invalid_mask');return parseInt(text,16)>>>0;
};

function commonSnapshot(frame,now){
  return {k:frame.k,profile:frame.profile,...Object.fromEntries(identityFields.map(key=>[key,frame[key]])),
    ...(frame.k==='host_preview'?{live_id:frame.live_id}:{}),source:'device',receivedAt:now,complete:true,historical:false};
}

function decodeConfig(ordered,frame,now){
  const result=commonSnapshot(frame,now);let seen=0;
  for(const page of ordered){
    if(page.offset!==seen++||typeof page.data!=='string')throw new Error('config_offset');
    const fields=page.data.split(','),key=fields.shift();
    if(key==='source'){if(!['active','draft','capture'].includes(fields[0]))throw new Error('config_source');result.source=fields[0];}
    else if(['width','height','x1','x2','x3','x4','y1','y2','y3','y4','radar_x','radar_y','angle_min','angle_max','distance_min','distance_max','energy_min','energy_max','threshold','situation','grid_version','zero_tenths'].includes(key))result[key]=number(fields[0],key);
    else if(['m00_q16','m01_q16','m10_q16','m11_q16','tx_q16','ty_q16'].includes(key))result[key]=number(fields[0],key,-2147483648,2147483647);
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

function unpackRecords(ordered,error){
  const records=[];
  for(const page of ordered){
    if(page.offset!==records.length||typeof page.data!=='string')throw new Error(error);
    for(const data of page.data.split(';'))records.push({offset:records.length,data});
  }
  return records;
}

function decodeMap(ordered,frame,now){
  const result=commonSnapshot(frame,now),counts=Array(25);let seen=0,stats=false,masks=false,path=false;
  for(const page of unpackRecords(ordered,'map_offset')){
    if(page.offset!==seen++||typeof page.data!=='string')throw new Error('map_offset');
    const fields=page.data.split(','),type=fields.shift();
    if(type==='s'){result.raw_points=number(fields[0],'raw');result.filtered_points=number(fields[1],'filtered');result.outside_points=number(fields[2],'outside');stats=true;}
    else if(['r','f','o'].includes(type)){result[{r:'raw_points',f:'filtered_points',o:'outside_points'}[type]]=number(fields[0],type);stats=['raw_points','filtered_points','outside_points'].every(k=>result[k]!==undefined);}
    else if(type==='m'){result.scanned_mask=mask(fields[0]);result.manual_mask=mask(fields[1]);result.effective_mask=mask(fields[2]);result.mask=result.effective_mask;masks=true;}
    else if(type==='p'){result.path_valid=number(fields[0],'path_valid',0,1)===1;result.failed_from_task=number(fields[1],'failed_from',0,255);result.failed_to_task=number(fields[2],'failed_to',0,255);path=true;}
    else if(type==='c'){const index=number(fields[0],'cell',0,24);if(counts[index]!==undefined)throw new Error('duplicate_cell');counts[index]=number(fields[1],'count',0,65535);}
    else throw new Error('map_record');
  }
  if(seen!==frame.total||!stats||!masks||!path||Object.keys(counts).length!==25)throw new Error('map_incomplete');
  result.counts=counts;return result;
}

function decodeWaypoint(data){
  const fields=data.split(',');
  if(fields.length!==8||fields.shift()!=='w')throw new Error('path_record');
  return [number(fields[0],'x',0,65535),number(fields[1],'y',0,65535),number(fields[2],'distance',0,65535),number(fields[3],'row',1,5),number(fields[4],'column',1,5),number(fields[5],'direction',0,255),number(fields[6],'turn',0,255)];
}

function validateResultPage(frame){
  if(typeof frame.data!=='string')throw new Error('invalid_data');
  if(frame.crc32!==undefined){
    if(typeof frame.crc32!=='string'||!/^[0-9A-Fa-f]{8}$/.test(frame.crc32))throw new Error('page_crc');
    const protectedFrame={v:frame.v,profile:frame.profile,k:frame.k,
      ...Object.fromEntries([...identityFields,'page','pages','offset','total','data'].map(key=>[key,frame[key]]))};
    if(crc32(new TextEncoder().encode(JSON.stringify(protectedFrame)))!==parseInt(frame.crc32,16))throw new Error('page_crc_mismatch');
  }
  // Older firmware has no text CRC. Reject a broken waypoint before it can
  // poison the pending circle and discard all its otherwise valid pages.
  if(frame.k==='host_path'){
    if(frame.total===0)decodePath([frame],frame,0);
    else for(const data of frame.data.split(';'))decodeWaypoint(data);
  }
}

function decodePath(ordered,frame,now){
  const result=commonSnapshot(frame,now),points=[];
  if(frame.total===0){
    const fields=ordered[0]?.data?.split(',')??[];
    if(ordered.length!==1||ordered[0].offset!==0||fields.shift()!=='n')throw new Error('path_empty');
    result.path_valid=number(fields[0],'path_valid',0,1)===1;result.failed_from_task=number(fields[1],'failed_from',0,255);result.failed_to_task=number(fields[2],'failed_to',0,255);
  }else{
    const records=unpackRecords(ordered,'path_offset');
    for(let index=0;index<records.length;index++){
      const page=records[index];
      if(page.offset!==index)throw new Error('path_offset');
      points.push(decodeWaypoint(page.data));
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
    points.push(bytesPerPoint===10?{angle_deg:angle/10,distance_mm:distance,x_mm:bytes[offset+4]|bytes[offset+5]<<8,y_mm:bytes[offset+6]|bytes[offset+7]<<8,votes:bytes[offset+8],flags:bytes[offset+9]}:bytesPerPoint===5?{angle_deg:angle/10,distance_mm:distance,energy:bytes[offset+4]}:{angle_deg:angle/10,distance_mm:distance});
  }
  result.crc32=expectedCrc;result.points=points;if(frame.k==='host_stable')result.stable_id=frame.stable_id??frame.scan;return result;
}

function decodeTransfer(ordered,frame,now){
  if(frame.k==='host_bundle')return decodeBundle(ordered,frame,now);
  if(frame.k==='host_config')return decodeConfig(ordered,frame,now);
  if(frame.k==='host_map')return decodeMap(ordered,frame,now);
  if(frame.k==='host_path')return decodePath(ordered,frame,now);
  return decodePacked(ordered,frame,now,frame.k==='host_stable'?10:frame.k==='host_cloud'?5:4);
}

function validateBundle(frame){
  const {c,...protectedFrame}=frame.wire;
  if(typeof c!=='string'||!/^[0-9A-Fa-f]{8}$/.test(c)||crc32(new TextEncoder().encode(JSON.stringify(protectedFrame)))!==parseInt(c,16))throw new Error('bundle_page_crc');
  if(frame.live!==undefined&&frame.live!==0&&frame.live!==1)throw new Error('bundle_mode');
  if(frame.k==='host_bundle_end'){
    if(!Number.isInteger(frame.pages)||frame.pages<1||frame.pages>30)throw new Error('bundle_pages');
    return;
  }
  const bytes=decodeBase64(frame.data);
  const chunkSize=frame.chunkSize??84;
  if(!u32(frame.total)||frame.total<74||frame.total>2078||frame.pages!==Math.ceil(frame.total/chunkSize)||!Number.isInteger(frame.page)||frame.page<0||frame.page>=frame.pages||bytes.length!==Math.min(chunkSize,frame.total-frame.offset))throw new Error('bundle_shape');
}
function decodeBundle(ordered,frame,now){
  const bytes=new Uint8Array(frame.total);
  for(const page of ordered)bytes.set(decodeBase64(page.data),page.offset);
  const view=new DataView(bytes.buffer),u16=at=>view.getUint16(at,true),u32=at=>view.getUint32(at,true);
  if(bytes[0]===2){
    const s32=at=>view.getInt32(at,true),s16=at=>view.getInt16(at,true);
    if(bytes.length<158||bytes[25]>1||158+10*u16(28)!==bytes.length)throw new Error('bundle_length');
    const map={...commonSnapshot({...frame,k:'host_map'},now),layout:2,raw_points:u32(1),filtered_points:u32(5),outside_points:u32(9),
      scanned_mask:u32(13),manual_mask:u32(17),effective_mask:u32(21),mask:u32(21),path_valid:!!bytes[25],failed_from_task:bytes[26],failed_to_task:bytes[27],
      counts:Array.from({length:25},(_,i)=>u16(30+2*i)),stable_points:u16(80),window_count:bytes[82],situation:bytes[83],grid_version:bytes[84],algorithm:bytes[85],candidate_count:u16(86),candidate_replacements:u32(88),candidate_dropped:u32(92),window_first:u32(96),window_last:u32(100),
      geometry:{matrix_q16:Array.from({length:4},(_,i)=>s32(104+4*i)),origin_q16:[s32(120),s32(124)],zero_tenths:u16(128)},
      grid:{x:Array.from({length:6},(_,i)=>s16(130+2*i)),y:Array.from({length:6},(_,i)=>s16(142+2*i))},pose:[u16(154),u16(156),u16(128)/10]};
    const points=[];for(let at=158;at<bytes.length;at+=10){if(bytes[at+6]<1||bytes[at+6]>5||bytes[at+7]<1||bytes[at+7]>5)throw new Error('bundle_waypoint');points.push([u16(at),u16(at+2),u16(at+4),...bytes.slice(at+6,at+10)]);}
    return {map,path:{...commonSnapshot({...frame,k:'host_path'},now),path_valid:map.path_valid,failed_from_task:map.failed_from_task,failed_to_task:map.failed_to_task,points}};
  }
  if(bytes[0]!==1||bytes[19]>1||74+10*u16(22)!==bytes.length)throw new Error('bundle_length');
  const map={...commonSnapshot({...frame,k:'host_map'},now),raw_points:u16(1),filtered_points:u16(3),outside_points:u16(5),
    scanned_mask:u32(7),manual_mask:u32(11),effective_mask:u32(15),mask:u32(15),path_valid:!!bytes[19],failed_from_task:bytes[20],failed_to_task:bytes[21],counts:Array.from({length:25},(_,i)=>u16(24+2*i))};
  const points=[];
  for(let at=74;at<bytes.length;at+=10){
    if(bytes[at+6]<1||bytes[at+6]>5||bytes[at+7]<1||bytes[at+7]>5)throw new Error('bundle_waypoint');
    points.push([u16(at),u16(at+2),u16(at+4),...bytes.slice(at+6,at+10)]);
  }
  const path={...commonSnapshot({...frame,k:'host_path'},now),path_valid:map.path_valid,failed_from_task:map.failed_from_task,failed_to_task:map.failed_to_task,points};
  return {map,path};
}

export class RadarHostModel{
  constructor(){this.snapshots={};this.pending=new Map();this.connected=false;this.status=null;this.session=null;this.lastError=null;this.lastEvent=null;this.resultIdentity=null;}
  get pendingKinds(){return [...this.pending.keys()];}
  missingPages(kind,identity,pages){
    const matches=value=>value&&identityFields.every(key=>value[key]===identity[key]);
    if(kind==='bundle'&&matches(this.snapshots.map)&&matches(this.snapshots.path)&&!this.snapshots.map.historical&&!this.snapshots.path.historical)return [];
    if(matches(this.snapshots[kind])&&!this.snapshots[kind].historical)return [];
    const transfer=this.pending.get(`host_${kind}`);
    const received=matches(transfer?.identity)?transfer.pages:new Map();
    return Array.from({length:pages},(_,page)=>page).filter(page=>!received.has(page));
  }
  connection(value){
    const connected=typeof value==='object'?value?.state==='connected'||value?.connected===true:!!value;
    this.connected=connected;
    if(!connected){this.pending.clear();this.status=null;this.resultIdentity=null;for(const [key,snapshot] of Object.entries(this.snapshots))this.snapshots[key]=deepFreeze({...snapshot,historical:true});}
    return this.connected;
  }
  accept(line,now=Date.now()){
    const frame=parseRadarHost(line);if(!frame)return false;this.lastError=null;
    if(['host_bundle','host_bundle_end'].includes(frame.k)){
      try{validateBundle(frame);}catch(error){this.lastError=error.message;this.lastEvent={accepted:true,frame,error:this.lastError,recoverable:true};return true;}
    }
    // Validate before trusting even the identity: a spliced wireless header
    // must not reset this session or displace already verified pages.
    if(['host_map','host_path'].includes(frame.k)){
      try{validateResultPage(frame);}
      catch(error){this.lastError=error.message;this.lastEvent={accepted:true,frame,error:this.lastError,recoverable:true};return true;}
    }
    if(this.session!==null&&this.session!==frame.session){this.pending.clear();for(const [key,snapshot] of Object.entries(this.snapshots))this.snapshots[key]=deepFreeze({...snapshot,historical:true});}
    this.session=frame.session;
    if(frame.k==='host_bundle_end'){this.lastEvent={accepted:true,frame};return true;}
    if(frame.k==='host_status'){
      this.status=deepFreeze({...frame,receivedAt:now,historical:!this.connected});this.lastEvent={accepted:true,frame,status:this.status};return true;
    }
    if(frame.k==='host_result_end'){
      const valid=['mp','pp'].every(key=>Number.isInteger(frame[key])&&frame[key]>=1&&frame[key]<=4096);
      this.lastError=valid?null:'invalid_result_end';
      this.lastEvent={accepted:true,frame,...(valid?{}:{error:this.lastError})};return true;
    }
    const pinned=['host_bundle','host_map','host_path'].includes(frame.k)&&this.resultIdentity&&identityFields.every(key=>this.resultIdentity[key]===frame[key]);
    if(!pinned&&this.status?.session===frame.session&&['host_bundle','host_map','host_path','host_cloud'].includes(frame.k)&&frame.scan<this.status.scan){
      this.lastError='stale_scan';this.lastEvent={accepted:true,frame,error:this.lastError};return true;
    }
    if(!transferKinds.has(frame.k)||!Number.isInteger(frame.page)||!Number.isInteger(frame.pages)||frame.pages<1||frame.pages>4096||frame.page<0||frame.page>=frame.pages||!u32(frame.offset)||!u32(frame.total)){
      this.lastError='invalid_page';this.lastEvent={accepted:true,frame,error:this.lastError};return true;
    }
    const identity={...Object.fromEntries(identityFields.map(key=>[key,frame[key]])),pages:frame.pages,total:frame.total,primary:primaryId(frame)};
    let transfer=this.pending.get(frame.k);
    if(frame.k==='host_preview'){
      const complete=this.snapshots.preview;
      const previous=transfer?.identity.primary??(complete?.session===frame.session&&!complete.historical?complete.live_id:null);
      // Late pages do not displace a newer pending/complete circle. Reconnection
      // marks the old snapshot historical so a restarted MCU can publish afresh.
      if(previous!=null&&frame.live_id!==previous&&!newerPreview(frame.live_id,previous)){
        this.lastEvent={accepted:true,frame,ignored:true};return true;
      }
    }
    if(transfer&&!sameIdentity(transfer.identity,identity)){
      const newer=frame.session!==transfer.identity.session||(frame.k==='host_preview'?newerPreview(identity.primary,transfer.identity.primary):
        frame.scan>transfer.identity.scan||frame.page===0&&identity.primary!==transfer.identity.primary);
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
      if(frame.k==='host_bundle'){
        // Publish the pair in one receive event: never show a new map with an old path.
        this.snapshots.map=snapshot.map;this.snapshots.path=snapshot.path;this.pending.delete(frame.k);
        this.lastEvent={accepted:true,frame,snapshot,key:'result'};return true;
      }
      const key=frame.k.slice(5);this.snapshots[key]=snapshot;
      if(key==='config'&&snapshot.source==='active')this.snapshots.active_config=snapshot;
      if(key==='config'&&snapshot.source==='capture')this.snapshots.capture_config=snapshot;
      this.pending.delete(frame.k);
      this.lastEvent={accepted:true,frame,snapshot,key};
    }catch(error){this.lastError=error.message;this.pending.delete(frame.k);this.lastEvent={accepted:true,frame,error:this.lastError};}
    return true;
  }
}
