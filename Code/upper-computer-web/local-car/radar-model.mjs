import {frameCommand} from './protocol.mjs';
import {radarLowerReferenceFromWorld,radarSnapshotFrame,referencePointToCurrent,referencePoseToCurrent,referenceGeometryToCurrent,referenceInstallationToCurrent} from './radar-installation.mjs';
export const INITIAL_VIEW=Object.freeze({x:-320,y:-370,width:3170,height:2990});
export const RADAR_PARAMETERS=[
  ['lidar_x_mm','下方参考雷达 X / mm',2280,0,2400],['lidar_y_mm','下方参考雷达 Y / mm',314,0,2400],
  ['zero_deg','起点雷达零角 / °',180,-360,360],['distance_min_mm','距离下限 / mm',100,0,65535],
  ['distance_max_mm','距离上限 / mm',40000,0,65535],['angle_min_tenths','角度下限 / 0.1°',0,0,3600],
  ['angle_max_tenths','顺时针角度上限 / 0.1°',3600,0,3600],['threshold','每格判障最少点数',1,1,65535],
  ['energy_min','能量下限',0,0,255],['energy_max','能量上限',255,0,255],
  ['situation','安装情况',2,0,2],['grid_version','判障网格版本',4,4,4],['zero_tenths','冻结零角 / 0.1°',1800,0,3599],
  ['m00_q16','仿射 M00 / Q16',65809,-262144,262144],['m01_q16','仿射 M01 / Q16',680,-262144,262144],
  ['m10_q16','仿射 M10 / Q16',-407,-262144,262144],['m11_q16','仿射 M11 / Q16',66623,-262144,262144],
  ['tx_q16','仿射原点 X / Q16',149407997,-2147483648,2147483647],['ty_q16','仿射原点 Y / Q16',20569846,-2147483648,2147483647],
  ['upper_lidar_x_mm','右上雷达 X / mm',2050,0,2400],['upper_lidar_y_mm','右上雷达 Y / mm',2250,0,2400]
].map(([key,label,value,min,max])=>({key,label,value,min,max}));
export const STATIONS={1:'启停区',2:'粗加工区',3:'暂存区',4:'原料区',5:'扫码区'};
export const RADAR_GRID=Object.freeze([0,550,1000,1400,1850,2400]);
export const RADAR_FIXED_MASK=0x00050140;
export {radarStateText,radarReasonText} from './radar-format.mjs';
/** Full raw measurements only; transport-complete display samples are still partial. */
export function isCompleteRadarCloud(cloud){
  return Array.isArray(cloud?.points)&&cloud.k!=='points'&&!cloud.truncated&&cloud.complete!==false&&(cloud.complete===true||cloud.cloud_complete===true);
}
/** Prefer the complete raw cloud of the latest capture, preserving newer capture previews. */
export function preferredRadarCloud(...snapshots){
  const available=snapshots.filter(snapshot=>Array.isArray(snapshot?.points)).sort((a,b)=>(b.receivedAt??0)-(a.receivedAt??0)),latest=available[0];
  return available.find(snapshot=>isCompleteRadarCloud(snapshot)&&['session','origin','scan','map'].every(key=>snapshot[key]===latest[key]))??latest;
}
const u32=x=>Number.isInteger(x)&&x>=0&&x<=0xffffffff;
export function parseRadar(line){
  if(typeof line!=='string')return null;
  // A complete atomic radar line can follow a partially echoed CLI command.
  // Recover at its marker; JSON and identity validation still reject damaged pages.
  const marker=line.indexOf('@RADAR ');if(marker<0)return null;
  line=line.slice(marker).trim();
  try{let f=JSON.parse(line.slice(7));
    if(['map','path'].includes(f?.k)&&f.crc32!==undefined){const protectedBody=line.slice(7).replace(/,\s*"crc32"\s*:\s*"[0-9A-Fa-f]{8}"\s*}\s*$/,'}');if(typeof f.crc32!=='string'||rawCrc(Array.from(new TextEncoder().encode(protectedBody),b=>b.toString(16).padStart(2,'0')).join(''))!==f.crc32.toUpperCase())return null;}
    if(f?.v===1&&Array.isArray(f.i)&&f.i.length===5&&f.i.every(u32))f={...f,...Object.fromEntries(['session','origin','scan','map','plan'].map((key,i)=>[key,f.i[i]]))};return f?.v===1&&typeof f.k==='string'&&['session','origin','scan','map','plan'].every(k=>u32(f[k]))?f:null;
  }catch{return null;}
}
const idFor=(kind,f)=>kind==='raw'?f.id:kind==='path'?f.plan:kind==='params'?0:['points','cloud','stable'].includes(kind)?f.scan:f.map;
function rawCrc(hex){let crc=0xffffffff;for(let i=0;i<hex.length;i+=2){crc^=parseInt(hex.slice(i,i+2),16);for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}return ((crc^0xffffffff)>>>0).toString(16).padStart(8,'0').toUpperCase();}
function freezeCloudGeometry(cloud,map){
  if(!cloud||!map||!['session','origin','scan','map'].every(k=>cloud[k]===map[k]))return;
  if(Array.isArray(map.pose))cloud.pose=[...map.pose];
  // Either download order must retain the calibration belonging to this cloud.
  for(const key of ['geometry','affine','matrix_q16','origin_q16','zero_tenths'])
    if(Object.hasOwn(map,key))cloud[key]=structuredClone(map[key]);
}
export class RadarModel{
  snapshots={};pending=new Map();connected=false;status=null;nav=null;probe=null;rawStatus=null;params={};paramsLive=false;identity=null;
  clearResults(){for(const kind of ['map','path','points','cloud','stable','start'])delete this.snapshots[kind];this.pending.clear();}
  connection(value){if(!value||!this.connected){for(const snapshot of Object.values(this.snapshots))snapshot.historical=true;if(this.probe)this.probe.historical=true;}this.connected=value;if(!value){this.pending.clear();this.status=null;this.nav=null;this.paramsLive=false;}}
  accept(line,now=Date.now()){
    const f=parseRadar(line);if(!f)return false;
    if(this.identity&&this.identity.session!==f.session){this.pending.clear();if(this.probe)this.probe.historical=true;}
    this.identity={session:f.session,origin:f.origin};
    if(f.k==='status'||f.k==='diag'||f.k==='progress'){this.status={...this.status,...f,receivedAt:now};return true;}
    if(f.k==='nav'){this.nav={...f,receivedAt:now};return true;}
    if(f.k==='probe'){
      if(!u32(f.id)||f.pages!==3||!Number.isInteger(f.page)||f.page<0||f.page>2)return true;
      const same=this.probe?.id===f.id&&this.probe?.session===f.session;
      if(!same&&f.page!==0)return true;
      this.probe={...(same?this.probe:{}),...f,receivedAt:now,historical:!this.connected};return true;
    }
    if(!['map','path','points','cloud','stable','raw','params','start'].includes(f.k)||!Number.isInteger(f.pages)||f.pages<1||f.pages>4096||!Number.isInteger(f.page)||f.page<0||f.page>=f.pages)return true;
    if(f.k==='raw'){if(!u32(f.id)||!f.id)return true;if(f.page===0)this.rawStatus={...f,receivedAt:now};}
    if(f.k==='start'&&(!u32(f.id)||f.id!==f.scan))return true;
    const identity=`${f.session}:${f.origin}:${f.scan}:${f.map}:${f.plan}:${f.pages}${f.k==='raw'?':'+f.id:''}`;
    const key=f.k+':'+identity;
    let pages=this.pending.get(key);if(!pages){for(const k of this.pending.keys())if(k.startsWith(f.k+':'))this.pending.delete(k);pages=new Map();this.pending.set(key,pages);}
    pages.set(f.page,{...f,_receivedAt:now});
    if(pages.size!==f.pages)return true;
    const ordered=Array.from({length:f.pages},(_,i)=>pages.get(i));
    if(ordered.some(p=>!p))return true;
    const complete={...ordered[0],source:'device',receivedAt:now,coordinate_frame:'current',complete:true,historical:false};
    delete complete._receivedAt;
    if(Number.isFinite(complete.age_ms))complete.age_ms+=now-ordered[0]._receivedAt;
    if(f.k==='raw'){
      if(f.pages<5)return true;
      for(const p of ordered.slice(1,5))for(const field of ['pose','unit','start','auto','points','filter','point_total','window','circle','cloud_truncated','received','losses','bad','restarts','cloud_valid'])if(Object.hasOwn(p,field))complete[field]=p[field];
      let hex='';
      for(const p of ordered.slice(5)){if(!Number.isInteger(p.offset)||p.offset!==hex.length/2||p.total!==complete.total||typeof p.hex!=='string'||!/^(?:[0-9a-fA-F]{2})*$/.test(p.hex))return true;hex+=p.hex;}
      if(!u32(complete.total)||hex.length/2!==complete.total||typeof complete.crc32!=='string'||rawCrc(hex)!==complete.crc32.toUpperCase())return true;
      if(!Array.isArray(complete.pose)||complete.pose.length!==3||!complete.pose.every(Number.isFinite)||!Array.isArray(complete.filter)||complete.filter.length!==7||!complete.filter.every(Number.isFinite))return true;
      complete.complete=ordered[0].complete===true;
      if(complete.complete&&(!Array.isArray(complete.circle)||complete.circle.length!==2||!complete.circle.every(u32)||!u32(complete.window)||complete.circle[0]<complete.window||complete.circle[1]>complete.window+complete.total||complete.circle[1]<=complete.circle[0]))return true;
      complete.complete=ordered[0].complete===true;complete.download_complete=true;complete.hex=hex.toUpperCase();
    }
    else if(f.k==='start'){
      if(typeof complete.auto!=='boolean'||![0,1,2].includes(complete.selected)||typeof complete.reason!=='string')return true;
      for(const field of ['qr','raw','conflicts']){
        const scores=ordered.find(p=>Object.hasOwn(p,field))?.[field];
        if(!Array.isArray(scores)||scores.length!==2||!scores.every(Number.isFinite))return true;
        complete[field]=[...scores];
      }
    }
    else if(f.k==='params'){if(ordered.some(p=>p.rev!==ordered[0].rev))return true;for(const p of ordered)if(typeof p.key==='string'&&Number.isFinite(p.value))this.params[p.key]=p.value;complete.params={...this.params};this.paramsLive=true;}
    else if(f.k==='stable'){
      let bytes=[];for(const p of ordered){if(p.offset!==bytes.length/10||typeof p.data!=='string'||p.crc32!==ordered[0].crc32)return true;let raw;try{raw=atob(p.data);}catch{return true;}if(raw.length%10)return true;bytes.push(...Array.from(raw,c=>c.charCodeAt(0)));}
      if(bytes.length/10!==f.total||rawCrc(bytes.map(b=>b.toString(16).padStart(2,'0')).join(''))!==String(f.crc32).toUpperCase())return true;
      complete.points=[];for(let i=0;i<bytes.length;i+=10)complete.points.push([bytes[i]|bytes[i+1]<<8,bytes[i+2]|bytes[i+3]<<8,bytes[i+4]|bytes[i+5]<<8,bytes[i+6]|bytes[i+7]<<8,bytes[i+8],bytes[i+9]]);
    }
    else{
      const field=f.k==='map'?'counts':'points',items=[];
      for(const p of ordered){
        if(f.k==='map'&&!('counts' in p))continue;
        if(!Array.isArray(p[field])||!Number.isInteger(p.offset)||p.offset!==items.length)return true;
        if(p[field].some(v=>field==='counts'?!Number.isFinite(v):!Array.isArray(v)||!v.every(Number.isFinite)))return true;
        items.push(...p[field]);
      }
      complete[field]=items;
      if(f.k==='map')for(const key of ['a','f','c','geometry','affine','matrix_q16','origin_q16','zero_tenths','window','stats','stable','grid','start_id','window_count','window_first','window_last','algorithm','situation','stable_points','grid_version','candidate_count','candidate_replacements','candidate_dropped']){const metadata=ordered.find(p=>Object.hasOwn(p,key));if(metadata)complete[key]=metadata[key];}
      if(f.k==='map'&&('mask' in complete||Array.isArray(complete.pose)||f.pages===6)&&items.length!==25)return true;
      if(['points','cloud'].includes(f.k)&&Number.isInteger(f.total)&&items.length!==f.total)return true;
      if(f.k==='points'||f.k==='cloud'){
        const map=this.snapshots.map;
        if(map&&['session','origin','scan','map'].every(k=>map[k]===f[k])){
          freezeCloudGeometry(complete,map);complete.age_ms=(map.age_ms??0)+now-map.receivedAt;
        }
      }
    }
    this.snapshots[f.k]=complete;
    if(f.k==='map')for(const kind of ['points','cloud'])freezeCloudGeometry(this.snapshots[kind],complete);
    this.pending.delete(key);return true;
  }
  describe(snapshot,now=Date.now()){
    if(!snapshot)return '尚无完整结果';
    const cloud=['points','cloud'].includes(snapshot.k);
    const historical=snapshot.historical===true||snapshot.source!=='device'||!this.connected||this.status&&(['session','origin','map',...(cloud||snapshot.k==='start'?['scan']:[]),...(snapshot.k==='path'?['plan']:[])].some(k=>this.status[k]!==snapshot[k])||cloud&&!this.status.points_valid||snapshot.k!=='start'&&(this.status.applicable===false||this.status.applicable===0));
    return `${historical?'历史 · ':''}${snapshot.source==='device'?'F407设备':'离线副本'} · 地图 ${snapshot.map??'—'} / 路径 ${snapshot.plan??'—'} · 结果年龄 ${Math.max(0,Math.floor(((snapshot.age_ms??0)+now-snapshot.receivedAt)/1000))} 秒`;
  }
}
export class RadarExchange{
  constructor({send,intervalMs=0,timeoutMs=10000,onProgress=()=>{}}){Object.assign(this,{send,intervalMs,timeoutMs,onProgress});this.pending=null;this.busy=false;this.revision=0;this.listeners=new Set();}
  subscribeState(listener){this.listeners.add(listener);return ()=>this.listeners.delete(listener);}
  notifyState(type){for(const listener of this.listeners)listener({type,busy:this.busy});}
  cancel(reason='读取已取消，保留已完成快照'){this.revision++;if(this.pending){clearTimeout(this.pending.timer);this.pending.reject(new Error(reason));this.pending=null;}this.notifyState('cancel');}
  receive(line){
    line=line.trim().replace(/^(?:arm>\s*)+/,'');
    const f=parseRadar(line),p=this.pending;
    // Another attached page can request the same packet. The matching packet
    // already in flight remains authoritative; do not add a retry delay.
    if(p?.pageRead&&line==='ERR radar page_pending')return true;
    if(p&&/^ERR radar\b/.test(line)){clearTimeout(p.timer);this.pending=null;p.reject(new Error(line));return true;}
    if(p?.matchLine&&p.matchLine(line)){clearTimeout(p.timer);this.pending=null;p.resolve({accepted:true});return true;}
    if(!f||!p||!p.match(f))return false;
    clearTimeout(p.timer);this.pending=null;p.resolve(f);return true;
  }
  async run(work){if(this.busy)throw new Error('上一项雷达读取仍在进行');this.busy=true;this.notifyState('busy');const revision=this.revision;try{return await work(revision);}finally{this.busy=false;this.notifyState('idle');}}
  read(wire,match,revision,matchLine){
    frameCommand(wire);if(revision!==this.revision)return Promise.reject(new Error('读取已取消'));
    const reply=new Promise((resolve,reject)=>{
      const item={match,matchLine,resolve,reject,pageRead:wire==='radar get'||wire.startsWith('radar fetch ')||wire.startsWith('radar result '),timer:setTimeout(()=>{if(this.pending===item){this.pending=null;reject(new Error('雷达页读回超时，已完成快照仍保留'));}},this.timeoutMs)};this.pending=item;
    });
    const writing=Promise.resolve().then(()=>this.send(wire)).then(ok=>{if(ok!==true)throw new Error('串口写出未确认');if(revision!==this.revision)throw new Error('读取已取消');}).catch(error=>{if(this.pending){clearTimeout(this.pending.timer);this.pending.reject(error);this.pending=null;}throw error;});
    return Promise.all([reply,writing]).then(([result])=>result);
  }
  async pages(kind,id,revision,first=null){
    const result=[];let total=first?.pages??1;
    for(let page=0;page<total;page++){
      if(revision!==this.revision)throw new Error('读取已取消');
      const f=page===0&&first?first:await this.read(`radar fetch ${kind} ${id} ${page}`,f=>f.k===kind&&idFor(kind,f)===id&&f.page===page,revision);
      if(page&&(['session','origin','scan','map','plan',...(kind==='params'?['rev']:[])].some(k=>f[k]!==result[0][k])))throw new Error('设备结果已更新，请重新读取完整快照');
      result.push(f);total=f.pages;this.onProgress({kind,page:page+1,pages:total});
      if(page+1<total&&this.intervalMs)await new Promise(r=>setTimeout(r,this.intervalMs));
    }
    return result;
  }
  fetch(kind,id){if(!['map','path','points','cloud','stable','raw','params'].includes(kind)||!u32(id)||(!id&&kind!=='params'))return Promise.reject(new Error('尚无可读取结果'));return this.run(revision=>this.pages(kind,id,revision));}
  result(target){
    const keys=['session','origin','scan','map','plan'];
    if(!target||keys.some(k=>!u32(target[k]))||!target.map||!target.plan)return Promise.reject(new Error('尚无可读取结果'));
    const identity=Object.fromEntries(keys.map(k=>[k,target[k]]));
    return this.run(async revision=>{
      const groups={map:{pages:null,items:new Map()},path:{pages:null,items:new Map()}};
      const matches=f=>keys.every(k=>f[k]===identity[k]);
      const complete=g=>g.pages!==null&&g.items.size===g.pages;
      const collect=f=>{
        if(f.k==='map'&&(f.algorithm===1||f.g===4)&&!groups.stable)groups.stable={pages:null,items:new Map()};
        if(f.k==='stable'&&!groups.stable)groups.stable={pages:null,items:new Map()};
        const g=groups[f.k];
        if(!Number.isInteger(f.pages)||f.pages<1||f.pages>4096||!Number.isInteger(f.page)||f.page<0||f.page>=f.pages)return false;
        if(g.pages!==null&&g.pages!==f.pages)return false;
        g.pages=f.pages;g.items.set(f.page,f);
        this.onProgress({kind:f.k,page:g.items.size,pages:g.pages});return true;
      };
      await this.read(`radar result ${identity.plan}`,f=>{
        if(!matches(f))return false;
        if(groups[f.k]||f.k==='stable'){collect(f);return Object.values(groups).every(complete);}
        if(f.k!=='result_end')return false;
        for(const[k,total]of [['map',f.mp],['path',f.pp]]){
          if(!Number.isInteger(total)||total<1||total>4096||groups[k].pages!==null&&groups[k].pages!==total)return false;
        }
        if(f.sp!==undefined){
          if(!Number.isInteger(f.sp)||f.sp<1||f.sp>4096)return false;
          groups.stable??={pages:null,items:new Map()};if(groups.stable.pages!==null&&groups.stable.pages!==f.sp)return false;groups.stable.pages=f.sp;
        }
        groups.map.pages=f.mp;groups.path.pages=f.pp;return true;
      },revision);
      for(const kind of Object.keys(groups)){
        const g=groups[kind];
        for(let page=0;page<g.pages;page++)if(!g.items.has(page)){
          const id=kind==='map'?identity.map:kind==='path'?identity.plan:identity.scan;
          const f=await this.read(`radar fetch ${kind} ${id} ${page}`,f=>matches(f)&&f.k===kind&&f.page===page&&f.pages===g.pages,revision);
          collect(f);
        }
      }
      if(groups.stable){
        const pages=Array.from({length:groups.stable.pages},(_,i)=>groups.stable.items.get(i));let hex='';
        if(!groups.stable.pages||pages.some(page=>!page))throw new Error('稳定XY分页尚未完整，冻结结果保留');
        for(const page of pages){let bytes;try{bytes=atob(page.data);}catch{throw new Error('稳定XY编码损坏，冻结结果保留');}if(page.offset!==hex.length/20||bytes.length%10||page.crc32!==pages[0].crc32)throw new Error('稳定XY分页不一致，冻结结果保留');hex+=Array.from(bytes,c=>c.charCodeAt(0).toString(16).padStart(2,'0')).join('');}
        if(hex.length/20!==pages[0].total||rawCrc(hex)!==String(pages[0].crc32).toUpperCase())throw new Error('稳定XY校验失败，冻结结果保留');
        if(revision!==this.revision)throw new Error('读取已取消');
        if(await this.send(`radar ack ${identity.plan}`)!==true)throw new Error('结果ACK未写出，冻结结果保留');
      }
      return Object.fromEntries(Object.entries(groups).map(([kind,g])=>[kind,Array.from({length:g.pages},(_,page)=>g.items.get(page))]));
    });
  }
  async settings(revision){const first=await this.read('radar get',f=>f.k==='params'&&f.page===0,revision);return this.pages('params',0,revision,first);}
  get(){return this.run(revision=>this.settings(revision));}
  set(key,value){return this.run(async revision=>{
    const p=RADAR_PARAMETERS.find(p=>p.key===key),number=Number(value);
    if(!p||String(value).trim()===''||!Number.isFinite(number)||number<p.min||number>p.max)throw new Error('参数值超出范围');
    await this.read(`radar set ${key} ${number}`,()=>false,revision,line=>line==='OK radar parameter_updated_ram');
    const pages=await this.settings(revision),readback=pages.find(p=>p.key===key);
    if(!readback)throw new Error('设备没有返回该参数');
    return {key,value:readback.value,confirmed:Math.abs(readback.value-number)<1e-5};
  });}
}
export function diagnosticSnapshot(model,id){
  const raw=model.snapshots.raw;
  if(!raw?.download_complete||raw.id!==id)throw new Error('原始诊断下载未完成，请等待全部分页');
  const matches=snapshot=>snapshot&&['session','origin','scan','map'].every(key=>snapshot[key]===raw[key]);
  const snapshots={raw},cloud=model.snapshots.cloud,map=model.snapshots.map,path=model.snapshots.path;
  if(raw.complete){
    if(!matches(cloud)||!cloud.complete||cloud.points?.length!==raw.points||!!cloud.truncated!==!!raw.cloud_truncated)throw new Error('同圈完整点云尚未下载，请先下载同圈诊断');
    snapshots.cloud=cloud;
    if(raw.map){if(!matches(map)||!matches(path)||path.plan!==map.plan)throw new Error('同圈地图与路径尚未下载，请先下载同圈诊断');Object.assign(snapshots,{map,path});}
    if(matches(model.snapshots.start))snapshots.start=model.snapshots.start;
  }
  const params=Object.fromEntries(RADAR_PARAMETERS.slice(3,10).map((p,i)=>[p.key,raw.filter[i]]));
  const pose=raw.start===1?[4500-raw.pose[0],2400-raw.pose[1],radarLowerReferenceFromWorld('upper',raw.pose[2])]:raw.pose;
  Object.assign(params,{lidar_x_mm:pose[0],lidar_y_mm:pose[1],zero_deg:pose[2]});
  return JSON.parse(JSON.stringify({v:1,coordinate_frame:'current',diagnostic:{id:raw.id,session:raw.session,origin:raw.origin,scan:raw.scan,map:raw.map,replayable:raw.complete,parameters_source:'raw_capture'},snapshots,params}));
}
export function importSnapshot(value){
  const editedOffline=value.offline&&(value.offline.loaded_from_snapshot===true||value.offlinePath||value.offline.manual_mask!==undefined||[1,2].includes(value.offline.start_id));
  let source=editedOffline?value.offline:value;
  if(!editedOffline&&value.snapshots?.map){
    const map=value.snapshots.map,matches=snapshot=>snapshot&&['session',map.profile==='f1_host'?'config':'origin','scan','map'].every(k=>u32(map[k])&&snapshot[k]===map[k]);
    const start=matches(value.snapshots.start)?value.snapshots.start:null,path=matches(value.snapshots.path)?value.snapshots.path:null;
    source={...map,coordinate_frame:radarSnapshotFrame({...value,...map}),points:path?.points??[],start_id:[1,2].includes(start?.selected)?start.selected:path?.start,params:{...(value.params??value.snapshots.params?.params??{})},stable:matches(value.snapshots.stable)?value.snapshots.stable:undefined,cloud:preferredRadarCloud(...[value.snapshots.cloud,value.snapshots.points].filter(matches))};
    if(Array.isArray(map.geometry)&&map.geometry.length===7)Object.assign(source.params,{m00_q16:map.geometry[0],m01_q16:map.geometry[1],m10_q16:map.geometry[2],m11_q16:map.geometry[3],tx_q16:map.geometry[4],ty_q16:map.geometry[5],zero_tenths:map.geometry[6]});
  }
  if(!source||typeof source!=='object'||!u32(source.mask))throw new Error('快照需要25格mask和坐标来源');
  const result={...source,params:{...source.params},receivedAt:Date.now(),source:'offline',coordinate_frame:'current'};
  const selected=[1,2].includes(source.start_id)?source.start_id:[1,2].includes(source.start?.selected)?source.start.selected:source.start_mode==='upper'?1:source.start_mode==='lower'?2:undefined;
  result.start_id=selected;result.start_mode=selected===1?'upper':selected===2?'lower':'manual';
  if(radarSnapshotFrame(source)==='reference'){
    result.points=(source.points??[]).map(referencePointToCurrent);if(source.pose)result.pose=referencePoseToCurrent(source.pose);
    for(const key of ['geometry','affine'])if(source[key])result[key]=referenceGeometryToCurrent(source[key]);
    if(source.matrix_q16&&source.origin_q16){const g=referenceGeometryToCurrent(source);Object.assign(result,{matrix_q16:g.matrix_q16,origin_q16:g.origin_q16,zero_tenths:g.zero_tenths});}
    if(source.stable)result.stable={...source.stable,coordinate_frame:'current',points:source.stable.points.map(p=>{if(Array.isArray(p)){const xy=referencePointToCurrent(p.slice(2,4));return [...p.slice(0,2),...xy,...p.slice(4)];}return {...p,x_mm:2400-p.y_mm,y_mm:p.x_mm};})};
    if(source.cloud){result.cloud={...source.cloud,coordinate_frame:'current'};if(source.cloud.pose??source.pose)result.cloud.pose=referencePoseToCurrent(source.cloud.pose??source.pose);for(const key of ['geometry','affine'])if(source.cloud[key])result.cloud[key]=referenceGeometryToCurrent(source.cloud[key]);}
    let mask=0;const counts=Array(25).fill(0);
    for(let row=0;row<5;row++)for(let col=0;col<5;col++){const from=row*5+col,to=col*5+4-row;if(source.mask&(1<<from))mask|=1<<to;counts[to]=source.counts?.[from]??0;}
    result.mask=mask;result.counts=counts;
    for(const key of ['manual_mask','scanned_mask','effective_mask','fixed_mask','protected_mask'])if(u32(source[key])){let rotated=0;for(let i=0;i<25;i++)if(source[key]&(1<<i))rotated|=1<<((i%5)*5+4-Math.floor(i/5));result[key]=rotated;}
    result.params=referenceInstallationToCurrent(result.params);
  }
  if(!result.cloud&&Array.isArray(source.points)&&source.cloud_complete===true)
    result.cloud={points:source.points,pose:result.pose,k:source.k==='points'?'points':'cloud',complete:source.complete,cloud_complete:true,truncated:source.truncated};
  return result;
}
