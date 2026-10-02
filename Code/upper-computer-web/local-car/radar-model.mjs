import {frameCommand} from './protocol.mjs';
export const INITIAL_VIEW=Object.freeze({x:-320,y:-370,width:3170,height:2990});
export const RADAR_PARAMETERS=[
  ['lidar_x_mm','起点雷达 X / mm',2170,0,2400],['lidar_y_mm','起点雷达 Y / mm',230,0,2400],
  ['zero_deg','起点雷达零角 / °',180,-360,360],['distance_min_mm','距离下限 / mm',100,0,65535],
  ['distance_max_mm','距离上限 / mm',40000,0,65535],['angle_min_tenths','角度下限 / 0.1°',0,0,3600],
  ['angle_max_tenths','角度上限 / 0.1°',3600,0,3600],['threshold','格内点数阈值',3,1,65535],
  ['energy_min','能量下限',0,0,255],['energy_max','能量上限',255,0,255]
].map(([key,label,value,min,max])=>({key,label,value,min,max}));
export const STATIONS={1:'启停区',2:'粗加工区',3:'暂存区',4:'原料区',5:'扫码区'};
export {radarStateText,radarReasonText} from './radar-format.mjs';
export function zoomView(view,anchor,ratio){
  const width=Math.max(300,Math.min(31700,view.width*ratio)),scale=width/view.width;
  return {x:anchor.x+(view.x-anchor.x)*scale,y:anchor.y+(view.y-anchor.y)*scale,width,height:view.height*scale};
}
export const panView=(view,delta)=>({...view,x:view.x-delta.x,y:view.y-delta.y});
const u32=x=>Number.isInteger(x)&&x>=0&&x<=0xffffffff;
export function parseRadar(line){
  if(typeof line!=='string'||!line.startsWith('@RADAR '))return null;
  try{const f=JSON.parse(line.slice(7));return f?.v===1&&typeof f.k==='string'&&['session','origin','scan','map','plan'].every(k=>u32(f[k]))?f:null;}catch{return null;}
}
const idFor=(kind,f)=>kind==='path'?f.plan:kind==='params'?0:['points','cloud'].includes(kind)?f.scan:f.map;
export class RadarModel{
  snapshots={};pending=new Map();connected=false;status=null;nav=null;params={};paramsLive=false;identity=null;
  connection(value){if(!value||!this.connected)for(const snapshot of Object.values(this.snapshots))snapshot.historical=true;this.connected=value;if(!value){this.pending.clear();this.status=null;this.nav=null;this.paramsLive=false;}}
  accept(line,now=Date.now()){
    const f=parseRadar(line);if(!f)return false;
    if(this.identity&&this.identity.session!==f.session)this.pending.clear();
    this.identity={session:f.session,origin:f.origin};
    if(f.k==='status'||f.k==='diag'){this.status={...this.status,...f,receivedAt:now};return true;}
    if(f.k==='nav'){this.nav={...f,receivedAt:now};return true;}
    if(!['map','path','points','cloud','params'].includes(f.k)||!Number.isInteger(f.pages)||f.pages<1||f.pages>4096||!Number.isInteger(f.page)||f.page<0||f.page>=f.pages)return true;
    const identity=`${f.session}:${f.origin}:${f.scan}:${f.map}:${f.plan}:${f.pages}`;
    const key=f.k+':'+identity;
    let pages=this.pending.get(key);if(!pages){for(const k of this.pending.keys())if(k.startsWith(f.k+':'))this.pending.delete(k);pages=new Map();this.pending.set(key,pages);}
    pages.set(f.page,{...f,_receivedAt:now});
    if(pages.size!==f.pages)return true;
    const ordered=Array.from({length:f.pages},(_,i)=>pages.get(i));
    if(ordered.some(p=>!p))return true;
    const complete={...ordered[0],source:'device',receivedAt:now,coordinate_frame:'current',complete:true,historical:false};
    delete complete._receivedAt;
    if(Number.isFinite(complete.age_ms))complete.age_ms+=now-ordered[0]._receivedAt;
    if(f.k==='params'){if(ordered.some(p=>p.rev!==ordered[0].rev))return true;for(const p of ordered)if(typeof p.key==='string'&&Number.isFinite(p.value))this.params[p.key]=p.value;complete.params={...this.params};this.paramsLive=true;}
    else{
      const field=f.k==='map'?'counts':'points',items=[];
      for(const p of ordered){
        if(f.k==='map'&&p.page===0&&!('counts' in p))continue;
        if(!Array.isArray(p[field])||!Number.isInteger(p.offset)||p.offset!==items.length)return true;
        if(p[field].some(v=>field==='counts'?!Number.isFinite(v):!Array.isArray(v)||!v.every(Number.isFinite)))return true;
        items.push(...p[field]);
      }
      complete[field]=items;
      if(f.k==='map'&&f.pages===6&&items.length!==25)return true;
      if(['points','cloud'].includes(f.k)&&Number.isInteger(f.total)&&items.length!==f.total)return true;
      if(f.k==='points'||f.k==='cloud'){const map=this.snapshots.map;if(map&&['session','origin','scan','map'].every(k=>map[k]===f[k])){complete.pose=[...map.pose];complete.age_ms=(map.age_ms??0)+now-map.receivedAt;}}
    }
    this.snapshots[f.k]=complete;this.pending.delete(key);return true;
  }
  describe(snapshot,now=Date.now()){
    if(!snapshot)return '尚无完整结果';
    const cloud=['points','cloud'].includes(snapshot.k);
    const historical=snapshot.historical===true||snapshot.source!=='device'||!this.connected||this.status&&(['session','origin','map',...(cloud?['scan']:[]),...(snapshot.k==='path'?['plan']:[])].some(k=>this.status[k]!==snapshot[k])||cloud&&!this.status.points_valid);
    return `${historical?'历史 · ':''}${snapshot.source==='device'?'F407设备':'离线副本'} · 地图 ${snapshot.map??'—'} / 路径 ${snapshot.plan??'—'} · 结果年龄 ${Math.max(0,Math.floor(((snapshot.age_ms??0)+now-snapshot.receivedAt)/1000))} 秒`;
  }
}
export class RadarExchange{
  constructor({send,intervalMs=1000,timeoutMs=10000,onProgress=()=>{}}){Object.assign(this,{send,intervalMs,timeoutMs,onProgress});this.pending=null;this.busy=false;this.revision=0;}
  cancel(reason='读取已取消，保留已完成快照'){this.revision++;if(this.pending){clearTimeout(this.pending.timer);this.pending.reject(new Error(reason));this.pending=null;}}
  receive(line){
    const f=parseRadar(line),p=this.pending;
    if(p&&/^ERR radar\b/.test(line)){clearTimeout(p.timer);this.pending=null;p.reject(new Error(line));return true;}
    if(!f||!p||!p.match(f))return false;
    clearTimeout(p.timer);this.pending=null;p.resolve(f);return true;
  }
  async run(work){if(this.busy)throw new Error('上一项雷达读取仍在进行');this.busy=true;const revision=this.revision;try{return await work(revision);}finally{this.busy=false;}}
  read(wire,match,revision){
    frameCommand(wire);if(revision!==this.revision)return Promise.reject(new Error('读取已取消'));
    return new Promise((resolve,reject)=>{
      const item={match,resolve,reject,timer:setTimeout(()=>{if(this.pending===item){this.pending=null;reject(new Error('雷达页读回超时，已完成快照仍保留'));}},this.timeoutMs)};
      this.pending=item;Promise.resolve().then(()=>this.send(wire)).then(ok=>{if(ok!==true&&this.pending===item){clearTimeout(item.timer);this.pending=null;reject(new Error('串口写出未确认'));}},error=>{if(this.pending===item){clearTimeout(item.timer);this.pending=null;reject(error);}});
    });
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
  fetch(kind,id){if(!['map','path','points','cloud','params'].includes(kind)||!u32(id)||(!id&&kind!=='params'))return Promise.reject(new Error('尚无可读取结果'));return this.run(revision=>this.pages(kind,id,revision));}
  async settings(revision){const first=await this.read('radar get',f=>f.k==='params'&&f.page===0,revision);return this.pages('params',0,revision,first);}
  get(){return this.run(revision=>this.settings(revision));}
  set(key,value){return this.run(async revision=>{
    const p=RADAR_PARAMETERS.find(p=>p.key===key),number=Number(value);
    if(!p||!Number.isFinite(number)||number<p.min||number>p.max)throw new Error('参数值超出范围');
    if(await this.send(`radar set ${key} ${number}`)!==true)throw new Error('串口写出未确认');
    const pages=await this.settings(revision),readback=pages.find(p=>p.key===key);
    if(!readback)throw new Error('设备没有返回该参数');
    return {key,value:readback.value,confirmed:Math.abs(readback.value-number)<1e-5};
  });}
}
export function importSnapshot(value){
  const source=value.snapshots?.map?{...value.snapshots.map,points:value.snapshots.path?.points??[],params:value.params??value.snapshots.params?.params??{},cloud:[value.snapshots.cloud,value.snapshots.points].filter(c=>c&&['session','origin','map','scan'].every(k=>c[k]===value.snapshots.map[k])).sort((a,b)=>b.receivedAt-a.receivedAt)[0]}:value;
  if(!source||typeof source!=='object'||!u32(source.mask))throw new Error('快照需要25格mask和坐标来源');
  const result={...source,params:{...source.params},receivedAt:Date.now(),source:'offline',coordinate_frame:'current'};
  if(source.coordinate_frame==='reference'){
    const point=p=>[2400-p[1],p[0],...p.slice(2)];
    result.points=(source.points??[]).map(point);if(source.pose)result.pose=[2400-source.pose[1],source.pose[0],(source.pose[2]??0)+90];
    let mask=0;const counts=Array(25).fill(0);
    for(let row=0;row<5;row++)for(let col=0;col<5;col++){const from=row*5+col,to=col*5+4-row;if(source.mask&(1<<from))mask|=1<<to;counts[to]=source.counts?.[from]??0;}
    result.mask=mask;result.counts=counts;
    if(result.params.lidar_x_mm!==undefined){result.params.lidar_x_mm=2400-source.params.lidar_y_mm;result.params.lidar_y_mm=source.params.lidar_x_mm;result.params.zero_deg=(source.params.zero_deg??0)+90;}
  }
  return result;
}
