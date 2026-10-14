const u32=value=>Number.isInteger(value)&&value>=0&&value<=0xffffffff;
const same=(a,b)=>a&&b&&a.s===b.s&&a.id===b.id;
const newer=(q,previous)=>q!==previous&&((q-previous)>>>0)<0x80000000;

export function parseRadarLive(line){
  if(typeof line!=='string')return null;
  line=line.trim().replace(/^(?:arm>\s*)+/,'');if(!line.startsWith('@RADAR '))return null;
  try{
    const frame=JSON.parse(line.slice(7));
    if(frame.v!==1||!['live','live_state'].includes(frame.k)||!u32(frame.s)||!u32(frame.id)||!u32(frame.q))return null;
    if(frame.k==='live_state')return typeof frame.active==='boolean'&&['preparing','waiting','streaming','stopped','error'].includes(frame.state)&&Array.isArray(frame.pose)&&frame.pose.length===3&&frame.pose.every(Number.isFinite)?frame:null;
    if(!frame.id||frame.n!==120||!Number.isInteger(frame.o)||frame.o<0||frame.o>96||frame.o%24||typeof frame.d!=='string')return null;
    const bytes=Uint8Array.from(atob(frame.d),char=>char.charCodeAt(0));if(bytes.length!==96)return null;
    const view=new DataView(bytes.buffer),points=[];
    for(let i=0;i<24;i++){const angle=view.getUint16(i*4,true);if(angle>=3600)return null;points.push([angle,view.getUint16(i*4+2,true)]);}
    return {...frame,points};
  }catch{return null;}
}

export function projectRadarPoint(point,pose){
  if(!point||point[1]<=0||!pose)return null;
  const angle=(pose[2]-point[0]/10)*Math.PI/180;
  return [pose[0]+point[1]*Math.cos(angle),pose[1]+point[1]*Math.sin(angle)];
}

export class RadarLiveModel{
  connected=false;state=null;frame=null;preview=null;retiredSessions=new Set();
  get running(){return !!(this.connected&&this.state?.active&&!this.state.historical);}
  connection(connected){this.connected=connected;if(!connected){this.state=null;this.retiredSessions.clear();if(this.frame)this.frame.historical=true;this.preview=null;}}
  accept(line,now=Date.now()){
    const incoming=parseRadarLive(line);if(!incoming)return false;
    if(incoming.k==='live_state'){
      if(this.retiredSessions.has(incoming.s))return false;
      if(this.state?.s===incoming.s&&incoming.id!==this.state.id&&!newer(incoming.id,this.state.id))return false;
      if(same(this.state,incoming)&&!this.state.active&&incoming.active)return false;
      if(this.state&&this.state.s!==incoming.s)this.retiredSessions.add(this.state.s);
      if(!same(this.state,incoming)){this.preview=null;if(this.frame)this.frame.historical=true;}
      this.state={...incoming,pose:[...incoming.pose],receivedAt:now,historical:!this.connected};
      if(!incoming.active){this.preview=null;if(this.frame)this.frame.historical=true;}
      return true;
    }
    if(!this.state?.active||this.state.historical||!same(this.state,incoming))return false;
    const current=same(this.frame,incoming)&&!this.frame.historical?this.frame:null;
    if(current&&!newer(incoming.q,current.q))return false;
    if(this.preview&&!same(this.preview,incoming))this.preview=null;
    if(this.preview&&incoming.q!==this.preview.q){if(!newer(incoming.q,this.preview.q))return false;this.preview=null;}
    if(!this.preview)this.preview={s:incoming.s,id:incoming.id,q:incoming.q,pose:[...this.state.pose],points:Array(120).fill(null),received:0,parts:new Set(),receivedAt:now,complete:false,historical:false};
    const preview=this.preview;
    for(let i=0;i<24;i++)preview.points[incoming.o+i]=incoming.points[i];
    preview.parts.add(incoming.o);preview.received=preview.parts.size*24;
    if(preview.received===120){this.frame={...preview,points:preview.points.map(point=>[...point]),receivedAt:now,complete:true};delete this.frame.parts;this.preview=null;}
    return true;
  }
  displayed(){return this.preview&&same(this.preview,this.frame)&&!this.frame.historical?this.frame:this.preview??this.frame;}
  describe(now=Date.now()){
    const displayed=this.displayed(),state=this.state;
    const phase=({preparing:'配置中',waiting:'等待完整圈',streaming:'实时更新',stopped:'已停止',error:'采集失败'})[state?.state]??'尚未开启实时点云';
    const age=displayed?Math.max(0,now-displayed.receivedAt):null;
    const history=!this.connected||displayed?.historical||state?.historical||!state?.active;
    const count=displayed?.points.filter(point=>point?.[1]>0).length??0;
    return `${!this.connected?'连接已断开 · ':''}${phase}${history&&displayed?' · 历史':''}${age!==null&&age>5000?' · 暂未更新':''}${displayed?` · ${displayed.complete?'完整圈':'接收中预览 '+displayed.received+'/120'} · ${count} 点 · 更新 ${age} ms前 · 圈 ${displayed.q}`:''}${this.preview&&displayed!==this.preview?' · 接收下一圈 '+this.preview.received+'/120':''}`;
  }
}
