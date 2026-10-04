import {frameCommand} from './protocol.mjs';
import {parseRadarHost,RadarHostModel} from './radar-host-model.mjs';

const commandFor=action=>({
  status:'radar host status',start:'radar host start',stop:'radar host stop',scan:'radar host scan',
  'live-start':'radar host live start','live-stop':'radar host live stop','live-status':'radar host live status',
})[action];
const kindName=kind=>`host_${kind}`;
const idFor=(kind,frame)=>kind==='config'?frame.config:kind==='map'?frame.map:kind==='path'?frame.plan:kind==='preview'?frame.scan:frame.scan;
const snapshotId=(kind,snapshot)=>kind==='config'?snapshot?.config:kind==='map'?snapshot?.map:kind==='path'?snapshot?.plan:snapshot?.scan;
const cancelled=()=>Object.assign(new Error('雷达结果读取已取消，已完成快照仍保留。'),{name:'RadarTransferCancelled'});

function compactValue(key,value){
  if(Array.isArray(value))return value.join('-');
  if(value&&typeof value==='object'){
    if(['start','goal'].includes(key))return `${value.row},${value.column},${value.anchor}`;
    throw new TypeError(`${key} 不是可发送的紧凑值`);
  }
  if(typeof value==='boolean')return value?'1':'0';
  if(typeof value==='number'&&!Number.isFinite(value))throw new TypeError(`${key} 不是有限数值`);
  const text=String(value);
  if(!text||/\s/.test(text))throw new TypeError(`${key} 必须是无空格的紧凑值`);
  return text;
}

export class RadarHostClient{
  constructor({send,model=new RadarHostModel(),onState=()=>{},onSnapshot=()=>{},onTransfer=()=>{},onError=()=>{},timeoutMs=10000}={}){
    if(typeof send!=='function')throw new TypeError('RadarHostClient requires a send function');
    Object.assign(this,{send,model,onState,onSnapshot,onTransfer,onError,timeoutMs});
    this.waiters=new Set();this.transferRevision=0;this.syncKey='';this.draft=null;this.connected=false;
  }
  connection(value){this.connected=this.model.connection(value);this.onState({connection:this.connected,status:this.model.status,snapshots:this.model.snapshots});return this.connected;}
  receive(line,now=Date.now()){
    if(typeof line==='string'&&/^ERR radar host\b/.test(line.trim().replace(/^(?:arm>\s*)+/,''))){
      const error=new Error(line.trim());for(const waiter of [...this.waiters])this.#rejectWaiter(waiter,error);this.onError(error);return true;
    }
    const parsed=parseRadarHost(line);if(!parsed)return false;
    this.model.accept(line,now);const event=this.model.lastEvent??{accepted:true,frame:parsed};
    for(const waiter of [...this.waiters]){let matched=false;try{matched=waiter.predicate(event);}catch(error){this.#rejectWaiter(waiter,error);continue;}if(matched)this.#resolveWaiter(waiter,event);}
    if(event.status)this.onState({connection:this.connected,status:event.status,snapshots:this.model.snapshots});
    if(event.snapshot)this.onSnapshot(event.key,event.snapshot);
    if(event.error)this.onError(new Error(`雷达分页无效：${event.error}`));
    if(parsed.k==='host_status'&&parsed.ss===4&&parsed.scan&&parsed.map&&parsed.plan)this.#scheduleScan(parsed);
    return true;
  }
  async command(action){
    const wire=commandFor(action);if(!wire)throw new RangeError(`Unknown radar host action: ${action}`);
    if(action==='stop'||action==='live-stop')this.cancelTransfer();
    return this.#write(wire);
  }
  cancelTransfer(){
    this.transferRevision++;this.syncKey='';
    for(const waiter of [...this.waiters])this.#rejectWaiter(waiter,cancelled());
    this.onTransfer({kind:'cancel',active:false});
  }
  async fetch(kind,id){
    if(!['config','map','path','preview','cloud'].includes(kind)||!Number.isInteger(id)||id<0||id>0xffffffff)throw new RangeError('无可读取的雷达结果编号');
    const revision=++this.transferRevision;for(const waiter of [...this.waiters])this.#rejectWaiter(waiter,cancelled());
    return this.#fetch(kind,id,revision);
  }
  async applyConfig(values){
    if(!values||typeof values!=='object'||Array.isArray(values))throw new TypeError('雷达配置必须是对象');
    this.draft=structuredClone(values);const revision=++this.transferRevision;
    for(const waiter of [...this.waiters])this.#rejectWaiter(waiter,cancelled());
    try{
      await this.#write('radar host draft reset');
      const draftWait=this.#wait(event=>event.key==='config'&&event.snapshot?.source==='draft',revision);
      try{await this.#write('radar host get draft');}catch(error){draftWait.cancel();throw error;}
      const draft=(await draftWait.promise).snapshot,draftRevision=draft.config;
      if(!Number.isInteger(draftRevision)||draftRevision<=0)throw new Error('设备没有返回有效草稿版本');
      for(const [key,value] of Object.entries(values))await this.#write(`radar host set ${key} ${compactValue(key,value)}`);
      await this.#write(`radar host apply ${draftRevision}`);
      const activeWait=this.#wait(event=>event.key==='config'&&event.snapshot?.source==='active'&&event.snapshot.config!==draftRevision,revision);
      try{await this.#write('radar host get active');}catch(error){activeWait.cancel();throw error;}
      const active=(await activeWait.promise).snapshot;this.draft=structuredClone(values);return active;
    }catch(error){if(error?.name!=='RadarTransferCancelled')this.onError(error);throw error;}
  }
  async #write(wire){frameCommand(wire);return this.send(wire);}
  #wait(predicate,revision=this.transferRevision){
    let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no;});
    const waiter={predicate,revision,resolve,reject,timer:null};
    waiter.timer=setTimeout(()=>this.#rejectWaiter(waiter,new Error('雷达分页读取超时，已完成快照仍保留。')),this.timeoutMs);waiter.timer?.unref?.();
    this.waiters.add(waiter);return {promise,cancel:()=>this.#rejectWaiter(waiter,cancelled())};
  }
  #resolveWaiter(waiter,value){if(!this.waiters.delete(waiter))return;clearTimeout(waiter.timer);waiter.resolve(value);}
  #rejectWaiter(waiter,error){if(!this.waiters.delete(waiter))return;clearTimeout(waiter.timer);waiter.reject(error);}
  async #request(kind,id,page,revision){
    if(revision!==this.transferRevision)throw cancelled();
    const waiting=this.#wait(event=>event.frame?.k===kindName(kind)&&idFor(kind,event.frame)===id&&event.frame.page===page,revision);
    try{await this.#write(`radar host fetch ${kind}:${id} ${page}`);}catch(error){waiting.cancel();try{await waiting.promise;}catch{}throw error;}
    const event=await waiting.promise;if(revision!==this.transferRevision)throw cancelled();return event;
  }
  async #fetch(kind,id,revision){
    let pages=1;
    this.onTransfer({kind,id,page:0,pages,active:true});
    try{
      for(let page=0;page<pages;page++){
        const event=await this.#request(kind,id,page,revision);pages=event.frame.pages;
        this.onTransfer({kind,id,page:page+1,pages,active:true});
      }
      const snapshot=this.model.snapshots[kind];
      if(!snapshot||snapshotId(kind,snapshot)!==id)throw new Error('雷达分页完整但快照身份不匹配');
      this.onTransfer({kind,id,page:pages,pages,active:false});return snapshot;
    }catch(error){this.onTransfer({kind,id,active:false,error:error.message});throw error;}
  }
  #scheduleScan(status){
    const key=`${status.session}:${status.config}:${status.scan}:${status.map}:${status.plan}`;
    if(key===this.syncKey)return;this.syncKey=key;
    const revision=++this.transferRevision;for(const waiter of [...this.waiters])this.#rejectWaiter(waiter,cancelled());
    void (async()=>{
      try{
        await this.#fetch('map',status.map,revision);
        await this.#fetch('path',status.plan,revision);
        if(revision===this.transferRevision)await this.#fetch('cloud',status.scan,revision);
      }catch(error){if(error?.name!=='RadarTransferCancelled')this.onError(error);}
    })();
  }
}
