import {frameCommand} from './protocol.mjs';
import {parseRadarHost,RadarHostModel} from './radar-host-model.mjs?v=fast-result-20261005';

const commandFor=action=>({
  status:'radar host status',start:'radar host start',stop:'radar host stop',scan:'radar host scan',
  'live-start':'radar host live start','live-stop':'radar host live stop','live-status':'radar host live status',
})[action];
const kindName=kind=>`host_${kind}`;
const idFor=(kind,frame)=>kind==='config'?frame.config:kind==='map'?frame.map:kind==='path'?frame.plan:kind==='preview'?frame.live_id:frame.scan;
const snapshotId=(kind,snapshot)=>snapshot&&idFor(kind,snapshot);
const matchesResult=(frame,status)=>frame&&['session','config','scan','map','plan'].every(key=>frame[key]===status[key]);
const resultReady=status=>status.ss===4&&!status.a&&!(status.l&&status.ls===0)&&status.scan&&status.map&&status.plan;
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
  constructor({send,model=new RadarHostModel(),onState=()=>{},onSnapshot=()=>{},onTransfer=()=>{},onError=()=>{},timeoutMs=10000,fastResults=false}={}){
    if(typeof send!=='function')throw new TypeError('RadarHostClient requires a send function');
    Object.assign(this,{send,model,onState,onSnapshot,onTransfer,onError,timeoutMs,fastResults});
    this.waiters=new Set();this.transferRevision=0;this.syncKey='';this.draft=null;this.connected=false;this.lastError=null;this.pageErrorKind=null;this.lastReply=null;this.applying=false;this.readConfigOnStatus=false;
    this.syncing=null;this.queuedStatus=null;this.pendingStart=null;this.writePending=Promise.resolve();
    this.bundleEnd=null;this.fastStopped=false;this.requestedLive=null;
  }
  connection(value){this.connected=this.model.connection(value);if(!this.connected)this.cancelTransfer();this.onState({connection:this.connected,status:this.model.status,snapshots:this.model.snapshots});return this.connected;}
  receive(line,now=Date.now()){
    const reply=typeof line==='string'?line.trim().replace(/^(?:arm>\s*)+/,''):'';
    if(/^(?:OK|ERR) radar host\b/.test(reply)){
      this.lastReply=reply;
      if(reply.startsWith('ERR')){
        this.pendingStart=null;
        const error=new Error(reply);this.lastError=error.message;this.pageErrorKind=null;for(const waiter of [...this.waiters])this.#rejectWaiter(waiter,error);this.onError(error);
      }else {this.lastError=null;this.pageErrorKind=null;}
      return true;
    }
    const parsed=parseRadarHost(line);if(!parsed)return false;
    const bundled=['host_bundle','host_bundle_end'].includes(parsed.k);
    if(bundled&&(this.fastStopped||this.pendingStart&&matchesResult(parsed,this.pendingStart)))return true;
    // A wireless queue can contain newer live circles that were produced before
    // a single-scan command reached the device. Their IDs alone are not a fence.
    if(this.requestedLive!==null&&((bundled&&parsed.live!==undefined&&parsed.live!==Number(this.requestedLive))||
      (parsed.k==='host_status'&&parsed.l!==Number(this.requestedLive))))return true;
    if(parsed.k==='host_status'){
      // A new acquisition invalidates the device's old result stream. Keep its
      // completed history, but never continue repairing it after the scan.
      if((parsed.a||parsed.l&&parsed.ls===0)&&this.model.resultIdentity)this.cancelTransfer();
      if(this.pendingStart&&(parsed.ss===6||resultReady(parsed)&&!matchesResult(parsed,this.pendingStart)))this.pendingStart=null;
    }
    this.model.accept(line,now);const event=this.model.lastEvent??{accepted:true,frame:parsed};
    let bundleStatus=null;
    if(bundled&&!event.error){
      if(parsed.k==='host_bundle_end')this.bundleEnd=parsed;
      if(this.pendingStart&&!matchesResult(parsed,this.pendingStart))this.pendingStart=null;
      // A pushed page can arrive even when the preceding status line was lost.
      bundleStatus={v:1,profile:'f1_host',k:'host_status',...Object.fromEntries(['session','config','scan','map','plan'].map(key=>[key,parsed[key]])),
        ss:4,a:0,l:parsed.live??(this.requestedLive===null?(this.model.status?.l??0):Number(this.requestedLive)),ls:this.requestedLive===false?3:(this.model.status?.ls??3),fast:1,reason:'complete',receivedAt:now};
      if(!this.syncing)this.model.status=Object.freeze(bundleStatus);
    }
    for(const waiter of [...this.waiters]){let matched=false;try{if(waiter.progress?.(event))waiter.restart();matched=waiter.predicate(event);}catch(error){this.#rejectWaiter(waiter,error);continue;}if(matched)this.#resolveWaiter(waiter,event);}
    if(event.status)this.onState({connection:this.connected,status:event.status,snapshots:this.model.snapshots});
    if(event.snapshot){
      if(this.pageErrorKind===event.frame.k){this.lastError=null;this.pageErrorKind=null;}
      this.onSnapshot(event.key,event.snapshot);
    }
    if(event.error&&!(event.recoverable&&(this.syncing||this.fastResults&&bundled))){this.lastError=`雷达分页无效：${event.error}`;this.pageErrorKind=event.frame.k;this.onError(new Error(this.lastError));}
    if(parsed.k==='host_status'&&(this.readConfigOnStatus||resultReady(parsed)))this.#scheduleScan(parsed);
    if(bundleStatus)this.#scheduleScan(bundleStatus);
    return true;
  }
  async command(action){
    let wire=commandFor(action);if(!wire)throw new RangeError(`Unknown radar host action: ${action}`);
    if(this.fastResults&&['scan','live-start','status'].includes(action))wire+=' fast';
    if(action==='stop'||action==='live-stop')this.cancelTransfer();
    if(['stop','live-stop'].includes(action))this.fastStopped=true;
    if(['scan','live-start','status'].includes(action))this.fastStopped=false;
    if(action==='live-start')this.requestedLive=true;
    if(['scan','stop','live-stop'].includes(action))this.requestedLive=false;
    if(action==='status')this.requestedLive=null;
    this.lastError=null;this.pageErrorKind=null;
    if(action==='scan'||action==='live-start'){
      const previous=this.model.status??this.model.snapshots.map;
      this.cancelTransfer();this.pendingStart={...previous};
    }
    if(action==='status'){this.syncKey='';this.readConfigOnStatus=true;}
    try{return await this.#write(wire);}
    catch(error){this.lastError=error.message;this.pendingStart=null;if(action==='status')this.readConfigOnStatus=false;throw error;}
  }
  cancelTransfer(){
    this.transferRevision++;this.syncKey='';this.readConfigOnStatus=false;
    this.syncing=null;this.queuedStatus=null;this.pendingStart=null;this.model.resultIdentity=null;
    this.bundleEnd=null;
    for(const waiter of [...this.waiters])this.#rejectWaiter(waiter,cancelled());
    this.onTransfer({kind:'cancel',active:false});
  }
  async fetch(kind,id){
    if(!['config','map','path','preview','cloud'].includes(kind)||!Number.isInteger(id)||id<0||id>0xffffffff)throw new RangeError('无可读取的雷达结果编号');
    const revision=++this.transferRevision;for(const waiter of [...this.waiters])this.#rejectWaiter(waiter,cancelled());
    this.syncing=null;this.queuedStatus=null;this.model.resultIdentity=null;
    return this.#fetch(kind,id,revision);
  }
  async applyConfig(values){
    if(!values||typeof values!=='object'||Array.isArray(values))throw new TypeError('雷达配置必须是对象');
    this.draft=structuredClone(values);this.applying=true;const revision=++this.transferRevision;
    this.syncing=null;this.queuedStatus=null;this.model.resultIdentity=null;
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
    finally{this.applying=false;const status=this.model.status,config=this.model.snapshots.config;if(status?.ss===4&&status.scan&&status.map&&status.plan&&status.config===config?.config)this.#scheduleScan(status);}
  }
  #write(wire,revision=this.transferRevision){
    frameCommand(wire);
    const previous=/^radar host (?:stop|live stop)$/.test(wire)?Promise.resolve():this.writePending;
    const pending=previous.then(async()=>{
      if(revision!==this.transferRevision)throw cancelled();
      const result=await this.send(wire);
      if(result===false)throw new Error('雷达指令未写出，请检查共享串口连接和停止锁定。');
      return result;
    });
    this.writePending=pending.catch(()=>{});return pending;
  }
  #wait(predicate,revision=this.transferRevision,progress=null,timeoutMs=this.timeoutMs){
    let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no;});
    // The serial write can still be pending when a response rejects this wait.
    // Attach a handler now; the caller still awaits the original promise below.
    void promise.catch(()=>{});
    const waiter={predicate,progress,revision,resolve,reject,timer:null,restart:()=>{
      clearTimeout(waiter.timer);
      waiter.timer=setTimeout(()=>this.#rejectWaiter(waiter,Object.assign(new Error('雷达分页读取超时，已完成快照仍保留。'),{name:'RadarPageTimeout'})),timeoutMs);waiter.timer?.unref?.();
    }};
    waiter.restart();
    this.waiters.add(waiter);return {promise,cancel:()=>this.#rejectWaiter(waiter,cancelled())};
  }
  #resolveWaiter(waiter,value){if(!this.waiters.delete(waiter))return;clearTimeout(waiter.timer);waiter.resolve(value);}
  #rejectWaiter(waiter,error){if(!this.waiters.delete(waiter))return;clearTimeout(waiter.timer);waiter.reject(error);}
  async #request(kind,id,page,revision){
    for(let attempt=0;attempt<3;attempt++){
      try{return await this.#requestOnce(kind,id,page,revision);}
      catch(error){if(error?.name!=='RadarPageTimeout'||attempt===2)throw error;}
    }
  }
  async #requestOnce(kind,id,page,revision){
    if(revision!==this.transferRevision)throw cancelled();
    const waiting=this.#wait(event=>!event.error&&!event.ignored&&event.frame?.k===kindName(kind)&&idFor(kind,event.frame)===id&&event.frame.page===page,revision);
    try{await this.#write(`radar host fetch ${kind}:${id} ${page}`,revision);}catch(error){waiting.cancel();try{await waiting.promise;}catch{}throw error;}
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
  #haveResults(status){return ['map','path'].every(kind=>matchesResult(this.model.snapshots[kind],status)&&!this.model.snapshots[kind].historical);}
  async #fetchBundle(status,revision){
    const relevant=event=>!event.error&&!event.ignored&&matchesResult(event.frame,status);
    const wait=(timeoutMs=this.timeoutMs)=>this.#wait(event=>relevant(event)&&(event.frame.k==='host_bundle_end'||this.#haveResults(status)),revision,
      event=>relevant(event)&&event.frame.k==='host_bundle',timeoutMs);
    this.onTransfer({kind:'result',id:status.plan,active:true});
    try{
      if(!this.#haveResults(status)&&!matchesResult(this.bundleEnd,status)){
        try{await wait(Math.min(this.timeoutMs,1000)).promise;}catch(error){if(error?.name!=='RadarPageTimeout')throw error;}
      }
      for(let attempt=0;!this.#haveResults(status)&&attempt<3;attempt++){
        if(revision!==this.transferRevision)throw cancelled();
        const pending=this.model.pending.get('host_bundle');
        const pages=matchesResult(this.bundleEnd,status)?this.bundleEnd.pages:matchesResult(pending?.identity,status)?pending.identity.pages:null;
        const missing=pages===null?'all':this.model.missingPages('bundle',status,pages).join(',');
        const waiting=wait();
        try{await this.#write(`radar host bundle ${status.plan} ${missing||'all'}`,revision);}catch(error){waiting.cancel();throw error;}
        try{await waiting.promise;}catch(error){if(error?.name!=='RadarPageTimeout')throw error;}
      }
      if(!this.#haveResults(status))throw new Error('雷达紧凑结果补读未完成，已保留上一圈地图和路线。');
    }finally{this.onTransfer({kind:'result',id:status.plan,active:false});}
  }
  async #fetchResult(status,revision){
    const relevant=event=>!event.error&&!event.ignored&&matchesResult(event.frame,status);
    const waiting=this.#wait(event=>relevant(event)&&(event.frame.k==='host_result_end'||this.#haveResults(status)),revision,
      event=>relevant(event)&&['host_map','host_path'].includes(event.frame.k));
    this.onTransfer({kind:'result',id:status.plan,active:true});
    let manifest=null;
    try{
      try{await this.#write(`radar host result ${status.plan}`,revision);}catch(error){waiting.cancel();throw error;}
      try{manifest=(await waiting.promise).frame;}catch(error){if(error?.name!=='RadarPageTimeout')throw error;}
      if(revision!==this.transferRevision)throw cancelled();
      for(const kind of ['map','path']){
        if(matchesResult(this.model.snapshots[kind],status)&&!this.model.snapshots[kind].historical)continue;
        const pending=this.model.pending.get(kindName(kind));
        let pages=manifest?.k==='host_result_end'?manifest[kind==='map'?'mp':'pp']:
          matchesResult(pending?.identity,status)?pending.identity.pages:null;
        if(pages===null){const first=await this.#request(kind,idFor(kind,status),0,revision);pages=first.frame.pages;}
        for(const page of this.model.missingPages(kind,status,pages))await this.#request(kind,idFor(kind,status),page,revision);
      }
      if(!this.#haveResults(status))throw new Error('雷达结果分页完整但同圈身份不匹配');
    }finally{this.onTransfer({kind:'result',id:status.plan,active:false});}
  }
  #scheduleScan(status){
    if(this.applying)return;
    if(this.pendingStart&&!this.readConfigOnStatus)return;
    if(status.l&&this.syncing?.revision===this.transferRevision&&status.session===this.syncing.status.session&&status.config===this.syncing.status.config){this.queuedStatus=status;return;}
    const readConfig=this.readConfigOnStatus;this.readConfigOnStatus=false;
    const ready=resultReady(status)&&!this.pendingStart;
    const key=`${status.session}:${status.config}:${status.scan}:${status.map}:${status.plan}`;
    if(key===this.syncKey&&!readConfig)return;this.syncKey=key;
    const haveResults=this.#haveResults(status);
    if(!readConfig&&haveResults){if(status.fast)void this.#write(`radar host ack ${status.plan}`).catch(error=>this.onError(error));return;}
    const revision=++this.transferRevision;for(const waiter of [...this.waiters])this.#rejectWaiter(waiter,cancelled());
    this.syncing={revision,status};this.model.resultIdentity=ready?{...status}:null;
    void (async()=>{
      try{
        if(readConfig)await this.#fetch('config',status.config,revision);
        if(ready&&!haveResults){if(status.fast)await this.#fetchBundle(status,revision);else await this.#fetchResult(status,revision);}
        if(ready)await this.#write(`radar host ack ${status.plan}`,revision);
        this.lastError=null;this.pageErrorKind=null;
      }catch(error){if(revision===this.transferRevision&&error?.name!=='RadarTransferCancelled'){this.syncKey='';this.lastError=error.message;this.pageErrorKind=null;this.onError(error);}}
      finally{
        if(revision===this.transferRevision){
          this.syncing=null;this.model.resultIdentity=null;
          const queued=this.queuedStatus;this.queuedStatus=null;
          if(queued&&(queued.scan!==status.scan||queued.config!==status.config||this.readConfigOnStatus))this.#scheduleScan(queued);
        }
      }
    })();
  }
}
