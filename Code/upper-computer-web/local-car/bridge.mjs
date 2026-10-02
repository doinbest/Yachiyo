import {frameCommand} from './protocol.mjs?v=heading-hold-1';

export const scopedStopCommand=wire=>/^(?:stop (?:all|base|z|x)|chassis route cancel|radar (?:stop|nav cancel)|imu cal cancel|(?:vision|material|camera|grab) stop)$/.test(wire);

export const readOnlyCommand=wire=>/^(?:info|help|bus status|console status|radar (?:status|get|nav status|fetch (?:map|path|points|cloud|params) \d+ \d+)|(?:state|position|config) (?:all|base|z|x)|chassis (?:task|status|snapshot|stop-status|feedback|route status)|imu status|qr (?:status|read)|(?:vision|material|camera|map|grab) status|grab get [a-z][a-z0-9_]*)$/.test(wire);

/** Attach to the single host-owned port. Attaching and reconnecting never send commands. */
export class BridgeLink {
  constructor({fetch:fetcher=globalThis.fetch.bind(globalThis),receive=()=>{},state=()=>{},error=()=>{},event=()=>{},configuration=()=>{}}={}) {
    Object.assign(this,{fetcher,receive,state,error,event,configuration});
    this.connected=false;this.disconnecting=false;this.displayState='disconnected';this.port='';this.available=false;this.stopLatched=false;this.cursor=0;
    this.connectionRevision=0;this.stopRevision=0;
    this.pending=new Map();this.completed=new Map();this.generation=0;this.running=false;
  }
  async api(path,body,signal){
    const response=await this.fetcher('/api/'+path,{method:body===undefined?'GET':'POST',cache:'no-store',signal,
      ...(body===undefined?{}:{headers:{'Content-Type':'application/json','X-Console-Token':this.token},body:JSON.stringify(body)})});
    const data=await response.json();if(!response.ok)throw new Error(data.error||`本地服务错误 ${response.status}`);return data;
  }
  setConnected(value,disconnecting=false){const closing=!!value&&disconnecting;if(this.connected!==value||this.disconnecting!==closing||this.displayState==='connecting')this.connectionRevision++;this.connected=value;this.disconnecting=closing;const state=this.disconnecting?'disconnecting':value?'connected':'disconnected';if(state!==this.displayState){this.displayState=state;this.state(state);}}
  rejectPending(message){for(const p of this.pending.values()){clearTimeout(p.timer);p.reject(new Error(message));}this.pending.clear();this.completed.clear();}
  async attach(){
    const status=await this.api('status');this.token=status.token;this.available=true;
    this.cursor=status.last_event_id||0;this.config=status.config;this.configuration(this.config);
    this.port=status.port||'';this.stopLatched=!!status.stop_latched;this.setConnected(!!status.connected,!!status.disconnecting);
    this.event({kind:'stop',latched:this.stopLatched,...status.stop});return status;
  }
  async start(){if(this.running)return;this.running=true;const generation=++this.generation;
    try{await this.attach();}catch(error){this.available=false;this.error(error);}
    if(this.running&&generation===this.generation)void this.poll(generation);
  }
  suspend(){this.running=false;this.generation++;this.controller?.abort();this.rejectPending('已离开本地服务连接。');}
  async poll(generation){
    while(this.running&&generation===this.generation){
      try{
        if(!this.available)await this.attach();
        this.controller=new AbortController();const batch=await this.api('events?after='+this.cursor,undefined,this.controller.signal);
        if(!this.running||generation!==this.generation)return;
        if(batch.gap||batch.last_event_id<this.cursor){this.rejectPending('记录已过期或服务已重启，请重新准备。');this.setConnected(false);await this.attach();continue;}
        for(const e of batch.events||[]){if(e.id<=this.cursor)continue;this.cursor=e.id;this.deliver(e);await Promise.resolve();}
      }catch(error){
        if(!this.running||generation!==this.generation)return;
        this.available=false;this.rejectPending('本地服务连接中断，命令不会重发。');this.setConnected(false);this.error(error);
        await new Promise(resolve=>setTimeout(resolve,1000));
      }
    }
  }
  deliver(e){
    if(e.kind==='connection'){if(e.port&&this.port&&e.port!==this.port)this.connectionRevision++;if(e.port)this.port=e.port;this.setConnected(!!e.connected,!!e.disconnecting);if(!e.connected)this.rejectPending('串口已断开，命令不会重发。');}
    if(e.kind==='stop'){this.stopRevision++;this.stopLatched=!!e.latched;if(this.stopLatched)this.rejectPending('停止锁定已取消待发送指令。');}
    if(e.request_id&&(e.kind==='tx'||e.error)){
      const p=this.pending.get(e.request_id);
      if(p){clearTimeout(p.timer);this.pending.delete(e.request_id);e.error?p.reject(new Error(e.error)):p.resolve(e);}
      else {this.completed.set(e.request_id,e);if(this.completed.size>100)this.completed.delete(this.completed.keys().next().value);}
    }
    if(e.kind==='rx'||e.kind==='radar')this.receive(e.text);
    this.event(e);
  }
  async connect(port,baudrate=115200){if(typeof port!=='string'||!/^COM[1-9]\d*$/i.test(port))throw new Error('请先选择串口。');port=port.toUpperCase();const revision=this.connectionRevision;this.displayState='connecting';this.state('connecting');try{await this.api('connect',{port,baudrate});if(revision===this.connectionRevision){this.port=port;this.setConnected(true);}}catch(error){this.setConnected(this.connected,this.disconnecting);throw error;}}
  async disconnect(){if(this.disconnecting)return;this.setConnected(this.connected,true);const revision=this.connectionRevision;this.rejectPending('正在停车并断开，待发送指令已取消。');try{await this.api('disconnect',{});if(revision===this.connectionRevision)this.setConnected(false);}catch(error){await this.attach();throw error;}}
  async send(command,fromPreparation=false){
    frameCommand(command);if(this.disconnecting)throw new Error('正在停车并断开，不能发送新指令。');if(!this.connected)throw new Error('请先连接本地服务串口。');
    if(this.stopLatched&&!readOnlyCommand(command)&&!scopedStopCommand(command)&&command!=='bus recover')throw new Error('停止已锁定；请确认停车后点击“解除停止锁定”。');
    const generation=this.generation,revision=this.connectionRevision;const result=await this.api('send',{command,...(fromPreparation?{owner:this.preparationOwner}:{})});
    if(result.queued===false&&result.stop){this.deliver({kind:'stop',latched:true,...result.stop});return result;}
    if(generation!==this.generation||revision!==this.connectionRevision||!this.connected||this.disconnecting)throw new Error('连接已变化，命令不会重发。');
    if(this.stopLatched&&!readOnlyCommand(command)&&!scopedStopCommand(command)&&command!=='bus recover')throw new Error('停止已锁定，指令已取消。');
    const done=this.completed.get(result.request_id);if(done){this.completed.delete(result.request_id);if(done.error)throw new Error(done.error);return done;}
    return new Promise((resolve,reject)=>{const timer=setTimeout(()=>{this.pending.delete(result.request_id);reject(new Error('未确认串口写出，请检查日志；不会自动重发。'));},12000);this.pending.set(result.request_id,{resolve,reject,timer});});
  }
  async stop(){this.stopRevision++;this.stopLatched=true;this.rejectPending('停止已取消待发送指令。');this.event({kind:'stop',latched:true,status:'requested'});return this.api('stop',{});}
  async resume(){const revision=this.stopRevision;const result=await this.api('resume',{});if(revision===this.stopRevision){this.stopLatched=false;this.event({kind:'stop',latched:false});}return result;}
  async saveConfig(config){const result=await this.api('config',config);this.config=result.config||result;this.configuration(this.config);return this.config;}
  async acquirePreparation(){
    const owner=crypto.randomUUID();await this.api('preparation',{active:true,owner});this.preparationOwner=owner;return owner;
  }
  async releasePreparation(){const owner=this.preparationOwner;this.preparationOwner=null;if(owner)await this.api('preparation',{active:false,owner});}
}
