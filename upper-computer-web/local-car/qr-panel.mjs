const UINT32_MAX=0xffffffff;
const CODE=/^\d{3}\+\d{3}\+\d{3}\+\d{3}$/;
const uint32=value=>/^\d+$/.test(value??'')&&Number(value)<=UINT32_MAX?Number(value):null;
const age=value=>value==='NA'?null:uint32(value);

/** Parse only the QR bridge; unrelated console text remains available to other consumers. */
export function parseQrLine(line){
  const match=/^(OK qr status|OK qr read|OK qr stream=on|OK qr stream=off|EVT qr code)(?:\s+(.*))?$/.exec(String(line).trim());
  if(!match)return null;
  const fields={};
  for(const token of (match[2]??'').split(/\s+/).filter(Boolean)){
    const pair=/^([a-z_]+)=([^\s]+)$/.exec(token);
    if(!pair||Object.hasOwn(fields,pair[1]))return null;
    fields[pair[1]]=pair[2];
  }
  if(match[1].startsWith('OK qr stream='))return {kind:'stream',stream:match[1].endsWith('=on')};
  const kind=match[1]==='OK qr status'?'status':match[1]==='OK qr read'?'read':'event';
  const seq=uint32(fields.seq);
  if(seq===null||!(fields.age_ms==='NA'||uint32(fields.age_ms)!==null))return null;
  if(kind!=='event'&&!/^[01]$/.test(fields.valid??''))return null;
  const valid=kind==='event'||fields.valid==='1';
  if((kind==='read'||kind==='event')&&(valid?!CODE.test(fields.code??''):fields.code!=='NA'))return null;
  if(fields.stream!==undefined&&!/^(on|off)$/.test(fields.stream))return null;
  const stats={};
  for(const [key,value] of Object.entries(fields)){
    if(['seq','age_ms','valid','code','stream'].includes(key))continue;
    const n=uint32(value);if(n===null)return null;stats[key]=n;
  }
  return {kind,seq,valid,ageMs:age(fields.age_ms),code:fields.code==='NA'?null:fields.code??null,stream:fields.stream===undefined?undefined:fields.stream==='on',stats};
}

/** Connection-scoped support and request state; retained code is explicitly historical. */
export class QrState{
  constructor(){this.connected=false;this.support='unchecked';this.pending=null;this.code=null;this.codeCurrent=false;this.seq=null;this.ageMs=null;this.stats={};this.stream=false;this.updatedAt=null;this.message='连接后检查二维码查询接口。';}
  connection(connected){
    this.connected=Boolean(connected);this.support='unchecked';this.pending=null;this.codeCurrent=false;this.seq=null;this.stream=false;this.stats={};this.ageMs=null;
    this.message=this.connected?'请先检查二维码接口支持。':'未连接；已接收任务码仅作历史记录。';
  }
  request(command,now=Date.now()){
    if(!this.connected||this.pending)return false;
    const check=this.support!=='supported';
    if(check&&command!=='qr status')return false;
    const kind=command==='qr status'?'status':command==='qr read'?'read':/^qr stream (on|off)$/.test(command)?'stream':null;
    if(!kind)return false;
    this.pending={kind,check,command,deadline:now+3000};
    this.message=check?'正在检查支持…':'等待设备回复…';return true;
  }
  expire(now=Date.now()){
    if(!this.pending||now<this.pending.deadline)return false;
    if(this.pending.check)this.support='unchecked';
    this.pending=null;this.message='设备未在 3 秒内回复，请检查连接后重试。';return true;
  }
  failed(){this.pending=null;this.message='发送失败，请检查连接后重试。';}
  receive(line,now=Date.now()){
    if(!this.connected)return false;
    this.expire(now);
    const message=parseQrLine(line);
    if(!message){
      if(this.pending&&/^ERR (?:qr\b|unknown\b|Unknown\b|format\s*$)/.test(String(line).trim())){
        const check=this.pending.check;this.pending=null;
        if(check)this.support='unsupported';
        this.message=check?'当前固件未提供有效二维码查询接口，请升级后重新检查。':'二维码请求被设备拒绝，请查看终端回复。';return true;
      }
      return false;
    }
    if(message.kind==='event'){
      if(this.support!=='supported')return false;
    }else{
      if(!this.pending||this.pending.kind!==message.kind)return false;
      if(message.kind==='stream'&&message.stream!==(this.pending.command==='qr stream on'))return false;
      if(this.pending.check&&message.kind==='status')this.support='supported';
      this.pending=null;
    }
    if(message.kind==='stream'){
      this.stream=message.stream;this.message=this.stream?'已开启新任务码跟随；等待扫码不代表离线。':'新任务码跟随已关闭。';return true;
    }
    if(message.stream!==undefined)this.stream=message.stream;
    this.stats={...this.stats,...message.stats};
    if(!message.valid){this.codeCurrent=false;this.ageMs=null;this.message='接口可用，设备暂无有效任务码。';return true;}
    if(this.seq!==null&&((message.seq-this.seq)>>>0)>=0x80000000){this.message='已忽略早于当前任务码的回复。';return true;}
    if(message.code){this.code=message.code;this.codeCurrent=true;this.seq=message.seq;this.ageMs=message.ageMs;this.updatedAt=now;}
    else if(this.codeCurrent&&this.seq===message.seq)this.ageMs=message.ageMs;
    else this.codeCurrent=false;
    this.message=message.kind==='status'?'接口可用；读取任务码可查看完整内容。':'已接收任务码；未触发运动。';return true;
  }
}

const mounted=new WeakMap();
export function mountQr({root,send,notify=()=>{},canSend=()=>true}){
  if(mounted.has(root))return mounted.get(root);
  const state=new QrState();let timer;
  const document=root.ownerDocument;
  const make=(tag,text,className)=>{const el=document.createElement(tag);if(text!==undefined)el.textContent=text;if(className)el.className=className;return el;};
  const panel=make('section',undefined,'qr-panel');
  panel.append(make('h2','二维码任务码'),make('p','读取扫码模块缓存的任务码，不触发扫码或车辆运动。','muted'));
  const controls=make('div',undefined,'quick-actions');const buttons=new Map();
  for(const [command,label] of [['qr status','检查接口支持'],['qr read','读取任务码'],['qr stream on','开启跟随'],['qr stream off','关闭跟随']]){
    const button=make('button',label);button.type='button';button.dataset.command=command;buttons.set(command,button);controls.append(button);
  }
  panel.append(controls);
  const status=make('p');status.setAttribute('role','status');status.setAttribute('aria-live','polite');
  const code=make('output','—','qr-code');const parts=make('div',undefined,'qr-groups');
  const metadata=make('p',undefined,'muted');const copy=make('button','复制任务码');copy.type='button';
  const details=make('details');details.append(make('summary','接收统计'));const stats=make('dl',undefined,'qr-stats');details.append(stats);
  panel.append(status,code,parts,metadata,copy,details);root.replaceChildren(panel);
  function render(){
    status.textContent=state.message;
    for(const [command,button] of buttons){button.disabled=!canSend(command)||!state.connected||Boolean(state.pending)||(command!=='qr status'&&state.support!=='supported');}
    buttons.get('qr status').textContent=state.support==='supported'?'读取接收统计':'检查接口支持';
    buttons.get('qr stream on').disabled ||= state.stream;
    buttons.get('qr stream off').disabled ||= !state.stream;
    code.textContent=state.code??'—';copy.disabled=!state.code;
    parts.replaceChildren(...(state.code?.split('+')??['—','—','—','—']).map((value,index)=>make('span',`第 ${index+1} 组 · ${value}`)));
    metadata.textContent=state.code?`${state.codeCurrent?'本次连接有效':'历史记录，需重新读取'} · 接收于 ${new Date(state.updatedAt).toLocaleTimeString('zh-CN')} · 序号 ${state.seq??'—'} · MCU 数据年龄 ${state.ageMs===null?'—':`${state.ageMs} ms`}`:'暂无任务码';
    stats.replaceChildren();
    const labels={received:'接收字节数',accepted:'有效帧数',rejected:'无效帧数',uart_errors:'串口通信错误',event_dropped:'新码事件丢弃数'};
    for(const [key,value] of Object.entries(state.stats))stats.append(make('dt',labels[key]??key),make('dd',String(value)));
    if(!stats.childNodes.length)stats.append(make('dd','—'));
  }
  controls.addEventListener('click',async event=>{
    const command=event.target.closest?.('button')?.dataset.command;if(!command||!state.request(command))return;
    const request=state.pending;render();clearTimeout(timer);timer=setTimeout(()=>{state.expire();render();},3010);
    try{const result=await send(command);if(result===false)throw new Error('send rejected');}
    catch{if(state.pending===request){state.failed();clearTimeout(timer);render();notify(state.message);}}
  });
  copy.addEventListener('click',async()=>{if(!state.code)return;try{await globalThis.navigator.clipboard.writeText(state.code);notify('任务码已复制。');}catch{notify('无法访问剪贴板，请选中任务码手动复制。');}});
  const api={refreshControls:render,receive(line){if(state.receive(line)){if(!state.pending)clearTimeout(timer);render();}},connection(connected){clearTimeout(timer);state.connection(connected);render();}};
  mounted.set(root,api);render();return api;
}
