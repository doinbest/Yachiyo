import {buildCommand,grabColorOptions} from './protocol.mjs';

/** Preview absolute Z locally; submit only after one full second without scrolling. */
export function createHeightWheel({initial=-100,min=-150,max=0,step=1,onPreview=()=>{},onCommit=()=>{},
  schedule=setTimeout,cancel=clearTimeout}={}){
  let value=initial,timer=null;
  const stop=()=>{if(timer!==null)cancel(timer);timer=null;};
  const normalize=z=>Math.round(Math.min(max,Math.max(min,z))*10)/10;
  return {
    get value(){return value;},cancel:stop,
    set(z){stop();if(!Number.isFinite(Number(z)))return;value=normalize(Number(z));onPreview(value,false);},
    scroll(ticks){
      if(!Number.isFinite(ticks)||!ticks)return;
      stop();value=normalize(value-ticks*step);onPreview(value,true);
      timer=schedule(()=>{timer=null;onCommit(value);},1000);
    }
  };
}

/** Single-object manual stacking test. Device replies own completion, never the timer. */
export function mountStacking({root,send,canSend,schedule=setTimeout,cancel=clearTimeout}){
  const make=(tag,text,cls)=>{const e=root.ownerDocument.createElement(tag);if(text!==undefined)e.textContent=text;if(cls)e.className=cls;return e;};
  const buttons=[];let connected=true;
  const title=make('h2','码垛 · 手动装料','vision-title');
  const note=make('p','先对准下层物料的颜色中心，再装入上层物料并闭合夹爪。调高、松爪和提起分别执行。','form-note');
  root.append(title,note);
  const section=label=>{const e=make('section',undefined,'vision-card');e.append(make('h3',label));root.append(e);return e;};
  const action=(box,label,wire,cls='',before=()=>{})=>{
    const b=make('button',label,cls);b.type='button';b.dataset.send='1';
    const update=()=>{b.title=typeof wire==='function'?wire():wire;b.disabled=!canSend(b.title);};
    b.onclick=()=>{before();update();if(!b.disabled)void send(b.title);};
    box.append(b);buttons.push(update);update();return b;
  };
  const align=section('1 · 对准下层物料'),fields=make('div',undefined,'fields');
  const colorLabel=make('label','下层物料颜色'),color=make('select');color.id='stack-target-color';
  for(const[value,label]of grabColorOptions){const option=make('option',label);option.value=value;color.append(option);}color.value='3';colorLabel.append(color);fields.append(colorLabel);align.append(fields);
  const alignActions=make('div',undefined,'actions');align.append(alignActions);
  action(alignActions,'静态对准并补偿',()=>buildCommand('grab-stack-align',{color:color.value}),'primary',()=>wheel.cancel());
  align.append(make('p','保持Z最高识别；使用视觉 DX、DY 对准绿色准心（视觉默认372,345），再让目标在画面中向右39、向下1像素，补偿到位后等待装料。仅此码垛功能补偿，不写回准心。','form-note'));
  const load=section('2 · 放入物料并夹紧'),loadActions=make('div',undefined,'actions');load.append(loadActions);
  action(loadActions,'打开夹爪','grip open','',()=>wheel.cancel());action(loadActions,'闭合夹爪','grip catch','primary',()=>wheel.cancel());
  const height=section('3 · 调整高度 · 保持夹持');
  const label=make('label','Z绝对目标 / mm'),input=make('input');input.id='stack-release-z';input.type='number';input.min='-150';input.max='0';input.step='1';input.value='-100';label.append(input);
  const dial=make('div',undefined,'height-wheel');dial.tabIndex=0;dial.setAttribute('role','spinbutton');
  dial.setAttribute('aria-label','码垛Z高度，滚动停止1秒后执行');dial.setAttribute('aria-valuemin','-150');dial.setAttribute('aria-valuemax','0');
  const value=make('strong','−100 mm'),caption=make('span','鼠标悬停滚动 · 每格1 mm'),feedback=make('p','滚动后停下1秒执行；下降到位后继续夹持。','form-note');dial.append(value,caption);height.append(label,dial,feedback);
  const preview=(z,pending)=>{input.value=String(z);value.textContent=`${z} mm`;dial.setAttribute('aria-valuenow',String(z));dial.classList.toggle('pending',pending);if(pending)feedback.textContent='高度预览 · 停止滚动1秒后发送最终目标';};
  const wheel=createHeightWheel({schedule,cancel,onPreview:preview,onCommit:async z=>{
    const wire=buildCommand('grab-stack-lower',{z});
    dial.classList.remove('pending');
    if(!connected||!root.isConnected||root.hidden||!canSend(wire)){feedback.textContent='未发送：页面已离开、连接变化或当前不可操作。';return;}
    feedback.textContent=`已请求Z到 ${z} mm，等待设备到位；保持夹持。`;
    if(await send(wire)===false)feedback.textContent='本次高度未写出，请重新操作；不会自动重发。';
  }});
  preview(-100,false);
  dial.addEventListener('wheel',e=>{
    if(!connected||!canSend(buildCommand('grab-stack-lower',{z:wheel.value})))return;
    e.preventDefault();if(e.deltaY)wheel.scroll(Math.sign(e.deltaY));
  },{passive:false});
  dial.addEventListener('keydown',e=>{if(!connected||!canSend('grab stack lower -100'))return;if(e.key==='ArrowDown'||e.key==='ArrowUp'){e.preventDefault();wheel.scroll(e.key==='ArrowDown'?1:-1);}});
  input.addEventListener('input',()=>{wheel.cancel();const z=Number(input.value);if(input.value.trim()&&Number.isFinite(z))wheel.set(z);feedback.textContent='输入仅更新目标，点击下降执行。';update();});
  const heightActions=make('div',undefined,'actions');height.append(heightActions);
  const lower=action(heightActions,'下降到目标高度',()=>buildCommand('grab-stack-lower',{z:wheel.value}),'primary',()=>wheel.cancel());
  lower.id='stack-lower';
  const release=section('4 · 确认高度后松爪'),releaseActions=make('div',undefined,'actions');release.append(releaseActions);
  action(releaseActions,'单独松爪',buildCommand('grab-stack-release'),'primary',()=>wheel.cancel());
  action(releaseActions,'提起到最高',buildCommand('grab-stack-lift'),'',()=>wheel.cancel());
  action(releaseActions,'停止并退出','grab stop','task-stop',()=>wheel.cancel());
  release.append(make('p','松爪保持当前高度；提起由独立按钮执行。停止后取消所有待发高度。','form-note'));
  color.addEventListener('change',()=>{wheel.cancel();update();});
  function update(){buttons.forEach(fn=>fn());dial.setAttribute('aria-disabled',String(!connected||!canSend('grab stack lower -100')));input.disabled=!connected;}
  return {update,receive(s){
    if(['error','stopping','idle'].includes(s.state)){wheel.cancel();dial.classList.remove('pending');}
    if((s.operation==='stack_align'||s.reason==='stack_aligned_ready')&&s.state==='hold')feedback.textContent='准心对准及码垛补偿已完成，可手动装料并夹紧。';
    else if(s.reason==='stack_compensating')feedback.textContent='绿色准心已对准，正在执行码垛专用补偿；完成前继续等待。';
    else if(s.state==='manual_z')feedback.textContent=`Z实际 ${s.z??'—'} mm → 目标 ${s.zt??'—'} mm；保持夹持。`;
    else if(s.operation==='stack_height'&&s.state==='hold')feedback.textContent=`高度已到位：${s.z??'—'} mm；可单独松爪或继续调整。`;
    else if(s.operation==='stack_release'&&s.result==='released')feedback.textContent=`已松爪，保持当前高度 ${s.z??'—'} mm。`;
  },cancel:()=>{wheel.cancel();dial.classList.remove('pending');feedback.textContent='待发高度已取消。';},
    connection(value){connected=value;wheel.cancel();update();},dispose(){wheel.cancel();}};
}
