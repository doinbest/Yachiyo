import {buildCommand,grabParameters} from './protocol.mjs?v=grab-preset-20260930';
import {createParameterExchange} from './parameter-console.mjs?v=parameter-page-20261001';
import {GRAB_FORM_DEFAULTS} from './grab-params.mjs?v=grab-preset-20260930';

const element=(tag,text,className)=>{
  const out=document.createElement(tag);
  if(text!==undefined)out.textContent=text;
  if(className)out.className=className;
  return out;
};

/** Explicit, operator-driven RAM configuration page; mounting never sends a command. */
export function mountParameterPage({root,send,canSend,onBusy,routeSpeed=5000,onRouteSpeed}){
  const exchange=createParameterExchange({send});
  const controls=[],readouts=[];
  root.append(element('p','这里修改 STM32 当前运行的参数。设备值来自主动读回；输入框是待设置值。连接或打开页面不会自动发送指令，应用后会再次读回核对。抓取任务运行时固件会拒绝 grab set。','form-note'));

  async function operate(status,work,show){
    status.textContent='等待设备回复…';onBusy(true);update();
    try{
      const result=await work();
      show(result);
      status.textContent=result.confirmed===false?'读回与输入不一致，未确认生效':
        result.confirmed===true?'设备读回与输入一致':'已读取设备当前值';
    }catch(error){status.textContent=error.message;}
    finally{onBusy(false);update();}
  }

  for(const group of new Set(grabParameters.map(p=>p.group))){
    const section=element('details',undefined,'parameter-group');
    section.append(element('summary',group));
    const list=element('div',undefined,'parameter-list');
    for(const p of grabParameters.filter(x=>x.group===group)){
      const row=element('div',undefined,'parameter-row');
      const label=element('label',p.label);label.append(element('small',` ${p.key}`));
      const input=element('input');input.type='number';input.min=p.min;input.max=p.max;input.step=p.step??'any';
      input.value=GRAB_FORM_DEFAULTS[p.key]??'';
      const current=element('span','设备值：未读取','parameter-current');readouts.push(current);
      const read=element('button','读取'),apply=element('button','应用');
      read.type=apply.type='button';
      const status=element('span','','parameter-result');status.setAttribute('role','status');
      read.onclick=()=>void operate(status,()=>exchange.readGrab(p.key),result=>{
        current.textContent=`设备值：${result.value===null?'未设置':result.value}`;
        if(input.value==='')input.value=result.value===null?'':String(result.value);
      });
      apply.onclick=()=>void operate(status,()=>exchange.setGrab(p.key,input.value),result=>{
        current.textContent=`设备值：${result.value===null?'未设置':result.value}`;
      });
      input.oninput=update;
      row.append(label,input,current,read,apply,status);list.append(row);
      controls.push({read,apply,input,valid:()=>{try{buildCommand(`grab-set-${p.key}`,{value:input.value});return true;}catch{return false;}},readWire:`grab get ${p.key}`,writeWire:`grab set ${p.key} ${input.value}`});
    }
    section.append(list);root.append(section);
  }

  const arm=element('section',undefined,'parameter-group');arm.append(element('h3','Base / X / Z 运动配置'));
  arm.append(element('p','配置只影响后续机械臂运动，保存在 STM32 RAM；回零速度由驱动器管理。先读取，再修改并应用。','form-note'));
  for(const [axis,name] of [['base','Base 底座'],['x','X 伸缩'],['z','Z 升降']]){
    const row=element('div',undefined,'parameter-row parameter-arm-row');
    row.append(element('strong',name));
    const inputs={};
    for(const [key,label,min,max] of [['rpm','RPM',1,3000],['acc','加速度档位',1,255],['limit','脉冲限制',axis==='z'?0:1,3200]]){
      const wrap=element('label',label),input=element('input');
      input.type='number';input.min=min;input.max=max;input.step=1;input.placeholder=axis==='z'&&key==='limit'?'0 表示不限':'';
      wrap.append(input);row.append(wrap);inputs[key]=input;input.oninput=update;
    }
    const current=element('span','设备值：未读取','parameter-current');readouts.push(current);
    const read=element('button','读取'),apply=element('button','应用');read.type=apply.type='button';
    const status=element('span','','parameter-result');status.setAttribute('role','status');
    const show=result=>{
      current.textContent=`设备值：${result.rpm} RPM / 加速度 ${result.acc} / 脉冲 ${result.limit===0?'不限':result.limit}`;
    };
    read.onclick=()=>void operate(status,()=>exchange.readArm(axis),result=>{
      show(result);
      for(const key of ['rpm','acc','limit'])if(!inputs[key].value)inputs[key].value=String(result[key]);
    });
    apply.onclick=()=>void operate(status,()=>exchange.setArm(axis,inputs.rpm.value,inputs.acc.value,inputs.limit.value),show);
    row.append(current,read,apply,status);arm.append(row);
    controls.push({read,apply,valid:()=>{try{buildCommand('config-set',{axis,rpm:inputs.rpm.value,acc:inputs.acc.value,limit:inputs.limit.value});return true;}catch{return false;}},readWire:`config ${axis}`,writeWire:`config ${axis} ${inputs.rpm.value} ${inputs.acc.value} ${inputs.limit.value}`});
  }
  root.append(arm);

  const route=element('section',undefined,'parameter-group');route.append(element('h3','下次路线启动速度'));
  route.append(element('p','此处只保存网页待用速度，不会启动小车或改变正在运行的路线。真正启动请前往“地图与遥测”中的实车路线控件。','form-note'));
  const routeInput=element('input');routeInput.type='number';routeInput.min=10;routeInput.max=5000;routeInput.step=1;routeInput.value=String(routeSpeed);
  const save=element('button','用于下次路线');save.type='button';
  const routeStatus=element('span','','parameter-result');routeStatus.setAttribute('role','status');
  save.onclick=()=>{
    try{buildCommand('chassis-route-start',{speed:routeInput.value});onRouteSpeed(Number(routeInput.value));routeStatus.textContent='已更新网页待用速度；尚未发送运动指令。';}
    catch(error){routeStatus.textContent=error.message;}
  };
  route.append(routeInput,save,routeStatus,element('a','前往地图与遥测 →'));
  route.lastChild.href='#map';root.append(route);

  function update(){
    for(const item of controls){
      item.read.disabled=exchange.busy||!canSend(item.readWire);
      item.apply.disabled=exchange.busy||!item.valid()||!canSend(item.writeWire);
    }
  }
  function cancel(reason='连接已变化'){
    exchange.cancel(reason);
    for(const current of readouts)current.textContent='设备值：未读取';
    update();
  }
  update();
  return {receive:line=>exchange.receive(line),cancel,update};
}
