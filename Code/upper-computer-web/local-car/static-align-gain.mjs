import {buildCommand} from './protocol.mjs';
import {createParameterExchange,parseGrabConfig} from './parameter-console.mjs';
import {GRAB_FORM_DEFAULTS,GRAB_PARAMETER_METADATA} from './grab-parameter-metadata.mjs';

const el=(tag,text,cls)=>{const item=document.createElement(tag);if(text!==undefined)item.textContent=text;if(cls)item.className=cls;return item;};

/** Explicit shared static gain editing; mounting and reconnecting are passive. */
export function mountStaticAlignGain({root,send,canSend,onBusy=()=>{}}){
  const exchange=createParameterExchange({send}),metadata=GRAB_PARAMETER_METADATA.lambda;
  const fields=el('div',undefined,'fields'),label=el('label','静态收敛增益 λ（s⁻¹）'),input=el('input');
  Object.assign(input,{type:'number',id:'static-align-lambda',min:metadata.min,max:metadata.max,step:metadata.step,value:String(GRAB_FORM_DEFAULTS.lambda)});
  label.htmlFor=input.id;input.setAttribute('aria-label','静态协同对准收敛增益λ');label.append(input);fields.append(label);
  const buttons=el('div',undefined,'actions'),readButton=el('button','读取λ'),applyButton=el('button','应用并核对λ');
  readButton.type=applyButton.type='button';buttons.append(readButton,applyButton);
  const readout=el('p','设备λ：尚未读取；输入为草稿。','form-note'),message=el('p','','validation');message.setAttribute('aria-live','polite');
  root.append(fields,buttons,readout,message,el('p','λ越大纠偏越积极；此参数由静态颜色、编号环和工位对准共用，不改变原料追踪的独立λ。参数只存设备RAM。','form-note'));
  let connected=false,busy=false,dirty=false,revision=0,lastValue;
  function wire(){return buildCommand('grab-set-lambda',{value:input.value});}
  function update(){
    readButton.disabled=busy||!connected||!canSend('grab get lambda');
    input.disabled=busy;
    try{applyButton.disabled=busy||!connected||!canSend(wire());}
    catch{applyButton.disabled=true;}
  }
  function showValue(value,historical=false){
    lastValue=value;readout.textContent=`${historical?'历史设备λ':'设备λ'}：${value===null?'未设置':value}`;
  }
  async function run(apply){
    if(busy||!connected)return;
    let requested;
    try{if(apply){wire();requested=Number(input.value);}}
    catch(error){message.textContent=error.message;update();return;}
    if(!canSend(apply?wire():'grab get lambda'))return;
    const current=++revision;busy=true;message.textContent=apply?'正在应用并核对λ…':'正在读取λ…';onBusy(true);update();
    try{
      const result=await (apply?exchange.setGrab('lambda',requested):exchange.readGrab('lambda'));
      if(current!==revision)return;
      showValue(result.value);
      if(!apply&&!dirty)input.value=result.value===null?'':String(result.value);
      if(apply&&result.confirmed){dirty=false;message.textContent='λ已应用并读回一致。';}
      else message.textContent=apply?'λ读回不一致，请核对设备值。':'已读取设备λ。';
    }catch(error){if(current===revision)message.textContent=error.message;}
    finally{busy=false;onBusy(false);update();}
  }
  function cancel(reason='λ参数操作已取消。'){
    revision++;exchange.cancel(reason);message.textContent=reason;
    if(lastValue!==undefined)showValue(lastValue,true);update();
  }
  function receive(line){
    const consumed=exchange.receive(line),result=parseGrabConfig(line);
    if(connected&&result?.key==='lambda')showValue(result.value);
    return consumed;
  }
  input.oninput=()=>{dirty=true;message.textContent='待应用；编辑不会发送指令。';update();};
  readButton.onclick=()=>run(false);applyButton.onclick=()=>run(true);update();
  return {update,receive,cancel,read:()=>run(false),apply:()=>run(true),get busy(){return busy;},
    connection(value){if(connected!==value){connected=value;cancel('连接已变化，λ参数操作已取消；请重新读取设备值。');}update();}};
}
