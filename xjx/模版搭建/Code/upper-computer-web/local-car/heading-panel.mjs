import {reasonText} from './protocol.mjs';

export function buildHeadingTestCommand(station,speed,source='ring'){
  if(!['proc','temp'].includes(station))throw Error('请选择加工区或暂存区');
  if(!['ring','line'].includes(source))throw Error('请选择色环或黑线');
  if(source==='line'&&station!=='temp')throw Error('前方黑线用于暂存区码垛，请选择暂存区');
  if(!String(speed).trim()||!Number.isFinite(Number(speed))||speed<1||speed>10)throw Error('测试转向速度为1～10 °/s');
  return `grab heading ${station} ${Number(speed)}${source==='line'?' line':''}`;
}
export function buildHeadingProbeCommand(station,polarity=-1,speed=5,source='ring'){
  buildHeadingTestCommand(station,speed,source);
  if(![1,-1].includes(Number(polarity)))throw Error('请选择按视觉角同号或反号试转');
  if(Number(speed)<5)throw Error('单次试转速度为5～10 °/s');
  return `grab heading angle ${station} ${Number(polarity)} ${Number(speed)}${source==='line'?' line':''}`;
}
export function mountHeadingTest({root,send,canSend}){
  const make=(tag,text,id)=>{const e=root.ownerDocument.createElement(tag);if(text!==undefined)e.textContent=text;if(id)e.id=id;return e;};
  let connected=false,pending=false,live=false,snapshot=null;
  root.append(make('h2','视觉航向矫正 · 独立测试'),make('p','将小车摆在所选工位前，色环或暂存区色环前方黑线应在视野内。上电回零完成后，先准备视觉姿态，再开始测试；不需要运行比赛或准备跑图。'));
  const setup=make('section');setup.className='vision-card';
  const station=make('select',undefined,'heading-station');
  for(const [value,text] of [['proc','加工区 · 标准车体航向180°'],['temp','暂存区 · 标准车体航向90°']]){const o=make('option',text);o.value=value;station.append(o);}station.value='proc';
  const source=make('select',undefined,'heading-source');
  for(const [value,text]of [['ring','色环连线'],['line','色环前方黑线 · 暂存区码垛']]){const o=make('option',text);o.value=value;source.append(o);}source.value='ring';
  const sourceLabel=make('label','校准参照');sourceLabel.append(source);setup.append(sourceLabel);
  const speed=make('input',undefined,'heading-speed');speed.type='number';speed.min='5';speed.max='10';speed.step='1';speed.value='5';
  const sl=make('label','测试工位'),vl=make('label','转向速度（°/s）');sl.append(station);vl.append(speed);setup.append(sl,vl);
  const pose=make('p','尚未读取准备状态','heading-pose'),result=make('p','尚未开始测试','heading-result');
  const buttons=[];
  const idle=()=>!snapshot||['idle','done','error','cancelled'].includes(snapshot.state);
  function button(label,id,wire,requiresIdle){
    const b=make('button',label,id);b.type='button';
    b.onclick=async()=>{if(b.disabled)return;try{const command=wire();pending=true;update();await send(command);}catch(e){result.textContent=e.message;}finally{pending=false;update();}};
    buttons.push({b,wire,requiresIdle});setup.append(b);return b;
  }
  button('1 · 准备视觉姿态','heading-prepare',()=> 'grab pose work',true);
  setup.append(make('p','与物料取放共用准备动作：抬Z、收X、Base转原参考+117.5°，到位后打开夹爪2450 µs。用于空爪测试。'),pose);
  button('2 · 开始视觉航向矫正','heading-start',()=>buildHeadingTestCommand(station.value,speed.value,source.value),true);
  const polarity=make('select',undefined,'heading-polarity');
  for(const [value,text] of [['-1','反号：底盘转角 = −视觉角'],['1','同号：底盘转角 = 视觉角']]){const o=make('option',text);o.value=value;polarity.append(o);}polarity.value='-1';
  const pl=make('label','单次试转方向');pl.append(polarity);setup.append(pl);
  button('按视觉角单次试转','heading-probe-start',()=>buildHeadingProbeCommand(station.value,polarity.value,speed.value,source.value),true);
  setup.append(make('p','先重新观测，再按图像角转一次，停稳后复测。默认反号：图像+9.51°则底盘顺时针转9.51°；同号则逆时针转。此入口会运动底盘，不自动重设地图角或写入标定。'));
  polarity.addEventListener('change',update);
  button('读取测试状态' ,'heading-status',()=> 'grab status',false);
  button('停止测试','heading-stop',()=> 'grab stop',false);
  setup.append(make('p','“开始视觉航向矫正”未标定时仅显示图像角，不修改航向或自动转车；补齐实测标定后执行矫正。STOP标志不参与控制。没有定位时自动建立本地测试基准，已有定位则保留。'),result);root.append(setup);
  function update(){for(const {b,wire,requiresIdle}of buttons){try{const w=wire();b.disabled=!connected||!canSend(w)||(pending&&w!=='grab stop')||(requiresIdle&&!idle());}catch{b.disabled=true;}}}
  function render(){
    if(!snapshot){result.textContent='尚未读取测试状态';return;}
    if(snapshot.mode!=='heading'){result.textContent='当前设备为其他任务，请先结束该任务再测试';return;}
    const angle=v=>v!==undefined&&v!==null&&Number.isFinite(Number(v))?`${Number(v).toFixed(2)}°`:'未取得';
    result.textContent=`${live?'当前':'历史'} · ${snapshot.station==3?'暂存区':'加工区'} · ${snapshot.heading_source==='line'?'黑线':snapshot.heading_source==='ring'?'色环':'参照来源未回传'} · ${reasonText(snapshot.reason||snapshot.heading_result||snapshot.state)}${Number.isFinite(Number(snapshot.heading_z))?` · 观察Z ${Number(snapshot.heading_z).toFixed(2)} mm`:''} · 图像角 ${angle(snapshot.heading_image)} · 测试前地图角 ${angle(snapshot.heading_before)} → 当前 ${angle(snapshot.map_yaw)}`;
    if(Number(snapshot.probe)===1)result.textContent+=` · 试转前 ${angle(snapshot.probe_before)} → 试转后 ${angle(snapshot.probe_after)} · 指令转角 ${angle(snapshot.probe_delta)} · 实际转角 ${Number.isFinite(Number(snapshot.probe_yaw_delta))?Number(snapshot.probe_yaw_delta).toFixed(3)+'°':'未取得'}`;
  }
  station.addEventListener('change',update);speed.addEventListener('input',update);
  source.addEventListener('change',()=>{if(source.value==='line')station.value='temp';update();});
  update();
  return {update,connection(value){connected=value;live=false;pose.textContent='连接已变化，请读取准备状态';render();update();},
    reset(){snapshot=null;live=false;pose.textContent='尚未读取准备状态';render();update();},
    receive(value){snapshot=value;live=connected;render();update();},
    receiveGrab(value){pose.textContent=value.state==='complete'&&value.result==='work_ready'?'视觉姿态已到位':`准备状态：${reasonText(value.reason||value.state)}`;}};
}
