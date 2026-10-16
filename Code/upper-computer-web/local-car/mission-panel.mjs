import {describeGrabRoute,describeGrabMissing,reasonText,deviceText} from './protocol.mjs';
import {missionResumeStatus} from './mission-resume-status.mjs';

export function buildMissionCommand({source,code='',speed=1000,turnSpeed=100,start='lower',radar=false,align='raised',visualHeading=false,profile='linear'}){
  if(!['qr','fixed'].includes(source))throw Error('请选择本轮执行码来源');
  if(!String(speed).trim()||!Number.isFinite(Number(speed))||speed<10||speed>5000||!String(turnSpeed).trim()||!Number.isFinite(Number(turnSpeed))||turnSpeed<5||turnSpeed>500)throw Error('路线速度10–5000，转向速度5–500 °/s');
  if(!['lower','upper'].includes(start)||typeof radar!=='boolean')throw Error('请选择起点和雷达模式');
  if(!['lowered','raised'].includes(align))throw Error('请选择工位视觉对准方式');
  if(typeof visualHeading!=='boolean')throw Error('请选择视觉航向矫正');
  if(!['linear','cosine'].includes(profile))throw Error('请选择底盘加减速方式');
  code=code.trim();
  if(source==='fixed'){
    const groups=code.split('+');
    if(!/^[1-6]{3}\+[1-3]{3}\+[1-6]{3}\+[1-3]{3}$/.test(code)||groups.some(g=>new Set(g).size!==3)||[...groups[0]].sort().join('')!==[...groups[2]].sort().join(''))throw Error('任务码须为两批同色、各三件，例如156+123+516+231');
  }
  const wire=`grab mission ${source}${source==='fixed'?' '+code:''} ${Number(speed)} ${Number(turnSpeed)} ${start} ${radar?'radar':'fixed'} ${visualHeading?(align==='raised'?'vhigh':'vlow'):align} ${profile}`;
  if(wire.length>95)throw Error('指令过长');
  return wire;
}

export function mountMission({root,send,canSend}){
  const make=(tag,text,cls,id)=>{const e=root.ownerDocument.createElement(tag);if(text!==undefined)e.textContent=text;if(cls)e.className=cls;if(id)e.id=id;return e;};
  let snapshot=null,live=false,connected=false,pending=false,profilePending=false;
  const hero=make('header',undefined,'mission-hero');
  const headline=make('div');headline.append(make('span','MATCH CONTROL','mission-eyebrow'),make('h2','两批比赛 · 连贯执行'),make('p','扫码 → 原料入仓 → 粗加工 → 暂存 / 码垛 → 返回起点','form-note'));
  const badge=make('span','待开始','mission-badge');hero.append(headline,badge);root.append(hero);
  const setup=make('section',undefined,'vision-card mission-setup');setup.append(make('h3','本轮任务'));
  const fields=make('div',undefined,'mission-fields');
  const field=(label,e)=>{const l=make('label',label);l.append(e);fields.append(l);return e;};
  const source=field('执行码来源',make('select',undefined,undefined,'mission-source'));
  for(const[value,text]of [['qr','采用本轮实际扫码'],['fixed','采用固定任务码']]){const o=make('option',text);o.value=value;source.append(o);}source.value='qr';
  const code=field('固定任务码',make('input',undefined,undefined,'mission-code'));code.value='156+123+516+231';code.maxLength=15;code.spellcheck=false;
  const start=field('比赛起点',make('select',undefined,undefined,'mission-start-point'));
  for(const[value,text]of [['lower','右下起点 · 车头朝左'],['upper','右上起点 · 车头朝下']]){const o=make('option',text);o.value=value;start.append(o);}start.value='lower';
  const radar=field('激光雷达',make('select',undefined,undefined,'mission-radar'));
  for(const[value,text]of [['off','关闭 · 固定路线'],['on','开启 · 自动扫描并规划绕障']]){const o=make('option',text);o.value=value;radar.append(o);}radar.value='off';
  const alignmentField=make('label','工位视觉对准','mission-alignment-field');
  const alignment=make('select',undefined,undefined,'mission-align-height');
  for(const value of ['raised','lowered']){const o=make('option',value==='lowered'?'下降对准 · 仅对照测试':deviceText(value));o.value=value;alignment.append(o);}alignment.value='raised';
  const alignmentNote=make('span','加工与暂存默认在Z=0识别对准，停稳后直接衔接取放，不再先下降识别再抬高；实际取放高度与原料追踪不变。选择仅在开始新比赛时采用，暂停继续沿用本轮模式。','form-note','mission-align-note');
  alignment.setAttribute('aria-describedby','mission-align-note');alignmentField.append(alignment,alignmentNote);fields.append(alignmentField);
  const visualHeading=field('视觉航向矫正',make('select',undefined,undefined,'mission-visual-heading'));
  for(const [value,text] of [['off','关闭 · 原流程'],['on','开启 · 测试']]){const o=make('option',text);o.value=value;visualHeading.append(o);}visualHeading.value='off';
  setup.append(make('p','开启后每次进站仅核验一次：加工区及暂存第一批用色环，暂存第二批码垛用前方黑线；区内换环及加工取回不重复；每件位置对准照常执行。观测最多5秒，接受间断但一致的3帧，证据不足跳过。尚未标定时只观测，不自动改角或转车；暂停继续保留本轮选择。','form-note'));
  const speed=field('路线速度 / mm/s',make('input'));speed.type='number';speed.value='1000';speed.min='10';speed.max='5000';
  const turnSpeed=field('转向速度 / °/s',make('input'));turnSpeed.type='number';turnSpeed.value='100';turnSpeed.min='5';turnSpeed.max='500';
  const profile=field('底盘加减速',make('select',undefined,undefined,'mission-profile'));
  for(const [value,text] of [['linear','线性 · 起步500/800/1000，停车300 mm/s²'],['cosine','余弦 · K200 / 400 ms']]){const o=make('option',text);o.value=value;profile.append(o);}profile.value='linear';
  setup.append(fields,make('p','开启雷达：展开扫描→规划→收回雷达→出发；关闭雷达：直接走固定路线。最终返回所选起点。路线速度包含起点至扫码点；转向速度为原地转向上限，到角自动减速。两种方式都会前往二维码工位实际扫码。串口屏仅显示本轮扫描结果，固定码只决定后续搬运顺序。','form-note'));
  const note=make('p','开始表示小车已在所选起点、按选项朝向摆放，三个料盘已清空且机构已回零。设备自动检查IMU及四轮反馈，复用有效起点定位或建立本轮定位，通过后出发；无需另行确认或准备跑图。','form-note');setup.append(note);
  const actions=make('div',undefined,'actions mission-actions');setup.append(actions);root.append(setup);
  const controls=[];
  const button=(label,wire,id,cls,allowed=()=>true)=>{
    const b=make('button',label,cls,id);b.type='button';
    const update=()=>{try{b.title=typeof wire==='function'?wire():wire;b.disabled=(pending&&!['grab stop','grab mission pause','grab mission stop'].includes(b.title))||!connected||!canSend(b.title)||!allowed();}catch{b.disabled=true;b.title='请检查任务设置';}};
    b.onclick=async()=>{update();if(b.disabled)return;pending=true;updateAll();try{if(id==='mission-start')profilePending=true;updateAll();const sent=await send(b.title);if(id==='mission-start'&&sent===false)profilePending=false;}catch(error){if(id==='mission-start')profilePending=false;reason.textContent=`指令发送失败 · ${error.message}`;}finally{pending=false;updateAll();}};
    actions.append(b);controls.push(update);return b;
  };
  const startAllowed=()=>!live||!snapshot||['idle','done','error','cancelled'].includes(snapshot.state);
  button('开始本轮比赛',()=>buildMissionCommand({source:source.value,code:code.value,speed:speed.value,turnSpeed:turnSpeed.value,start:start.value,radar:radar.value==='on',align:alignment.value,visualHeading:visualHeading.value==='on',profile:profile.value}),'mission-start','primary',startAllowed);
  button('暂停比赛','grab mission pause','mission-pause');
  button('停止比赛','grab mission stop','mission-stop','task-stop');
  button('继续当前阶段','grab mission continue','mission-continue','',()=>live&&['paused','config_wait','stop_wait','bus_recovery_wait'].includes(snapshot?.state));
  button('恢复电机总线','bus recover','mission-recover','',()=>live&&['paused','stop_wait','bus_recovery_wait'].includes(snapshot?.state));
  button('读取比赛状态','grab mission status','mission-status');
  button('恢复诊断','bus recovery','mission-diagnostic');
  setup.append(make('p','暂停保留进度，可继续当前阶段；停止结束本轮，停稳后才能开始新比赛。两者都保持夹爪当前输出。','form-note'));
  const progress=make('section',undefined,'vision-card mission-progress');
  const heading=make('h3','等待开始'),freshness=make('span','尚无设备读回','form-note','mission-freshness');progress.append(heading,freshness);
  const codes=make('div',undefined,'mission-codes');
  const scanned=make('strong','—',undefined,'mission-scanned'),adopted=make('strong','—',undefined,'mission-adopted');
  for(const[label,value]of [['实际扫码 · 串口屏显示',scanned],['本轮采用 · 搬运执行依据',adopted]]){const box=make('div');box.append(make('span',label),value);codes.append(box);}progress.append(codes);
  const adoptedAlignment=make('strong','设备未报告',undefined,'mission-adopted-align'),alignmentReadback=make('p',undefined,'mission-alignment-readback');alignmentReadback.append(make('span','本轮采用 · 工位视觉对准'),adoptedAlignment);progress.append(alignmentReadback);
  const adoptedProfile=make('strong','设备未报告',undefined,'mission-adopted-profile'),profileReadback=make('p',undefined,'mission-alignment-readback');profileReadback.append(make('span','本轮采用 · 底盘加减速'),adoptedProfile);progress.append(profileReadback);
  const adoptedHeading=make('strong','设备未报告',undefined,'mission-adopted-heading');
  const headingResult=make('p','尚无航向核验结果','form-note','mission-heading-result');
  const headingReadback=make('p');headingReadback.append(make('span','本轮采用 · 视觉航向矫正 '),adoptedHeading);progress.append(headingReadback,headingResult);
  const metrics=make('div',undefined,'mission-metrics'),counts=make('strong','入仓 0 / 12 · 放置 0 / 12'),position=make('span','批次 — · 件 — · 仓 — · 目标环 —');metrics.append(counts,position);progress.append(metrics);
  const timeline=make('ol',undefined,'mission-timeline');const steps=['实际扫码','第一批搬运','第二批码垛','返回起点'].map(t=>{const li=make('li',t);timeline.append(li);return li;});progress.append(timeline);
  const reason=make('p','状态以设备报告为准。暂停保留进度，明确点击继续才恢复；停止结束本轮。','form-note');progress.append(reason);root.append(progress);
  const prep=make('details',undefined,'vision-card mission-preflight');prep.append(make('summary','工位高度配置'));
  const link=make('a','配置工位高度 →');link.href='#params/material/z_rough_align';prep.append(link,make('p','粗加工先空爪对准编号环并停稳，再保存X/Z回仓取料。1/2号盘夹取后提起并伸X30mm，3号盘保持收回；转回117.5°、恢复对准位置后下降放置。各工位高度按实物设置，缺项到该阶段填写后继续。','form-note'));root.append(prep);
  function updateAll(){code.disabled=source.value!=='fixed';profile.disabled=pending||profilePending||!!snapshot&&!['idle','done','error','cancelled'].includes(snapshot.state);controls.forEach(fn=>fn());}
  for(const e of [source,code,speed,turnSpeed,start,radar,alignment,visualHeading,profile]){e.addEventListener('input',updateAll);e.addEventListener('change',updateAll);}
  function render(){
    adoptedProfile.textContent=({linear:'线性 · 起步500/800/1000，停车300 mm/s²',cosine:'余弦 · K200 / 400 ms'})[snapshot?.profile]||'设备未报告';
    adoptedHeading.textContent=snapshot?.visual_heading==='on'?'开启 · 测试':snapshot?.visual_heading==='off'?'关闭':'设备未报告';
    const imageAngle=Number(snapshot?.heading_image),observationZ=Number(snapshot?.heading_z),headingSpeed=Number(snapshot?.heading_speed);
    const headingStrategy=snapshot?.heading_method==='direct_ring'?' · 已验证反号1∶1':'';
    headingResult.textContent=snapshot?.heading_result?`${snapshot.heading_source==='line'?'黑线 · ':snapshot.heading_source==='ring'?'色环 · ':''}${reasonText(snapshot.heading_result)}${headingStrategy}${Number.isFinite(headingSpeed)?` · 校正上限 ${headingSpeed.toFixed(1)} °/s`:""}${Number.isFinite(imageAngle)?` · 图像角 ${imageAngle.toFixed(2)}°`:''}${Number.isFinite(observationZ)?` · 观察Z ${observationZ.toFixed(2)} mm`:''}`:'尚无航向核验结果';
    adoptedAlignment.textContent=['lowered','raised'].includes(snapshot?.align)?deviceText(snapshot.align):'设备未报告';
    if(snapshot){heading.textContent=describeGrabRoute(snapshot);scanned.textContent=snapshot.scan_code&&snapshot.scan_code!=='none'?snapshot.scan_code:'等待本轮扫码';adopted.textContent=snapshot.exec_code&&snapshot.exec_code!=='none'?snapshot.exec_code:'扫码后采用';counts.textContent=`入仓 ${snapshot.stored??0} / 12 · 放置 ${snapshot.placed??0} / 12`;position.textContent=`第 ${snapshot.batch??'—'} 批 · 第 ${snapshot.item??'—'} 件 · 仓 ${snapshot.slot??'—'} · 目标环 ${snapshot.ring??'—'}`;
      badge.textContent=({idle:'待开始',cancelled:'本轮已结束',cancelling:'正在结束本轮',cancel_wait:'结束中 · 停车待确认',bus_recovery_stopping:'通信异常 · 停车中',bus_recovering:'总线恢复中',bus_recovery_verify:'恢复后确认停稳',bus_recovery_wait:'总线恢复待处理',bus_recovery_retry:'通信重试中',stop_wait:'停车待恢复',paused:'已暂停',stopping:'正在暂停',config_wait:'等待配置',done:'已完成',error:'需处理'})[snapshot.state]||'执行中';
      reason.textContent=snapshot.state==='cancel_wait'?'本轮已停止推进，机构停车尚未确认；处理通信/驱动故障后再次点击停止比赛，不恢复本轮。':snapshot.state==='cancelled'?'本轮已结束且停车已确认，不能继续原阶段；参数、参考及库存保留。':snapshot.state==='stop_wait'?'停车尚未确认：处理通信/驱动故障，执行总线恢复后点击继续，重新确认停稳再续接。':snapshot.state==='config_wait'&&String(snapshot.reason).startsWith('radar_')?`${reasonText(snapshot.reason)}。点击继续当前阶段重试。`:snapshot.state==='config_wait'?`待补齐：${describeGrabMissing(snapshot.reason)}。填写后继续当前件。`:`${reasonText(snapshot.reason)}${snapshot.state==='paused'?' · 停车已确认，可继续当前阶段':''}`;
      const resume=missionResumeStatus(snapshot);
      if(resume){badge.textContent=resume.label;reason.textContent=resume.detail;}
      const index=snapshot.state==='done'?4:snapshot.station===1?3:!snapshot.exec_code||snapshot.exec_code==='none'?0:snapshot.batch===2?2:1;
      if(snapshot.state==='bus_recovery_wait')reason.textContent=`本轮进度保留 · ${reasonText(snapshot.reason)}。处理故障后点击“恢复电机总线”，验证后再继续；也可点击“继续当前阶段”重新恢复并续跑。`;
      else if(['bus_recovery_stopping','bus_recovering','bus_recovery_verify','bus_recovery_retry'].includes(snapshot.state))reason.textContent=`${reasonText(snapshot.reason)}。${String(snapshot.auto_resume)==='1'?'验证成功后自动继续；点击暂停可取消自动续跑。':'验证成功后保持暂停，等待手动继续。'}`;
      steps.forEach((e,i)=>{e.className=i<index?'complete':i===index?'current':'';});
    }
    freshness.textContent=live?'设备最近读回':snapshot?'历史状态 · 请读取比赛状态':'尚无设备读回';updateAll();
  }
  render();return {reset(){profilePending=false;snapshot=null;live=false;heading.textContent='等待开始';badge.textContent='待开始';scanned.textContent=adopted.textContent='—';counts.textContent='入仓 0 / 12 · 放置 0 / 12';position.textContent='批次 — · 件 — · 仓 — · 目标环 —';steps.forEach(e=>{e.className='';});reason.textContent='设备已重启，完成回零并放好小车后开始；底盘定位由设备自动准备。';render();},update:updateAll,receiveReply(line){const rejected=String(line).trim().replace(/^(?:arm>\s*)+/,'').match(/^ERR grab mission reason=([a-z_]+)(?:\s|$)/);if(rejected){profilePending=false;reason.textContent=`比赛指令未受理 · ${reasonText(rejected[1])}`;updateAll();}},receive(s){if(s?.mode!=='mission')return;snapshot=s;profilePending=false;live=connected;render();},connection(value){connected=value;live=false;render();}};
}
