import test from 'node:test';
import assert from 'node:assert/strict';
import * as protocol from '../protocol.mjs';
import {GRAB_FORM_DEFAULTS,GRAB_TEST_PRESET} from '../grab-params.mjs';

test('ring alignment height and finish selections are independent',()=>{
  for(const height of ['lowered','raised'])for(const finish of ['raise','stay']){
    const wire=protocol.buildCommand('grab-ring-align-options',{ring:'2',height,finish,z:'-100'});
    assert.equal(wire,`grab align ring 2 ${height==='raised'?0:-100}${finish==='stay'?' stay':''}`);
  }
  assert.equal(protocol.buildCommand('grab-ring-align-options',{ring:'1',height:'raised',finish:'stay',z:''}),'grab align ring 1 0 stay');
  for(const args of [{height:'bad',finish:'stay',z:'-100'},{height:'lowered',finish:'bad',z:'-100'},{height:'lowered',finish:'stay',z:'-151'}])
    assert.throws(()=>protocol.buildCommand('grab-ring-align-options',{ring:'1',...args}));
});

test('Base home acceptance is explicit and degraded feedback is a validated boolean',()=>{
  assert.equal(protocol.buildCommand('grab-home-accept'),'grab home accept');
  const line='OK grab state=complete mode=fixed reason=work_ready base_home_degraded=';
  assert.equal(protocol.parseGrabStatus(line+'1').base_home_degraded,1);
  assert.equal(protocol.parseGrabStatus(line+'0').base_home_degraded,0);
  assert.equal(protocol.parseGrabStatus(line+'2'),null);
  assert.equal(protocol.parseGrabStatus('OK grab state=idle').base_home_degraded,null);
});

test('centralized grasp preset builds explicit RAM commands and validates edits',()=>{
  assert.equal(GRAB_FORM_DEFAULTS.travel_mm,0);
  assert.deepEqual(protocol.buildGrabPresetCommands(),Object.entries(GRAB_TEST_PRESET).map(([key,value])=>`grab set ${key} ${value}`));
  assert.deepEqual(protocol.buildGrabPresetCommands({travel_mm:0,body_speed:12}),['grab set travel_mm 0','grab set body_speed 12']);
  assert.throws(()=>protocol.buildGrabPresetCommands({travel_mm:-1}));
  assert.throws(()=>protocol.buildGrabPresetCommands({unknown:1}));
});

test('fixed Z jog converts measured millimetres to signed motor degrees without clamping silently',()=>{
  assert.equal(protocol.buildFixedZJog(480,-20),'pos z -1080');
  assert.equal(protocol.buildFixedZJog(480,-1),'pos z -54');
  assert.equal(protocol.buildFixedZJog(480,1),'pos z 54');
  assert.equal(protocol.buildFixedZJog(400,20),'pos z 900');
  for(const [ppm,mm] of [['',1],[0,1],[NaN,1],[480,0],[480,21],[480,-21],[480,Infinity],[480,0.0001],['480\nhelp',1]])
    assert.throws(()=>protocol.buildFixedZJog(ppm,mm));
  assert.throws(()=>protocol.buildCommand('pos',{axis:'base',degree:1080}));
});

test('single grasp actions have explicit distinct wires and retain pure route controls',()=>{
  for(const mode of ['fixed','align','pick']) assert.equal(protocol.buildCommand(`grab-${mode}`),mode==='align'?'grab align color 3 0':`grab start ${mode}`);
  for(const mode of ['align','pick'])for(let color=1;color<=6;color++)
    assert.equal(protocol.buildCommand(`grab-${mode}-color`,{color}),mode==='align'?`grab align color ${color} 0`:`grab start ${mode} ${color}`);
  for(const invalid of [0,7,'red','1\nstop'])assert.throws(()=>protocol.buildCommand('grab-align-color',{color:invalid}));
  for(const action of ['status','stop','rehome']) assert.equal(protocol.buildCommand(`grab-${action}`),`grab ${action}`);
  assert.equal(protocol.buildCommand('grab-route',{speed:300}),'grab route 300');
  for(const speed of ['',0,5001,'NaN']) assert.throws(()=>protocol.buildCommand('grab-route',{speed}));
  assert.equal(protocol.buildCommand('chassis-route-auto',{speed:300}),'chassis route auto 300');
  assert.equal(protocol.buildCommand('grab-calib',{target:1}),'grab calib 1');
});

test('grab inputs distinguish startup defaults from unknown calibration and fit the firmware frame',()=>{
  assert.ok(protocol.grabParameters?.length>20);
  const defaults=GRAB_FORM_DEFAULTS;
  for(const parameter of protocol.grabParameters){
    const command=protocol.commands.find(c=>c.id===`grab-set-${parameter.key}`);
    assert.equal(command.fields[0].value,defaults[parameter.key]??'',parameter.key);
    assert.throws(()=>protocol.buildCommand(command.id,{value:''}));
    const value=parameter.exclusiveMin?parameter.min+1:parameter.min<=0&&parameter.max>=0?0:parameter.min;
    const wire=protocol.buildCommand(command.id,{value});
    assert.equal(wire,`grab set ${parameter.key} ${value}`);
    assert.ok(protocol.frameCommand(wire).length<=65);
    assert.equal(protocol.buildCommand(`grab-get-${parameter.key}`),`grab get ${parameter.key}`);
    for(const bad of ['NaN','Infinity','1\nhelp']) assert.throws(()=>protocol.buildCommand(command.id,{value:bad}));
  }
  assert.equal(protocol.buildCommand('grab-set-z_grab',{value:0}),'grab set z_grab 0');
  assert.equal(protocol.buildCommand('grab-set-j00',{value:-1.2}),'grab set j00 -1.2');
  assert.throws(()=>protocol.buildCommand('grab-set-retarget_ok',{value:1}));
});

test('grab replies stay in vision and never equate hold or accepted with a proven grasp',()=>{
  for(const line of ['grab status','OK grab state=hold mode=pick reason=done','ERR grab missing=z_ppm','VISION RX fn=B4 color=1 valid=1 cx=350 cy=321']) assert.equal(protocol.moduleForWire(line),'vision');
  assert.match(protocol.describeReply('OK grab state=hold mode=pick reason=done'),/人工/);
  assert.match(protocol.describeReply('OK grab start fixed accepted'),/接受/);
  assert.match(protocol.describeReply('ERR grab missing=z_ppm'),/拒绝/);
});

test('grab status parses explicit feedback and rejects malformed status without inventing values',()=>{
  assert.equal(typeof protocol.parseGrabStatus,'function');
  const status=protocol.parseGrabStatus('OK grab state=settle mode=pick reason=none x=12.5 xt=13 z=20 zt=20 dx=-2 dy=1 age=24 vf=0 vl=0 stop_requested=1 stop_confirmed=0 elapsed=400');
  assert.equal(status.state,'settle');assert.equal(status.x,12.5);assert.equal(status.stop_confirmed,0);
  assert.equal(protocol.parseGrabStatus('OK grab state=hold mode=fixed reason=done x=nan').x,null);
  assert.equal(protocol.parseGrabStatus('OK grab state=prepare mode=fixed reason=none').z,null);
  for(const line of ['OK material state=ALIGNED','OK grab start fixed accepted','OK grab state=hold state=idle','OK grab state=hold x=Infinity','OK grab state=hold stop_confirmed=2']) assert.equal(protocol.parseGrabStatus(line),null,line);
});

test('rejected align keeps its mode and displays every missing calibration with labels',()=>{
  const s=protocol.parseGrabStatus('OK grab state=idle mode=align reason=config_missing missing=x_ppm,x_min,x_max,j12 stop_requested=0 stop_confirmed=0 elapsed=6400');
  assert.equal(s.mode,'align');assert.equal(s.x,null);
  assert.equal(protocol.describeGrabMissing(s.missing),'X 命令脉冲 / mm（x_ppm）；X 最小位置 / mm（x_min）；X 最大位置 / mm（x_max）；DY / X 伸出 / px·mm⁻¹（j12）');
  assert.equal(protocol.describeGrabMissing('none'),'无');
  assert.match(protocol.describeGrabMissing('matrix_rank'),/视觉响应矩阵.*无效/);
  assert.equal(protocol.describeGrabMissing('future_condition'),'future_condition');
  const initial=protocol.parseGrabStatus('OK grab state=align mode=align reason=running missing=none model=initial');
  assert.equal(initial.model,'initial');
});

test('combined route status preserves a route failure independently from an idle grasp task',()=>{
  assert.equal(typeof protocol.parseGrabRouteStatus,'function');
  const route=protocol.parseGrabRouteStatus('ERR grab route state=error segment=2 source=simulated code=156+123+516+231 elapsed_ms=1400 reason=display_failed');
  assert.equal(route.state,'error');assert.equal(route.segment,2);assert.equal(route.reason,'display_failed');
  assert.equal(protocol.parseGrabStatus('OK grab state=idle mode=fixed reason=none').state,'idle');
  for(const line of ['OK grab state=idle','OK grab route state=driving segment=-1','OK grab route state=driving segment=2 segment=3']) assert.equal(protocol.parseGrabRouteStatus(line),null);
});


test('terminal displays color names and current pixel coordinates without protocol metadata',()=>{
  const format=protocol.formatTerminalLine;
  assert.equal(typeof format,'function');
  for(const [id,name] of ['red','yellow','blue','green','black','light_blue'].entries()) {
    assert.equal(format(`VISION RX fn=B4 color=${id+1} valid=1 fresh=1 cx=350 cy=321 dx=-9 dy=17 phase=2 frame=123 age=25`),`Color = ${name}，CX = 350，CY = 321`);
  }
  assert.equal(format('VISION RX fn=B4 color=1 valid=0 fresh=0 cx=na cy=na'),'Color = red，当前无有效视觉数据');
  assert.equal(format('VISION RX fn=B4 color=1 valid=1 fresh=0 cx=350 cy=321'),'Color = red，坐标已过期');
  for(const text of ['camera status','VISION RX fn=B4 color=1 valid=1 fresh=1 cx=bad cy=321']) assert.equal(format(text),text);
});


test('B2 receive provenance is explicit and old firmware does not acquire invented metadata',()=>{
  const s=protocol.parseGrabStatus('OK grab state=align protocol=B2 age_source=rx rx_seq=4294967295 vision=Ok age=50');
  assert.equal(s.rx_seq,4294967295); assert.equal(s.protocol,'B2'); assert.equal(s.age_source,'rx');
  assert.equal(protocol.parseGrabStatus('OK grab state=align').protocol,undefined);
  for(const seq of ['-1','1.5','4294967296']) assert.equal(protocol.parseGrabStatus(`OK grab state=align rx_seq=${seq}`),null);
  assert.equal(protocol.grabParameters.find(p=>p.key==='frames').label,'稳定坐标观测次数');
  assert.equal(protocol.grabParameters.find(p=>p.key==='age_ms').label,'有效坐标接收超时 / ms');
});

test('B2 console distinguishes enumeration, no reply, invalid coordinates and stale coordinates',()=>{
  for(const [state,label] of [['Off','USB 未枚举'],['Wait','等待首个回复'],['Lost','已收到报文，当前无有效坐标'],['Ok','有效坐标'],['Stale','坐标已过期']]) {
    assert.ok(protocol.formatTerminalLine(`OK camera state=${state} protocol=B2 age_source=rx usb=1 active=1 frame=0`).includes(label));
  }
  const old=protocol.formatTerminalLine('OK camera state=Wait usb=1 active=1 frame=0');
  assert.match(old,/协议：未报告/); assert.match(old,/收到过匹配报文：0/);
  assert.equal(protocol.formatTerminalLine('VISION RX fn=B2 color=1 valid=1 fresh=1 cx=334 cy=338 rx_seq=5 age_source=rx age=20'),'Color = red，CX = 334，CY = 338');
});


test('unavailable and legacy grab snapshots do not present old pixel errors as current',()=>{
  assert.equal(protocol.grabPixelText({vision:'Ok',dx:2,dy:-3}),'2 / -3 px');
  for(const vision of [undefined,'Idle','Wait','Lost','Off','Stale'])
    assert.equal(protocol.grabPixelText({vision,dx:2,dy:-3}),'—');
  assert.equal(protocol.grabPixelText({vision:'Ok',dx:null,dy:null}),'—');
});


test('vision pause presents only device recovery progress, with old firmware left unknown',()=>{
  const s=protocol.parseGrabStatus('OK grab state=vision_pause reason=vision_recover_wait recovery_used=1 recovery_count=2 recovery_left_ms=350 stop_confirmed=1');
  assert.equal(s.recovery_used,1); assert.equal(s.recovery_count,2); assert.equal(s.recovery_left_ms,350);
  assert.match(protocol.grabRecoveryText(s),/等待目标恢复.*2\/3.*350/);
  const reacquire=protocol.parseGrabStatus('OK grab state=vision_pause mode=align reason=vision_reacquire_wait recovery_used=0 recovery_count=1 recovery_left_ms=400 stop_confirmed=1');
  assert.match(protocol.grabRecoveryText(reacquire),/已重发目标颜色识别.*1\/3.*400/);
  const stopping=protocol.parseGrabStatus('OK grab state=vision_pause reason=vision_pause_stopping recovery_used=0 recovery_count=0 recovery_left_ms=na stop_confirmed=0');
  assert.match(protocol.grabRecoveryText(stopping),/正在停车/); assert.doesNotMatch(protocol.grabRecoveryText(stopping),/剩余/);
  assert.match(protocol.grabRecoveryText(protocol.parseGrabStatus('OK grab state=align reason=vision_recovered recovery_used=2')),/已恢复对准.*2次/);
  assert.equal(protocol.grabRecoveryText(protocol.parseGrabStatus('OK grab state=align')),'未报告');
  for(const value of ['-1','120001','1.2']) assert.equal(protocol.parseGrabStatus(`OK grab state=vision_pause recovery_left_ms=${value}`),null);
});


test('blue align acceptance and compact progress remain distinct from completion',()=>{
  const ack='OK grab start align accepted color=3 protocol=B2';
  assert.equal(protocol.parseGrabStatus(ack),null);
  assert.match(protocol.describeReply(ack),/尚未确认到位/);
  const progress=protocol.parseGrabStatus('OK grab state=align mode=align reason=running protocol=B2 age_source=rx rx_seq=12 vision=Ok color=3 stop_requested=0 stop_confirmed=0 recovery_used=0 recovery_count=0 recovery_left_ms=na');
  assert.equal(progress.color,3); assert.equal(progress.stop_confirmed,0);
  assert.equal(progress.dx,null); assert.equal(progress.dy,null);
  const done=protocol.parseGrabStatus('OK grab state=idle mode=align reason=cancelled color=3 stop_requested=1 stop_confirmed=1');
  assert.equal(done.color,3); assert.equal(done.stop_confirmed,1);
});


test('short blue loss exposes device timing and requires new observations without displaying old pixels',()=>{
  const s=protocol.parseGrabStatus('OK grab state=vision_grace mode=align reason=vision_gap_decelerating protocol=B2 age_source=rx rx_seq=58 vision=Ok dx=na dy=na color=3 stop_requested=0 stop_confirmed=0 recovery_used=0 recovery_count=0 recovery_left_ms=na loss_ms=170 loss_max_ms=200 grace_count=2');
  assert.equal(s.loss_ms,170); assert.equal(s.loss_max_ms,200); assert.equal(s.grace_count,2);
  assert.equal(protocol.grabPixelText(s),'—');
  assert.match(protocol.grabRecoveryText(s),/短暂漏检.*减速.*170 ms.*2\/3/);
  assert.match(protocol.grabRecoveryText({...s,state:'align',reason:'vision_gap_recovered'}),/未消耗停车恢复次数/);
  assert.equal(protocol.parseGrabStatus('OK grab state=vision_pause recovery_left_ms=1500').recovery_left_ms,1500);
  for(const [key,values] of Object.entries({loss_ms:['-1','1.5','4294967296'],loss_max_ms:['-1','1.5','4294967296'],grace_count:['-1','4','1.5']}))
    for(const value of values) assert.equal(protocol.parseGrabStatus(`OK grab state=vision_grace ${key}=${value}`),null);
  for(const value of [0,200,250,300]) assert.equal(protocol.buildCommand('grab-set-loss_grace_ms',{value}),`grab set loss_grace_ms ${value}`);
  for(const value of [-1,301,1.5]) assert.throws(()=>protocol.buildCommand('grab-set-loss_grace_ms',{value}));
  const compact=protocol.parseGrabStatus('OK grab state=align vision=Ok dx=-95 dy=-1 loss_ms=200 loss_max_ms=200 grace_count=0');
  assert.equal(protocol.grabPixelText(compact),'-95 / -1 px');
});

test('recovery counter is diagnostic and the 12-second wait is readable',()=>{
 const s=protocol.parseGrabStatus('OK grab state=vision_pause recovery_used=5 recovery_count=0 recovery_left_ms=12000 stop_confirmed=1');
 assert.equal(s.recovery_used,5);assert.equal(s.recovery_left_ms,12000);
 assert.equal(protocol.buildGrabPresetCommands({vision_wait_ms:12000})[0],'grab set vision_wait_ms 12000');
});

test('rear-home mission commands and resumable scene configuration have explicit status',()=>{
  assert.equal(protocol.buildCommand('grab-mission',{speed:200,turnSpeed:50}),'grab mission qr 200 50 lower fixed vhigh linear');
  assert.equal(protocol.buildCommand('grab-mission-continue'),'grab mission continue');
  assert.equal(protocol.buildCommand('grab-pose-rear'),'grab pose rear');
  assert.equal(protocol.buildCommand('grab-pose-work'),'grab pose work');
  const r=protocol.parseGrabRouteStatus('OK grab route state=config_wait mode=mission source=fixed code_source=fixed scan_code=356+123+536+231 exec_code=156+123+516+231 batch=2 item=1 station=3 visit=7 stored=12 placed=9 elapsed_ms=5000 reason=z_temp_layer2');
  assert.equal(r.batch,2);assert.equal(r.item,1);assert.equal(r.placed,9);
  assert.match(protocol.describeGrabRoute(r),/第二批.*第 1 件/);
  assert.match(protocol.describeGrabRoute(r),/补齐.*z_temp_layer2/);
  assert.equal(protocol.parseGrabRouteStatus('OK grab route state=transfer mode=mission stored=-1'),null);
});

test('single blue route reports recovery/configuration and completion without mission batch labels',()=>{
  const r=protocol.parseGrabRouteStatus('OK grab route state=config_wait mode=single source=test code=none color=3 slot=1 stored=0 reason=z_grab elapsed_ms=10');
  assert.match(protocol.describeGrabRoute(r),/蓝色/);
  assert.match(protocol.describeGrabRoute(r),/补齐/);
  assert.match(protocol.describeGrabRoute({...r,state:'done'}),/完成/);
});


test('black and white ring test has separate align, manual grip and placement commands',()=>{
  assert.equal(protocol.buildCommand('grab-ring-align',{ring:'1',z:-100}),'grab align ring 1 -100');
  for(const ring of ['1','2','3']) assert.equal(protocol.buildCommand('camera-ring',{ring}),`camera ring ${ring}`);
  for(const ring of ['2','3']) assert.equal(protocol.buildCommand('grab-ring-align',{ring,z:-100}),`grab align ring ${ring} -100`);
  assert.throws(()=>protocol.buildCommand('grab-ring-align',{ring:'4'}));
  assert.equal(protocol.buildCommand('grab-ring-place',{z:-150}),'grab ring place -150');
  assert.throws(()=>protocol.buildCommand('grab-ring-place',{z:-151}));
});

test('ring release completion stays distinct from alignment hold',()=>{
  assert.equal(protocol.describeReply('OK grab state=hold mode=align operation=ring_place result=released'),'色环放置完成 · 已松爪提起');
});

test('ring low observation keeps adjustable height and distinct completion',()=>{
  for(const ring of ['1','2','3']) for(const z of [-30,-60,-100,-150,0])
    assert.equal(protocol.buildCommand('grab-ring-align-stay',{ring,z}),`grab align ring ${ring} ${z} stay`);
  for(const z of [-151,1,NaN]) assert.throws(()=>protocol.buildCommand('grab-ring-align-stay',{ring:'1',z}));
  assert.equal(protocol.describeReply('OK grab state=hold mode=align operation=ring_align reason=ring_aligned_low result=aligned'),'色环对准完成 · 保持对准高度观察');
});
