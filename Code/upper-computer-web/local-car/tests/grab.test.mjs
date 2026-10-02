import test from 'node:test';
import assert from 'node:assert/strict';
import * as protocol from '../protocol.mjs';
import {GRAB_FORM_DEFAULTS,GRAB_TEST_PRESET} from '../grab-params.mjs';

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
  for(const mode of ['fixed','align','pick']) assert.equal(protocol.buildCommand(`grab-${mode}`),`grab start ${mode}`);
  for(const mode of ['align','pick'])for(let color=1;color<=6;color++)
    assert.equal(protocol.buildCommand(`grab-${mode}-color`,{color}),`grab start ${mode} ${color}`);
  for(const invalid of [0,7,'red','1\nstop'])assert.throws(()=>protocol.buildCommand('grab-align-color',{color:invalid}));
  for(const action of ['status','stop','rehome']) assert.equal(protocol.buildCommand(`grab-${action}`),`grab ${action}`);
  assert.equal(protocol.buildCommand('grab-route',{speed:300}),'grab route 300');
  for(const speed of ['',0,5001,'NaN']) assert.throws(()=>protocol.buildCommand('grab-route',{speed}));
  assert.equal(protocol.buildCommand('chassis-route-auto',{speed:300}),'chassis route auto 300');
  assert.equal(protocol.buildCommand('material-auto',{target:1}),'material auto 1');
});

test('grab inputs distinguish startup defaults from unknown calibration and fit the firmware frame',()=>{
  assert.ok(protocol.grabParameters?.length>20);
  const defaults={z_speed:27.777778,loss_grace_ms:250,age_ms:1000,accel:20,body_speed:20,lambda:0.6,travel_mm:0,lead_mm:2,dx_tol:5,dy_tol:5,ref_u:334,ref_v:338,j00:2,j01:0,j02:0,j10:0,j11:2,j12:2,x_ppm:31.746032,x_min:0,x_max:100,x_pre:0,z_observe:0,z_ppm:480,z_min:-80,z_max:0,z_grab:-80,z_place:-60,z_lift:-40,pos_tol:1,stop_speed:2,feedback_ms:2000,close_ms:500};
  for(const parameter of protocol.grabParameters){
    const command=protocol.commands.find(c=>c.id===`grab-set-${parameter.key}`);
    assert.equal(command.fields[0].value,defaults[parameter.key]??'',parameter.key);
    assert.throws(()=>protocol.buildCommand(command.id,{value:''}));
    const value=parameter.min<=0&&parameter.max>=0?0:parameter.min;
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
  assert.equal(protocol.describeGrabMissing('matrix_rank'),'matrix_rank');
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
  assert.match(protocol.grabRecoveryText(reacquire),/已重发蓝色识别.*1\/3.*400/);
  const stopping=protocol.parseGrabStatus('OK grab state=vision_pause reason=vision_pause_stopping recovery_used=0 recovery_count=0 recovery_left_ms=na stop_confirmed=0');
  assert.match(protocol.grabRecoveryText(stopping),/正在停车/); assert.doesNotMatch(protocol.grabRecoveryText(stopping),/剩余/);
  assert.match(protocol.grabRecoveryText(protocol.parseGrabStatus('OK grab state=align reason=vision_recovered recovery_used=2')),/已恢复对准.*2\/2/);
  assert.equal(protocol.grabRecoveryText(protocol.parseGrabStatus('OK grab state=align')),'未报告');
  for(const value of ['-1','1501','1.2']) assert.equal(protocol.parseGrabStatus(`OK grab state=vision_pause recovery_left_ms=${value}`),null);
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
