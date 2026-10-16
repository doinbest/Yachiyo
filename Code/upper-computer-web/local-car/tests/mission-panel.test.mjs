import test from 'node:test';
import assert from 'node:assert/strict';
import {buildMissionCommand,mountMission} from '../mission-panel.mjs';
import {parseGrabRouteStatus,buildCommand,describeGrabRoute,reasonText,frameCommand} from '../protocol.mjs';

test('smooth start is opt-in, lower-linear only, and composes with mission scale',()=>{
 assert.equal(buildMissionCommand({source:'qr',smoothStart:true}),'grab mission qr 1000 100 lower fixed vhigh linear smooth=1');
 assert.equal(buildMissionCommand({source:'qr',smoothStart:false}),'grab mission qr 1000 100 lower fixed vhigh linear');
 const wire=buildMissionCommand({source:'fixed',code:'156+123+516+231',speed:1000,turnSpeed:360,scale:5,radar:true,smoothStart:true});
 assert.equal(wire,'grab mission fixed 156+123+516+231 1000 360 lower radar vhigh linear scale=5 smooth=1');
 assert.ok(wire.length<=95);assert.equal(wire.split(' ').length,12);assert.equal(frameCommand(wire),wire+'\r\n');
 for(const options of [{start:'upper'},{profile:'cosine'},{smoothStart:'1'}])assert.throws(()=>buildMissionCommand({source:'qr',smoothStart:true,...options}));
 assert.equal(buildCommand('grab-mission',{speed:1000,turnSpeed:100,scale:1.2,smoothStart:true}),'grab mission qr 1000 100 lower fixed vhigh linear scale=1.2 smooth=1');
 for(const options of [{start:'upper'},{profile:'cosine'}])assert.throws(()=>buildCommand('grab-mission',{speed:1000,turnSpeed:100,smoothStart:true,...options}));
});

test('smooth start draft stays passive and clears for upper start or cosine',()=>{
 const f=fixture();f.panel.connection(true);const smooth=f.get('mission-start-smooth');
 assert.ok(smooth);assert.equal(smooth.checked,false);assert.equal(smooth.disabled,false);
 smooth.checked=true;smooth.change();assert.deepEqual(f.wires,[]);
 assert.match(f.get('mission-adopted-start-smooth').textContent,/未报告/);
 for(const [id,value,reset] of [['mission-start-point','upper','lower'],['mission-profile','cosine','linear']]){
  f.get(id).value=value;f.get(id).change();assert.equal(smooth.checked,false);assert.equal(smooth.disabled,true);assert.deepEqual(f.wires,[]);
  f.get(id).value=reset;f.get(id).change();assert.equal(smooth.disabled,false);assert.equal(smooth.checked,false);
  smooth.checked=true;smooth.change();
 }
 assert.deepEqual(f.wires,[]);
});

test('smooth start locks with the mission and readback never derives from its draft',async()=>{
 const f=fixture();f.panel.connection(true);const smooth=f.get('mission-start-smooth');assert.ok(smooth);
 smooth.checked=true;smooth.change();await f.get('mission-start').onclick();
 assert.equal(f.wires[0],'grab mission qr 1000 100 lower fixed vhigh linear smooth=1');assert.equal(smooth.disabled,true);
 assert.equal(f.get('mission-start-point').disabled,true);
 for(const state of ['driving','paused','bus_recovery_wait']){
  f.panel.receive(parseGrabRouteStatus(`OK grab route state=${state} mode=mission start_smooth=0 entry_smooth=1`));
  assert.equal(smooth.disabled,true);assert.match(f.get('mission-adopted-start-smooth').textContent,/关闭/);
 }
 f.panel.connection(false);f.panel.connection(true);assert.equal(smooth.disabled,true);assert.equal(f.wires.length,1);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission start_smooth=1'));
 assert.match(f.get('mission-adopted-start-smooth').textContent,/开启/);
 await f.get('mission-continue').onclick();assert.equal(f.wires.at(-1),'grab mission continue');
 f.panel.receive(parseGrabRouteStatus('OK grab route state=done mode=mission entry_smooth=1'));
 assert.equal(smooth.disabled,false);assert.equal(f.get('mission-start-point').disabled,false);assert.match(f.get('mission-adopted-start-smooth').textContent,/未报告/);
 f.panel.reset();assert.equal(f.get('mission-start-point').disabled,false);assert.equal(smooth.disabled,false);assert.match(f.get('mission-adopted-start-smooth').textContent,/未报告/);assert.equal(f.wires.length,2);
});

test('smooth start readback accepts only binary reported values and explains rejection',()=>{
 for(const value of [0,1]){
  const status=parseGrabRouteStatus(`OK grab route state=paused mode=mission start_smooth=${value} entry_smooth=${value}`);
  assert.equal(status.start_smooth,value);assert.equal(status.entry_smooth,value);
 }
 for(const key of ['start_smooth','entry_smooth'])for(const value of ['2','-1','on','nan'])assert.equal(parseGrabRouteStatus(`OK grab route state=paused mode=mission ${key}=${value}`),null);
 assert.match(reasonText('start_smooth_lower_linear_only'),/右下.*linear/);
});

test('mission scale validates its own and effective speed bounds without scaling turn speed',()=>{
 assert.equal(buildMissionCommand({source:'qr',scale:1.2}),'grab mission qr 1000 100 lower fixed vhigh linear scale=1.2');
 assert.equal(buildMissionCommand({source:'fixed',code:'156+123+516+231',speed:1000,scale:5,turnSpeed:360}),'grab mission fixed 156+123+516+231 1000 360 lower fixed vhigh linear scale=5');
 for(const scale of ['',0,0.09,5.01,Infinity,NaN])assert.throws(()=>buildMissionCommand({source:'qr',scale}));
 for(const [speed,scale] of [[10,0.1],[1100,5]])assert.throws(()=>buildMissionCommand({source:'qr',speed,scale}));
 assert.throws(()=>buildMissionCommand({source:'qr',scale:1.2,profile:'cosine'}));
 assert.equal(buildMissionCommand({source:'qr',scale:1,profile:'cosine'}),'grab mission qr 1000 100 lower fixed vhigh cosine');
 assert.equal(buildCommand('grab-mission',{speed:1000,turnSpeed:100,scale:1.2}),'grab mission qr 1000 100 lower fixed vhigh linear scale=1.2');
 assert.equal(frameCommand(buildMissionCommand({source:'fixed',code:'156+123+516+231',speed:100,scale:0.1})),'grab mission fixed 156+123+516+231 100 100 lower fixed vhigh linear scale=0.1\r\n');
 assert.throws(()=>buildCommand('grab-mission',{speed:1100,turnSpeed:100,scale:5}));
});

test('scale preview is passive, locks through acceptance and recovery, and never supplies readback',async()=>{
 const f=fixture();f.panel.connection(true);
 const scale=f.get('mission-scale'),speed=f.get('mission-base-speed');
 assert.ok(scale);assert.ok(speed);assert.equal(scale.value,'1');
 scale.value='1.2';scale.input();assert.match(f.get('mission-speed-preview').textContent,/1200/);
 assert.match(f.get('mission-adopted-scale').textContent,/未报告/);assert.deepEqual(f.wires,[]);
 await f.get('mission-start').onclick();assert.equal(f.wires.at(-1),'grab mission qr 1000 100 lower fixed vhigh linear scale=1.2');
 assert.equal(scale.disabled,true);assert.equal(speed.disabled,true);assert.equal(f.get('mission-start').disabled,true);
 for(const state of ['driving','paused','bus_recovery_wait']){
  f.panel.receive(parseGrabRouteStatus(`OK grab route state=${state} mode=mission route_base_speed=800.00 route_scale=0.50 route_speed=400.00`));
  assert.equal(scale.disabled,true);assert.equal(speed.disabled,true);
  assert.equal(f.get('mission-adopted-scale').textContent,'0.5');
  assert.match(f.get('mission-adopted-base-speed').textContent,/800/);assert.match(f.get('mission-adopted-speed').textContent,/400/);
 }
 f.panel.connection(false);f.panel.connection(true);assert.equal(scale.disabled,true);assert.equal(f.wires.length,1);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission route_base_speed=800 route_scale=0.5 route_speed=400'));
 await f.get('mission-continue').onclick();assert.equal(f.wires.at(-1),'grab mission continue');
 f.panel.receive(parseGrabRouteStatus('OK grab route state=done mode=mission route_speed=400'));
 assert.equal(scale.disabled,false);assert.equal(speed.disabled,false);assert.match(f.get('mission-adopted-scale').textContent,/未报告/);assert.match(f.get('mission-adopted-base-speed').textContent,/未报告/);
 f.panel.reset();assert.match(f.get('mission-adopted-speed').textContent,/未报告/);
});

test('cosine with non-unit scale and invalid effective speed block start without sending',()=>{
 const f=fixture();f.panel.connection(true);const scale=f.get('mission-scale');
 assert.ok(scale);scale.value='1.2';scale.input();f.get('mission-profile').value='cosine';f.get('mission-profile').change();
 assert.equal(f.get('mission-start').disabled,true);assert.match(f.get('mission-speed-preview').textContent,/余弦/);assert.deepEqual(f.wires,[]);
 f.get('mission-profile').value='linear';f.get('mission-profile').change();f.get('mission-base-speed').value='5000';f.get('mission-base-speed').input();
 assert.equal(f.get('mission-start').disabled,true);assert.match(f.get('mission-speed-preview').textContent,/10.*5000/);assert.deepEqual(f.wires,[]);
});

test('mission speed readback is numeric and rejects non-finite firmware values',()=>{
 const s=parseGrabRouteStatus('OK grab route state=paused mode=mission route_base_speed=1000.00 route_scale=1.20 route_speed=1200.00');
 assert.equal(s.route_base_speed,1000);assert.equal(s.route_scale,1.2);assert.equal(s.route_speed,1200);
 assert.equal(parseGrabRouteStatus('OK grab route state=paused mode=mission route_scale=nan'),null);
 assert.match(reasonText('route_scale_range'),/倍率/);
 assert.match(reasonText('route_scale_linear_only'),/余弦/);
 assert.match(reasonText('route_scaled_speed_range'),/10.*5000/);
});

test('adopted profile shows scaled acceleration only from device scale',()=>{
 const f=fixture();f.panel.connection(true);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission profile=linear route_scale=1.2'));
 assert.match(f.get('mission-adopted-profile').textContent,/600\/960\/1200.*360/);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission profile=linear'));
 assert.match(f.get('mission-adopted-profile').textContent,/基准/);
});
test('new mission defaults to 1000 mm/s without transmitting on mount',()=>{
 assert.equal(buildMissionCommand({source:'qr'}),'grab mission qr 1000 100 lower fixed vhigh linear');
 const f=fixture();assert.equal(f.get('mission-visual-heading'),undefined);assert.equal(f.elements.find(e=>e.tag==='input'&&e.value==='1000').value,'1000');assert.deepEqual(f.wires,[]);
});

test('formal linear profile displays per-stage acceleration without changing command defaults',()=>{
 const f=fixture();
 assert.match(f.get('mission-profile').children.find(e=>e.value==='linear').textContent,/500\/800\/1000.*300/);
 f.panel.connection(true);f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission profile=linear'));
 assert.match(f.get('mission-adopted-profile').textContent,/500\/800\/1000.*300/);
 assert.equal(f.get('mission-profile').value,'linear');
 assert.deepEqual(f.wires,[]);
});
test('profile choice is passive, explicit, and locked through pending start and pause',async()=>{
 const f=fixture();f.panel.connection(true);const select=f.get('mission-profile');
 assert.equal(select.value,'linear');assert.equal(select.disabled,false);
 select.value='cosine';select.change();assert.deepEqual(f.wires,[]);
 await f.get('mission-start').onclick();assert.ok(f.wires[0].endsWith(' vhigh cosine'));assert.equal(select.disabled,true);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission profile=cosine'));
 assert.match(f.get('mission-adopted-profile').textContent,/余弦/);assert.equal(select.disabled,true);
 f.panel.connection(false);f.panel.connection(true);assert.equal(select.disabled,true);assert.equal(f.wires.length,1);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=done mode=mission profile=cosine'));assert.equal(select.disabled,false);
 f.panel.reset();assert.match(f.get('mission-adopted-profile').textContent,/未报告/);
 assert.throws(()=>buildMissionCommand({source:'qr',profile:'invalid'}));
 for(const profile of ['linear','cosine']){
  const wire=buildMissionCommand({source:'fixed',code:'156+123+516+231',speed:5000,turnSpeed:360,start:'upper',radar:true,align:'lowered',profile});
  assert.ok(wire.endsWith(` vhigh ${profile}`));assert.ok(wire.length<=95);assert.equal(wire.split(' ').length,10);
 }
});
test('processing progress describes empty ring alignment before pickup',()=>{
 const f=fixture();f.panel.connection(true);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=empty_ring_align mode=mission batch=1 item=1 ring=2 reason=empty_ring_align'));
 assert.ok(f.elements.some(e=>e.textContent.includes('空爪对准目标编号环')));
 assert.deepEqual(f.wires,[]);
});
test('firmware rejection and transport failures release pending profile selection',async()=>{
 const f=fixture();f.panel.connection(true);await f.get('mission-start').onclick();assert.equal(f.get('mission-profile').disabled,true);
 f.panel.receiveReply('ERR grab mission reason=arm_busy');assert.equal(f.get('mission-profile').disabled,false);
 assert.equal(f.get('mission-start-point').disabled,false);assert.equal(f.get('mission-start-smooth').disabled,false);
 assert.ok(f.elements.some(e=>e.textContent.includes('比赛指令未受理')));
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission profile=cosine'));
 f.panel.receiveReply('ERR grab mission reason=arm_busy');assert.equal(f.get('mission-profile').disabled,true);
 for(const sender of [async()=>false,async()=>{throw Error('transport failed');}]){
  const failed=fixture(sender);failed.panel.connection(true);await failed.get('mission-start').onclick();
  assert.equal(failed.get('mission-profile').disabled,false);
  assert.equal(failed.get('mission-start-point').disabled,false);assert.equal(failed.get('mission-start-smooth').disabled,false);
 }
});
test('bus recovery progress remains a match and offers explicit retry',async()=>{
 const f=fixture();f.panel.connection(true);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=bus_recovery_wait mode=mission batch=1 item=1 stored=3 placed=0 bus_recoveries=1 reason=bus_recovery_failed'));
 assert.equal(f.get('mission-start').disabled,true);
 assert.equal(f.get('mission-continue').disabled,false);
 assert.equal(f.get('mission-recover').disabled,false);
 await f.get('mission-recover').onclick();assert.deepEqual(f.wires,['bus recover']);
 assert.ok(f.elements.some(e=>e.textContent.includes('总线恢复')));
});
test('explicit source, valid two-batch mapping, one bounded ASCII command',()=>{
 assert.equal(buildMissionCommand({source:'qr',speed:200,turnSpeed:50}),'grab mission qr 200 50 lower fixed vhigh linear');
 assert.equal(buildMissionCommand({source:'fixed',code:'156+123+516+231',speed:200,turnSpeed:50}),'grab mission fixed 156+123+516+231 200 50 lower fixed vhigh linear');
 for(const code of ['156+123+356+231','116+123+116+231','156+112+516+231'])assert.throws(()=>buildMissionCommand({source:'fixed',code,speed:200,turnSpeed:50}));
 assert.throws(()=>buildMissionCommand({speed:200,turnSpeed:50}));
 assert.equal(buildCommand('grab-mission',{speed:200,turnSpeed:50}),'grab mission qr 200 50 lower fixed vhigh linear');
});
function fixture(sendResult=async()=>true){
 const elements=[],wires=[];const doc={createElement(tag){const e={tag,ownerDocument:doc,children:[],dataset:{},value:'',textContent:'',hidden:false,append(...a){this.children.push(...a);},setAttribute(){},addEventListener(k,fn){this[k]=fn;}};elements.push(e);return e;}};
 const panel=mountMission({root:doc.createElement('div'),send:async wire=>{wires.push(wire);return sendResult(wire);},canSend:()=>true});
 return {panel,wires,elements,get:id=>elements.find(e=>e.id===id)};
}
test('mount, edits, reconnect stay passive; start submits once and preserves device codes',async()=>{
 const f=fixture();f.panel.connection(true);assert.deepEqual(f.wires,[]);
 f.get('mission-source').value='fixed';f.get('mission-source').change();
 f.get('mission-code').value='156+123+516+231';f.get('mission-code').input();assert.deepEqual(f.wires,[]);
 await f.get('mission-start').onclick();assert.deepEqual(f.wires,['grab mission fixed 156+123+516+231 1000 100 lower fixed vhigh linear']);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission code_source=fixed scan_code=356+123+536+231 exec_code=156+123+516+231 batch=1 item=2 slot=2 ring=3 stop_confirmed=1 stored=2 placed=0 reason=operator_stop'));
 assert.equal(f.get('mission-scanned').textContent,'356+123+536+231');assert.equal(f.get('mission-adopted').textContent,'156+123+516+231');
 assert.equal(f.get('mission-continue').disabled,false);
 f.panel.connection(false);f.panel.connection(true);assert.equal(f.wires.length,1);assert.match(f.get('mission-freshness').textContent,/历史/);
 f.panel.reset();assert.equal(f.get('mission-start').disabled,false);assert.equal(f.get('mission-continue').disabled,true);assert.equal(f.get('mission-adopted').textContent,'—');
});
test('automatic firmware preflight needs no placement confirmation and explains feedback wait',async()=>{
 const f=fixture();f.panel.connection(true);
 assert.ok(f.elements.some(e=>e.textContent.includes('无需另行确认或准备跑图')));
 await f.get('mission-start').onclick();assert.deepEqual(f.wires,['grab mission qr 1000 100 lower fixed vhigh linear']);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=preparing_travel mode=mission reason=start_feedback_wait'));
 assert.ok(f.elements.some(e=>e.textContent.includes('等待四轮新鲜')));
 assert.equal(f.get('mission-start').disabled,true);assert.equal(f.get('mission-stop').disabled,false);
 f.panel.connection(false);f.panel.connection(true);assert.equal(f.wires.length,1);
});
test('pause retains continue; stop ends the match and cannot continue',async()=>{
 const f=fixture();f.panel.connection(true);
 assert.equal(f.get('mission-pause').textContent,'暂停比赛');
 assert.equal(f.get('mission-stop').textContent,'停止比赛');
 await f.get('mission-pause').onclick();assert.equal(f.wires.at(-1),'grab mission pause');
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission reason=operator_stop'));
 assert.equal(f.get('mission-continue').disabled,false);
 await f.get('mission-stop').onclick();assert.equal(f.wires.at(-1),'grab mission stop');
 f.panel.receive(parseGrabRouteStatus('OK grab route state=cancelling mode=mission reason=operator_cancelled'));
 assert.equal(f.get('mission-start').disabled,true);assert.equal(f.get('mission-continue').disabled,true);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=cancelled mode=mission reason=operator_cancelled'));
 assert.equal(f.get('mission-start').disabled,false);assert.equal(f.get('mission-continue').disabled,true);
});
test('recovery retry shows retained progress and never enables a second start',()=>{
 const f=fixture();f.panel.connection(true);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=bus_recovery_retry mode=mission reason=bus_recovery_retry auto_resume=1 stored=3 placed=0'));
 assert.equal(f.get('mission-start').disabled,true);
 assert.equal(f.get('mission-pause').disabled,false);
 assert.equal(f.get('mission-diagnostic').title,'bus recovery');
 assert.ok(f.elements.some(e=>e.textContent.includes('重试')));
});

test('start and radar are independent and passive until match start',async()=>{
 for(const start of ['lower','upper'])for(const enabled of [false,true]){
  const f=fixture();f.panel.connection(true);
  f.get('mission-start-point').value=start;f.get('mission-start-point').change();
  f.get('mission-radar').value=enabled?'on':'off';f.get('mission-radar').change();
  assert.deepEqual(f.wires,[]);await f.get('mission-start').onclick();
  assert.deepEqual(f.wires,[`grab mission qr 1000 100 ${start} ${enabled?'radar':'fixed'} vhigh linear`]);
 }
});


test('mission starts always use high alignment and heading, with a360deg/s selectable ceiling',async()=>{
 for(const source of ['qr','fixed'])for(const start of ['upper','lower'])for(const radar of [false,true]){
  const wire=buildMissionCommand({source,code:'156+123+516+231',turnSpeed:360,start,radar});
  assert.ok(wire.endsWith(' vhigh linear'));assert.ok(wire.length<=95);
 }
 assert.throws(()=>buildMissionCommand({source:'qr',turnSpeed:361}));
 assert.throws(()=>buildMissionCommand({source:'qr',turnSpeed:4}));
 const f=fixture();f.panel.connection(true);
 assert.equal(f.get('mission-align-height'),undefined);
 assert.equal(f.get('mission-visual-heading'),undefined);
 const turn=f.elements.find(e=>e.tag==='input'&&e.value==='100');assert.equal(turn.max,'360');
 assert.deepEqual(f.wires,[]);
 await f.get('mission-start').onclick();assert.ok(f.wires[0].endsWith(' vhigh linear'));
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission align=raised visual_heading=on heading_result=heading_uncalibrated heading_image=0.50 heading_error=nan heading_z=0.00'));
 assert.match(f.get('mission-adopted-align').textContent,/高位/);
 assert.match(f.get('mission-adopted-heading').textContent,/开启/);
 assert.match(f.get('mission-heading-result').textContent,/未标定/);
 await f.get('mission-continue').onclick();assert.equal(f.wires.at(-1),'grab mission continue');
 f.panel.connection(false);f.panel.connection(true);assert.equal(f.wires.length,2);
 // Device readback still describes a previous/terminal-started mission accurately.
 f.panel.receive(parseGrabRouteStatus('OK grab route state=done mode=mission align=lowered visual_heading=off'));
 assert.match(f.get('mission-adopted-align').textContent,/下降/);
 assert.match(f.get('mission-adopted-heading').textContent,/关闭/);
 await f.get('mission-start').onclick();assert.ok(f.wires.at(-1).endsWith(' vhigh linear'));
 f.panel.reset();assert.match(f.get('mission-adopted-heading').textContent,/未报告/);
});

test('competition direct heading displays its own10deg/s limit independently of route speed',()=>{
 const f=fixture();f.panel.connection(true);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=station_heading mode=mission visual_heading=on turn_speed=100 heading_source=ring heading_method=direct_ring heading_speed=10 heading_result=heading_turning heading_image=0.45 heading_error=0.45 heading_z=0'));
 assert.match(f.get('mission-heading-result').textContent,/反号.*1[∶:]1/);
 assert.match(f.get('mission-heading-result').textContent,/10(?:\.0)?\s*°\/s/);
 assert.doesNotMatch(f.get('mission-heading-result').textContent,/100\s*°\/s|未标定/);
 assert.deepEqual(f.wires,[]);
});

test('processing departure heading is distinct from arrival and stays passive',()=>{
 const f=fixture();f.panel.connection(true);
 for(const state of ['departure_heading_prepare','departure_heading']){
  const status=parseGrabRouteStatus(`OK grab route state=${state} mode=mission station=2 batch=1 item=3 stored=6 placed=3 visual_heading=on heading_source=ring`);
  assert.match(describeGrabRoute(status),/离站/);f.panel.receive(status);
  assert.equal(f.get('mission-start').disabled,true);
 }
 assert.deepEqual(f.wires,[]);
});
