import test from 'node:test';
import assert from 'node:assert/strict';
import {buildMissionCommand,mountMission} from '../mission-panel.mjs';
import {parseGrabRouteStatus,buildCommand} from '../protocol.mjs';
test('new mission defaults to 1000 mm/s without transmitting on mount',()=>{
 assert.equal(buildMissionCommand({source:'qr'}),'grab mission qr 1000 100 lower fixed raised linear');
 const f=fixture();assert.equal(f.elements.find(e=>e.tag==='input'&&e.value==='1000').value,'1000');assert.deepEqual(f.wires,[]);
});
test('profile choice is passive, explicit, and locked through pending start and pause',async()=>{
 const f=fixture();f.panel.connection(true);const select=f.get('mission-profile');
 assert.equal(select.value,'linear');assert.equal(select.disabled,false);
 select.value='cosine';select.change();assert.deepEqual(f.wires,[]);
 await f.get('mission-start').onclick();assert.ok(f.wires[0].endsWith(' raised cosine'));assert.equal(select.disabled,true);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission profile=cosine'));
 assert.match(f.get('mission-adopted-profile').textContent,/余弦/);assert.equal(select.disabled,true);
 f.panel.connection(false);f.panel.connection(true);assert.equal(select.disabled,true);assert.equal(f.wires.length,1);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=done mode=mission profile=cosine'));assert.equal(select.disabled,false);
 f.panel.reset();assert.match(f.get('mission-adopted-profile').textContent,/未报告/);
 assert.throws(()=>buildMissionCommand({source:'qr',profile:'invalid'}));
 for(const profile of ['linear','cosine']){
  const wire=buildMissionCommand({source:'fixed',code:'156+123+516+231',speed:5000,turnSpeed:500,start:'upper',radar:true,align:'lowered',profile});
  assert.ok(wire.endsWith(` lowered ${profile}`));assert.ok(wire.length<=95);assert.equal(wire.split(' ').length,10);
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
 assert.ok(f.elements.some(e=>e.textContent.includes('比赛指令未受理')));
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission profile=cosine'));
 f.panel.receiveReply('ERR grab mission reason=arm_busy');assert.equal(f.get('mission-profile').disabled,true);
 for(const sender of [async()=>false,async()=>{throw Error('transport failed');}]){
  const failed=fixture(sender);failed.panel.connection(true);await failed.get('mission-start').onclick();
  assert.equal(failed.get('mission-profile').disabled,false);
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
 assert.equal(buildMissionCommand({source:'qr',speed:200,turnSpeed:50}),'grab mission qr 200 50 lower fixed raised linear');
 assert.equal(buildMissionCommand({source:'fixed',code:'156+123+516+231',speed:200,turnSpeed:50}),'grab mission fixed 156+123+516+231 200 50 lower fixed raised linear');
 for(const code of ['156+123+356+231','116+123+116+231','156+112+516+231'])assert.throws(()=>buildMissionCommand({source:'fixed',code,speed:200,turnSpeed:50}));
 assert.throws(()=>buildMissionCommand({speed:200,turnSpeed:50}));
 assert.equal(buildCommand('grab-mission',{speed:200,turnSpeed:50}),'grab mission qr 200 50');
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
 await f.get('mission-start').onclick();assert.deepEqual(f.wires,['grab mission fixed 156+123+516+231 1000 100 lower fixed raised linear']);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission code_source=fixed scan_code=356+123+536+231 exec_code=156+123+516+231 batch=1 item=2 slot=2 ring=3 stop_confirmed=1 stored=2 placed=0 reason=operator_stop'));
 assert.equal(f.get('mission-scanned').textContent,'356+123+536+231');assert.equal(f.get('mission-adopted').textContent,'156+123+516+231');
 assert.equal(f.get('mission-continue').disabled,false);
 f.panel.connection(false);f.panel.connection(true);assert.equal(f.wires.length,1);assert.match(f.get('mission-freshness').textContent,/历史/);
 f.panel.reset();assert.equal(f.get('mission-start').disabled,false);assert.equal(f.get('mission-continue').disabled,true);assert.equal(f.get('mission-adopted').textContent,'—');
});
test('automatic firmware preflight needs no placement confirmation and explains feedback wait',async()=>{
 const f=fixture();f.panel.connection(true);
 assert.ok(f.elements.some(e=>e.textContent.includes('无需另行确认或准备跑图')));
 await f.get('mission-start').onclick();assert.deepEqual(f.wires,['grab mission qr 1000 100 lower fixed raised linear']);
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
  assert.deepEqual(f.wires,[`grab mission qr 1000 100 ${start} ${enabled?'radar':'fixed'} raised linear`]);
 }
});


test('alignment mode is explicit for both code sources and maximum command stays bounded',()=>{
 for(const source of ['qr','fixed'])for(const align of ['lowered','raised']){
  const wire=buildMissionCommand({source,code:'156+123+516+231',speed:5000,turnSpeed:100,start:'upper',radar:true,align});
  assert.ok(wire.endsWith(` upper radar ${align} linear`));assert.ok(wire.length<=95);
 }
 assert.equal(buildMissionCommand({source:'fixed',code:'156+123+516+231',speed:5000,turnSpeed:100,start:'upper',radar:true,align:'lowered'}).length,70);
 assert.throws(()=>buildMissionCommand({source:'qr',align:'typo'}));
});

test('alignment selection sends only on start; paused adopted mode comes from device not draft',async()=>{
 const f=fixture();f.panel.connection(true);
 assert.equal(f.get('mission-align-height')?.value,'raised');
 assert.match(f.get('mission-adopted-align')?.textContent??'',/未报告/);
 f.get('mission-align-height').value='raised';f.get('mission-align-height').change();
 assert.deepEqual(f.wires,[]);
 await f.get('mission-start').onclick();
 assert.deepEqual(f.wires,['grab mission qr 1000 100 lower fixed raised linear']);
 assert.match(f.get('mission-adopted-align').textContent,/未报告/);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission align=raised reason=operator_stop'));
 assert.match(f.get('mission-adopted-align').textContent,/高位.*Z=0/);
 f.get('mission-align-height').value='lowered';f.get('mission-align-height').change();
 assert.match(f.get('mission-adopted-align').textContent,/高位.*Z=0/);
 await f.get('mission-continue').onclick();assert.equal(f.wires.at(-1),'grab mission continue');
 f.panel.connection(false);f.panel.connection(true);assert.equal(f.wires.length,2);
 assert.match(f.get('mission-adopted-align').textContent,/高位/);assert.match(f.get('mission-freshness').textContent,/历史/);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission reason=operator_stop'));
 assert.match(f.get('mission-adopted-align').textContent,/未报告/);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission align=lowered reason=operator_stop'));
 assert.match(f.get('mission-adopted-align').textContent,/下降.*工位高度/);
 f.panel.reset();assert.match(f.get('mission-adopted-align').textContent,/未报告/);
});


test('heading switch is independent, bounded, locked by device and passive',async()=>{
 for(const align of ['lowered','raised'])for(const start of ['upper','lower'])for(const radar of [false,true]){
  const wire=buildMissionCommand({source:'fixed',code:'156+123+516+231',speed:5000,turnSpeed:100,start,radar,align,visualHeading:true});
  assert.ok(wire.endsWith(align==='raised'?' vhigh linear':' vlow linear'));assert.ok(wire.length<=95);
 }
 const f=fixture();f.panel.connection(true);
 assert.equal(f.get('mission-visual-heading')?.value,'off');
 f.get('mission-visual-heading').value='on';f.get('mission-visual-heading').change();assert.deepEqual(f.wires,[]);
 await f.get('mission-start').onclick();assert.ok(f.wires[0].endsWith(' vhigh linear'));
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission visual_heading=on heading_result=heading_uncalibrated heading_image=0.50 heading_error=nan heading_z=0.00'));
 assert.match(f.get('mission-adopted-heading').textContent,/开启/);
 assert.match(f.get('mission-heading-result').textContent,/未标定/);
 assert.match(f.get('mission-heading-result').textContent,/观察Z 0.00 mm/);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission visual_heading=on heading_source=line heading_result=heading_uncalibrated heading_image=0.50 heading_z=0.00'));
 assert.match(f.get('mission-heading-result').textContent,/黑线/);
 f.get('mission-visual-heading').value='off';f.get('mission-visual-heading').change();
 assert.match(f.get('mission-adopted-heading').textContent,/开启/);
 await f.get('mission-continue').onclick();assert.equal(f.wires.at(-1),'grab mission continue');
 f.panel.connection(false);f.panel.connection(true);assert.equal(f.wires.length,2);
 f.panel.receive(parseGrabRouteStatus('OK grab route state=paused mode=mission'));
 assert.match(f.get('mission-adopted-heading').textContent,/未报告/);
});
