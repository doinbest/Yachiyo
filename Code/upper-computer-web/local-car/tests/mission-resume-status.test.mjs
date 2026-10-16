import test from 'node:test';
import assert from 'node:assert/strict';
import {mountMission} from '../mission-panel.mjs';
import {parseGrabRouteStatus,describeGrabRoute,formatTerminalLine} from '../protocol.mjs';

function fixture(){
  const elements=[],wires=[];
  const doc={createElement(tag){const e={tag,ownerDocument:doc,children:[],dataset:{},value:'',textContent:'',append(...a){this.children.push(...a);},setAttribute(){},addEventListener(k,fn){this[k]=fn;}};elements.push(e);return e;}};
  const panel=mountMission({root:doc.createElement('div'),send:async wire=>{wires.push(wire);return true;},canSend:()=>true});
  panel.connection(true);
  return {panel,elements,wires};
}

test('confirmed parking never appears as unconfirmed in route, panel, or terminal',()=>{
  const raw='OK grab route state=stop_wait mode=mission batch=1 item=1 stop_confirmed=1 reason=feedback_invalid';
  const snapshot=parseGrabRouteStatus(raw),f=fixture();f.panel.receive(snapshot);
  for(const text of [describeGrabRoute(snapshot),formatTerminalLine(raw),...f.elements.map(e=>e.textContent)]){
    assert.doesNotMatch(text,/停车未确认|停车尚未确认/);
  }
  assert.match(describeGrabRoute(snapshot),/已停稳.*定位反馈/);
  assert.ok(f.elements.some(e=>/已停稳.*定位反馈/.test(e.textContent)));
  assert.match(formatTerminalLine(raw),/已停稳.*定位反馈/);
  assert.deepEqual(f.wires,[]);
});

test('missing parking proof and invalid reference remain distinct',()=>{
  for(const [confirmed,reason,expected] of [[0,'feedback_invalid',/停车待确认/],[1,'resume_origin_invalid',/定位参考失效/],[1,'station_reference_changed',/定位参考.*变化/]]){
    const snapshot=parseGrabRouteStatus(`OK grab route state=stop_wait mode=mission stop_confirmed=${confirmed} reason=${reason}`);
    assert.match(describeGrabRoute(snapshot),expected);
  }
});

test('resume feedback wait reports actual progress without automatic UI commands',()=>{
  const f=fixture();
  f.panel.receive(parseGrabRouteStatus('OK grab route state=resuming mode=mission stop_confirmed=1 reason=resume_feedback_wait'));
  assert.ok(f.elements.some(e=>/等待.*定位反馈/.test(e.textContent)));
  assert.deepEqual(f.wires,[]);
});

test('recovery instruction follows the actual automatic continuation flag',()=>{
  const f=fixture();
  for(const automatic of [0,1]){
    f.panel.receive(parseGrabRouteStatus(`OK grab route state=bus_recovery_verify mode=mission auto_resume=${automatic} reason=bus_recovery_verify`));
    assert.ok(f.elements.some(e=>e.textContent.includes(automatic?'验证成功后自动继续':'验证成功后保持暂停')));
  }
  assert.deepEqual(f.wires,[]);
});

test('IMU wait retains the match and separates parking proof from recovered pause',async()=>{
  const f=fixture(),get=id=>f.elements.find(e=>e.id===id);
  for(const confirmed of [0,1]){
    const raw=`OK grab route state=imu_recovery_wait mode=mission batch=2 item=2 stored=9 placed=7 stop_confirmed=${confirmed} reason=imu_recovery_wait`;
    const snapshot=parseGrabRouteStatus(raw);f.panel.receive(snapshot);
    assert.equal(get('mission-start').disabled,true);
    assert.equal(get('mission-continue').disabled,true);
    assert.equal(get('mission-recover').disabled,true);
    assert.equal(get('mission-stop').disabled,false);
    for(const text of [describeGrabRoute(snapshot),formatTerminalLine(raw)]){
      assert.match(text,/IMU/);
      assert.doesNotMatch(text,/imu_recovery_wait/);
      assert.match(text,confirmed?/已停稳/:/停车待确认/);
    }
    assert.ok(f.elements.some(e=>/入仓 9.*放置 7/.test(e.textContent)));
  }
  assert.deepEqual(f.wires,[]);
  const raw='OK grab route state=paused mode=mission stop_confirmed=1 reason=imu_recovered auto_resume=0';
  f.panel.receive(parseGrabRouteStatus(raw));
  assert.equal(get('mission-continue').disabled,false);
  assert.ok(f.elements.some(e=>/IMU已恢复.*暂停/.test(e.textContent)));
  assert.ok(f.elements.some(e=>/显式.*继续|点击.*继续/.test(e.textContent)));
  assert.match(formatTerminalLine(raw),/IMU.*恢复/);
  assert.deepEqual(f.wires,[]);
  await get('mission-continue').onclick();assert.deepEqual(f.wires,['grab mission continue']);
});

test('IMU recovery reports real wheel faults and reference changes without claiming ready',()=>{
  const f=fixture();
  for(const [reason,want] of [['resume_wheel_fault',/四轮驱动.*保护/],['resume_wheels_disabled',/四轮.*未使能/],
    ['station_reference_changed',/参考.*变化/],['resume_origin_invalid',/定位参考失效/]]){
    const raw=`OK grab route state=stop_wait mode=mission stop_confirmed=1 reason=${reason}`;
    const snapshot=parseGrabRouteStatus(raw);f.panel.receive(snapshot);
    assert.match(formatTerminalLine(raw),want);
    assert.ok(f.elements.some(e=>want.test(e.textContent)));
    assert.doesNotMatch(formatTerminalLine(raw),/imu_recovered|IMU已恢复/);
  }
  assert.deepEqual(f.wires,[]);
});
