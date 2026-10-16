import test from 'node:test';
import assert from 'node:assert/strict';
import {parseChassisTestResult,mountChassisTestResult} from '../chassis-test-result.mjs';
const result=(id=7,extra='')=>`OK chassis test_result id=${id} state=3 profile=cosine elapsed_ms=2500 peak_mm_s=701.2 error_mm=1.5 stop_ms=103 stop_confirmed=1${extra}`;
const firmwareAck='OK chassis test reason=accepted id=1 profile=linear distance_mm=1000.000 speed_mm_s=1000.0; request only, arrival requires feedback';
function fixture(){
 const elements=[];const doc={createElement(tag){const e={tag,ownerDocument:doc,children:[],textContent:'',append(...a){this.children.push(...a);},setAttribute(){}};elements.push(e);return e;}};
 const panel=mountChassisTestResult({root:doc.createElement('div')});panel.connection(true);
 return {panel,elements,status:()=>elements.find(e=>e.tag==='p').textContent,metrics:()=>elements.find(e=>e.tag==='dl'),values:()=>elements.filter(e=>e.tag==='dd').map(e=>e.textContent)};
}
test('linear result displays device acceleration, independent of the requested draft',()=>{
 const f=fixture();f.panel.requested('chassis test 1000 0 1000 linear 800');
 f.panel.receive(firmwareAck+'');
 f.panel.receive(result(1).replace('profile=cosine','profile=linear')+' accel_mm_s2=500.0');
 assert.equal(f.values()[0],'线性 · 500 mm/s²');
 assert.equal(parseChassisTestResult(result()+' accel_mm_s2=nan'),null);
});
test('cosine result and heading gains display adopted values without using the form draft',()=>{
 const f=fixture();f.panel.requested('chassis test 1000 0 1000 cosine 200 0.02 0');
 f.panel.receive('OK chassis test reason=accepted id=7 profile=cosine cosine_k=400 heading_kp=0.1 heading_kd=0.01');
 f.panel.receive(result()+' cosine_k=400 heading_kp=0.1 heading_kd=0.01');
 assert.equal(f.values()[0],'余弦 · K400 / 800 ms');
 assert.ok(f.elements.some(e=>e.textContent==='设备采用航向 P=0.1，D=0.01'));
 for(const extra of [' cosine_k=nan',' heading_kp=-1',' heading_kd=1.1'])assert.equal(parseChassisTestResult(result()+extra),null);
 f.panel.reset();assert.equal(f.elements.find(e=>e.textContent==='设备采用航向 P=0.1，D=0.01').hidden,true);
});
test('test result parses separate scalar metrics and rejects other or malformed replies',()=>{
 assert.equal(parseChassisTestResult('arm> '+result()).error_mm,1.5);
 for(const s of ['OK chassis distance id=7 error_mm=1,2',result().replace('error_mm=1.5','error_mm=1,2'),result().replace('peak_mm_s=701.2','peak_mm_s=nan'),result(7,' id=8'),result().replace('profile=cosine','profile=invalid')])assert.equal(parseChassisTestResult(s),null);
});
test('exact firmware ACK suffix permits binding and preserves duplicate-field checks',()=>{
 const f=fixture();f.panel.requested('chassis test 1000 0 1000 linear');
 const parsed=parseChassisTestResult(firmwareAck);assert.equal(parsed.id,1);assert.equal(parsed.profile,'linear');
 f.panel.receive(firmwareAck);f.panel.receive(result(1).replace('profile=cosine','profile=linear'));
 assert.equal(f.metrics().hidden,false);assert.match(f.status(),/已完成/);assert.equal(f.values()[0],'线性');
 assert.equal(parseChassisTestResult(firmwareAck.replace('id=1','id=1 id=2')),null);
 assert.equal(parseChassisTestResult(result()+'; request only, arrival requires feedback'),null);
});
test('rejected test ACK ends pending status, displays reason and ignores late acceptance',()=>{
 const f=fixture();f.panel.requested('chassis test 1000 0 1000 linear');
 f.panel.receive(firmwareAck.replace('OK chassis test reason=accepted','ERR chassis test reason=feedback_stale'));
 assert.match(f.status(),/未受理/);assert.doesNotMatch(f.status(),/等待设备受理/);
 f.panel.receive(firmwareAck);f.panel.receive(result(1));assert.equal(f.metrics().hidden,true);
 f.panel.requested('chassis test 1000 0 1000 linear');f.panel.receive(firmwareAck);f.panel.receive(result(1));assert.equal(f.metrics().hidden,false);
});
test('results bind only to explicit test ACK and ignore old or other actions',()=>{
 const f=fixture();f.panel.requested('chassis task');f.panel.receive(result());assert.equal(f.metrics().hidden,true);
 f.panel.requested('chassis test 500 0 1000 cosine');f.panel.receive(result());assert.equal(f.metrics().hidden,true);
 f.panel.receive('OK chassis test reason=accepted id=7 profile=cosine');f.panel.receive(result(6));assert.equal(f.metrics().hidden,true);
 f.panel.receive(result());assert.deepEqual(f.values(),['余弦','2500 ms','701.2 mm/s','1.5 mm','103 ms']);assert.match(f.status(),/已完成/);
 f.panel.requested('chassis test 500 0 1000 linear');assert.equal(f.metrics().hidden,true);f.panel.receive(result());assert.equal(f.metrics().hidden,true);
});
test('running measurements do not claim endpoint or confirmed stop and reconnect keeps history',()=>{
 const f=fixture();f.panel.requested('chassis test 500 0 1000 cosine');f.panel.receive('OK chassis test reason=accepted id=7 profile=cosine');
 f.panel.receive(result().replace('state=3','state=1').replace('stop_confirmed=1','stop_confirmed=0'));
 assert.equal(f.values()[3],'待完成');assert.equal(f.values()[4],'待确认');
 f.panel.receive(result());f.panel.connection(false);assert.match(f.status(),/历史/);
 f.panel.receive(result(8));assert.match(f.status(),/#7/);f.panel.connection(true);assert.match(f.status(),/历史/);
 f.panel.requested('chassis task');f.panel.receive(result(8));assert.match(f.status(),/历史/);
 f.panel.receive(result());assert.match(f.status(),/历史/);
 f.panel.requested('chassis test 500 0 1000 cosine');f.panel.receive('OK chassis test reason=accepted id=7 profile=cosine');f.panel.receive(result());assert.match(f.status(),/设备读回/);
 f.panel.reset();f.panel.receive(result());assert.equal(f.metrics().hidden,true);
});
test('a new preparation marks prior measurements as history until its own ACK arrives',()=>{
 const f=fixture();f.panel.requested('chassis test 500 0 1000 cosine');
 f.panel.receive('OK chassis test reason=accepted id=7 profile=cosine');f.panel.receive(result());
 f.panel.preparing();assert.match(f.status(),/历史/);
 f.panel.receive(result());assert.match(f.status(),/历史/);
});
test('manual stop retires the prior result binding and labels cached readings',()=>{
 const f=fixture();f.panel.requested('chassis test 500 0 1000 cosine');
 f.panel.receive('OK chassis test reason=accepted id=7 profile=cosine');
 f.panel.receive(result().replace('state=3','state=1').replace('stop_confirmed=1','stop_confirmed=0'));
 f.panel.interrupted();assert.match(f.status(),/停止前/);
 f.panel.receive(result());assert.match(f.status(),/停止前/);assert.doesNotMatch(f.status(),/已完成/);
});
