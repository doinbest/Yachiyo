import test from 'node:test';
import assert from 'node:assert/strict';
import {commands, buildCommand, moduleForWire, describeReply} from '../protocol.mjs';

test('distance acceleration tests normalize diagonals and leave legacy timed controls separate', () => {
  const vectors={forward:[1,0],back:[-1,0],left:[0,1],right:[0,-1],fl:[1,1],fr:[1,-1],bl:[-1,1],br:[-1,-1]};
  for(const [direction,[x,y]] of Object.entries(vectors)){
    const wire=buildCommand('chassis-test',{direction,distance:3000,speed:1000});
    const [,verb,f,l,v]=wire.split(' ');
    assert.equal(verb,'test');assert.equal(Number(v),1000);
    assert.equal(Math.sign(Number(f)),x);assert.equal(Math.sign(Number(l)),y);
    assert.ok(Math.hypot(Number(f),Number(l))<=3000);
    assert.ok(Math.hypot(Number(f),Number(l))>2999.99);
  }
  for(const changed of [{distance:0},{distance:3001},{speed:9},{speed:5001},{speed:'NaN'},{direction:'oops'}])
    assert.throws(()=>buildCommand('chassis-test',{direction:'forward',distance:500,speed:1000,...changed}));
  assert.equal(buildCommand('chassis-test',{direction:'back',distance:500,speed:1000}),'chassis test -500 0 1000 linear 500 0.02 0 1 300');
  assert.equal(buildCommand('chassis-test',{direction:'forward',distance:1000,speed:1000,accel:800}),'chassis test 1000 0 1000 linear 800 0.02 0');
  for(const accel of [0,9,5001,'NaN','',100.5])assert.throws(()=>buildCommand('chassis-test',{direction:'forward',distance:500,speed:1000,accel}));
  assert.equal(buildCommand('chassis-test',{direction:'forward',distance:500,speed:1000,profile:'cosine',accel:''}),'chassis test 500 0 1000 cosine 200 0.02 0');
  assert.equal(buildCommand('chassis-test',{direction:'forward',distance:500,speed:1000,profile:'cosine'}),'chassis test 500 0 1000 cosine 200 0.02 0');
  assert.throws(()=>buildCommand('chassis-test',{direction:'forward',distance:500,speed:1000,profile:'unknown'}));
});

test('finite velocity controls convert seconds to integer milliseconds once', () => {
  assert.equal(buildCommand('chassis-run', {vx:60,vy:80,omega_deg:500,seconds:1.25}), 'chassis run 60 80 8.726646 1250');
  assert.equal(buildCommand('chassis-run', {vx:0,vy:0,omega_deg:-500,seconds:0.1}), 'chassis run 0 0 -8.726646 100');
  assert.equal(buildCommand('chassis-heading', {vx:10,vy:0,heading:-90,seconds:59}), 'chassis heading 10 0 -90 59000');
  for (const changed of [{vx:300,vy:1},{vx:301},{vy:-301},{omega_deg:500.1},{omega_deg:-500.1},{seconds:0.09},{seconds:59.001},{seconds:'NaN'}]) {
    assert.throws(() => buildCommand('chassis-run', {vx:0,vy:0,omega_deg:0,seconds:1,...changed}));
  }
  assert.equal(buildCommand('chassis-run',{vx:0,vy:0,omega_deg:30.5,seconds:1}),'chassis run 0 0 0.532325 1000');
  const angleField=commands.find(c=>c.id==='chassis-run').fields.find(f=>f.key==='omega_deg');
  assert.match(angleField.label,/°/);
  assert.equal(angleField.max,500);
  assert.throws(() => buildCommand('chassis-heading',{vx:0,vy:0,heading:'',seconds:1}));
  assert.throws(() => buildCommand('chassis-heading',{vx:300,vy:1,heading:0,seconds:1}));
});

test('live translation controls accept 300 mm/s and preserve the resultant speed limit', () => {
  for (const [vx,vy] of [[300,0],[-300,0],[0,300],[0,-300],[180,240],[-180,-240]]) {
    assert.equal(buildCommand('chassis-run',{vx,vy,omega_deg:0,seconds:1}),`chassis run ${vx} ${vy} 0 1000`);
    assert.equal(buildCommand('chassis-heading',{vx,vy,heading:90,seconds:1}),`chassis heading ${vx} ${vy} 90 1000`);
  }
  for (const id of ['chassis-run','chassis-heading']) {
    const fields = commands.find(c => c.id === id).fields;
    for (const key of ['vx','vy']) {
      assert.equal(fields.find(f => f.key === key).min,-300);
      assert.equal(fields.find(f => f.key === key).max,300);
    }
    assert.throws(() => buildCommand(id,{vx:300,vy:300,omega_deg:0,heading:90,seconds:1}),/300 mm\/s/);
  }
});

test('module heading and map origin remain separate commands; confirmation fields start empty', () => {
  assert.equal(buildCommand('chassis-origin',{x:1200,y:400,heading:90}), 'chassis origin 1200 400 90');
  for (const id of ['chassis-heading','chassis-origin','units']) {
    const definition=commands.find(c=>c.id===id);
    assert.equal(definition.requiresInput,true);
    assert.throws(()=>buildCommand(id,Object.fromEntries(definition.fields.map(f=>[f.key,f.value]))));
  }
  assert.equal(buildCommand('units',{units:'0'}),'chassis units 0');
  assert.equal(buildCommand('units',{units:'65536'}),'chassis units 65536');
  for (const units of ['16384','3200','65535']) assert.throws(()=>buildCommand('units',{units}));
  assert.equal(buildCommand('reset-confirmed'),'chassis reset-confirmed');
});

test('read-only and subscription commands generate their exact independent wires', () => {
  const fixed={
    'chassis-task':'chassis task','feedback-read':'chassis feedback',
    'chassis-status':'chassis status',
    'feedback-on':'chassis feedback on','feedback-off':'chassis feedback off',
    'qr-status':'qr status','qr-read':'qr read','qr-stream-on':'qr stream on','qr-stream-off':'qr stream off',
    'imu-status':'imu status','imu-cal-start':'imu cal start','imu-cal-cancel':'imu cal cancel',
    'imu-verify':'imu verify',
    'imu-stream-on':'imu stream on','imu-stream-off':'imu stream off',
  };
  for (const [id,wire] of Object.entries(fixed)) assert.equal(buildCommand(id),wire);
  for (const wheel of ['0','1','2','3','4']) assert.equal(buildCommand('feedback-select',{wheel}),`chassis feedback ${wheel}`);
  assert.throws(()=>buildCommand('feedback-select',{wheel:'5'}));
  assert.equal(buildCommand('chassis-hold',{speed:-30,seconds:2}),'chassis hold -30 2');
  for (const seconds of [0,0.5,11]) assert.throws(()=>buildCommand('chassis-hold',{speed:0,seconds}));
});

test('module classification preserves favorite wire text and refuses to infer generic replies', () => {
  const mapping={
    '  grip open  ':'arm','pos x 15':'arm','chassis run 0 0 0 100':'chassis',
    'wheel fl 10':'chassis','OK wheel=1 raw_rpm=0':'chassis',
    'chassis stream on 123':'map','@CHASSIS {"v":1}':'map',
    'material auto 1':'vision','OK material state=ALIGNED':'vision',
    'EVT qr code seq=1 code=123+231+312+123 age_ms=0':'qr','qr read':'qr','@QR seq=1':'qr','imu status':'system','[IMU CAL] state=DONE':'system',
    'OK accepted':'manual','ERR busy':'manual','future instruction':'manual',
  };
  for (const [wire,module] of Object.entries(mapping)) assert.equal(moduleForWire(wire),module,wire);
  assert.equal(moduleForWire(null),'manual');
  assert.equal(commands.find(c=>c.id==='grip-open').group,'grip');
  assert.equal(commands.find(c=>c.id==='grip-open').owner,'arm');
  assert.equal(commands.find(c=>c.id==='camera-material').section,'debug');
  assert.equal(commands.find(c=>c.id==='grab-calib').section,'calibration');
  assert.equal(commands.some(c=>c.wire?.startsWith('vision ')||c.id.startsWith('vision-')),false);
  assert.equal(buildCommand('grab-calib-ref'),'grab calib ref');
  assert.equal(buildCommand('grab-calib',{target:'3'}),'grab calib 3');
  assert.equal(commands.some(c=>c.id.startsWith('material-')),false);
  assert.doesNotMatch(describeReply('OK grab state=hold mode=align'),/搬运完成|已到位/);
});

test('aligned is alignment only',()=>{assert.equal(describeReply('OK grab state=hold mode=align'),'对准动作完成 · 未执行抓取');assert.doesNotMatch(describeReply('OK grab state=acquire'),/完成/);});

test('real route controls send one exact command and map displacement is nonzero up to 300 mm',()=>{
  for(const action of ['start','next','cancel','status']) {
    assert.equal(buildCommand(`chassis-route-${action}`,action==='start'?{speed:5000}:{}),`chassis route ${action}${action==='start'?' 5000':''}`);
    assert.equal(moduleForWire(`chassis route ${action}`),'chassis');
  }
  for(const [dx,dy] of [[100,0],[-150,150],[0,-300],[180,240],[0.1,0]])
    assert.equal(buildCommand('chassis-move',{dx,dy}),`chassis move ${dx} ${dy}`);
  for(const [dx,dy] of [[0,0],[300,1],[301,0],[250,250],['',100],['NaN',0],['100\nhelp',0]])
    assert.throws(()=>buildCommand('chassis-move',{dx,dy}));
  assert.equal(describeReply('OK chassis route state=waiting segment=1 reason=reached'),'收到设备回复');
  assert.doesNotMatch(describeReply('OK chassis route start accepted'),/已到位|完成/);
});

test('real route speed is explicit and bounded independently of simulation',()=>{
 assert.equal(buildCommand('chassis-route-start',{speed:5000}),'chassis route start 5000');
 assert.throws(()=>buildCommand('chassis-route-start',{speed:5001}));
 assert.throws(()=>buildCommand('chassis-route-start',{speed:0}));
});

test('full automatic route is an explicit command with bounded speed',()=>{
 assert.equal(buildCommand('chassis-route-auto',{speed:5000}),'chassis route auto 5000');
 assert.throws(()=>buildCommand('chassis-route-auto',{speed:5001}));
});

test('ID8 manual setup and nearest home use dedicated commands and arm replies',()=>{
  for(const op of ['enable','disable','save_zero','home']){
    assert.equal(buildCommand('turntable-'+op),'turntable '+op);
    assert.equal(moduleForWire('OK turntable accepted action='+op),'arm');
  }
});

test('direct onboard arm release has its own command independent of tray index',()=>{
  assert.equal(buildCommand('grab-place-car'),'grab place car');
  assert.equal(buildCommand('turntable-index',{slot:2}),'turntable index 2');
});

test('short axis diagnostics are read-only while stop is latched',async()=>{
  const {readOnlyCommand}=await import('../bridge.mjs');
  assert.equal(buildCommand('grab-base-status'),'grab status base');
  for(const axis of ['base','x','z'])assert.equal(readOnlyCommand('grab status '+axis),true);
  assert.equal(readOnlyCommand('grab status all'),false);
});

test('ring automatic tray placement sends one firmware command with no color or height draft',()=>{
 assert.equal(buildCommand('grab-ring-take',{slot:'2'}),'grab ring take 2');
 assert.throws(()=>buildCommand('grab-ring-take',{slot:'4'}));
});
