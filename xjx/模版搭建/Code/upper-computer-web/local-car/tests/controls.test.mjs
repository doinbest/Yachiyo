import test from 'node:test';
import assert from 'node:assert/strict';
import {commands, buildCommand, moduleForWire, describeReply} from '../protocol.mjs';

test('finite velocity controls convert seconds to integer milliseconds once', () => {
  assert.equal(buildCommand('chassis-run', {vx:60,vy:80,omega:0.15,seconds:1.25}), 'chassis run 60 80 0.15 1250');
  assert.equal(buildCommand('chassis-run', {vx:0,vy:0,omega:-0.15,seconds:0.1}), 'chassis run 0 0 -0.15 100');
  assert.equal(buildCommand('chassis-heading', {vx:10,vy:0,heading:-90,seconds:59}), 'chassis heading 10 0 -90 59000');
  for (const changed of [{vx:80,vy:80},{omega:0.151},{seconds:0.09},{seconds:59.001},{seconds:'NaN'}]) {
    assert.throws(() => buildCommand('chassis-run', {vx:0,vy:0,omega:0,seconds:1,...changed}));
  }
  assert.throws(() => buildCommand('chassis-heading',{vx:0,vy:0,heading:'',seconds:1}));
  assert.throws(() => buildCommand('chassis-heading',{vx:80,vy:80,heading:0,seconds:1}));
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
  assert.equal(commands.find(c=>c.id==='vision-calib-material').section,'calibration');
  assert.equal(buildCommand('material-auto',{target:'3'}),'material auto 3');
  assert.doesNotMatch(describeReply('OK material state=ALIGNED'),/搬运完成|已到位/);
});

test('aligned is alignment only',()=>{assert.equal(describeReply('OK material state=ALIGNED'),'对准完成');assert.doesNotMatch(describeReply('OK material state=SEARCHING'),/完成/);});

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
