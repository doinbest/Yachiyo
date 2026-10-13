import test from 'node:test';
import assert from 'node:assert/strict';
import { commands, buildCommand, frameCommand, LineDecoder, describeReply, moduleForWire } from '../protocol.mjs';
import {buildMissionCommand} from '../mission-panel.mjs';

test('mission start frames preserve origin and route mode for both code sources', () => {
  const cases = [
    [{source:'qr',speed:500,turnSpeed:500,start:'lower',radar:false}, 'grab mission qr 500 500 lower fixed raised\r\n'],
    [{source:'fixed',code:'156+123+516+231',speed:500,turnSpeed:100,start:'upper',radar:true,align:'raised'}, 'grab mission fixed 156+123+516+231 500 100 upper radar raised\r\n'],
  ];
  cases.push([{source:'fixed',code:'156+123+516+231',speed:5000,turnSpeed:100,start:'upper',radar:true,align:'lowered'}, 'grab mission fixed 156+123+516+231 5000 100 upper radar lowered\r\n']);
  for (const [request, expected] of cases)
    assert.equal(frameCommand(buildMissionCommand(request)), expected);
});

test('bus recovery controls use exact ASCII commands and system log category',()=>{
  assert.equal(buildCommand('bus-recover'),'bus recover');
  assert.equal(buildCommand('bus-status'),'bus status');
  assert.equal(moduleForWire('OK bus recover complete arm_ready=1'),'system');
  assert.equal(moduleForWire('ERR bus recover reason=reply_timeout'),'system');
});
test('car tray pickup selects a slot and defaults to 40mm without a release command',()=>{
  const c=commands.find(c=>c.id==='grab-take-work');
  assert.ok(c);assert.equal(c.fields.find(f=>f.key==='depth').value,40);
  assert.match(c.note,/1\/2号盘.*X30mm/);assert.match(c.note,/3号盘保持收回/);
  assert.doesNotMatch(c.note,/X到取放口/);
  assert.equal(buildCommand(c.id,{slot:'3',depth:40}),'grab take 3 work 40');
  assert.equal(buildCommand(c.id,{slot:'2',depth:60}),'grab take 2 work 60');
  for(const depth of [0,-40,151,'NaN'])assert.throws(()=>buildCommand(c.id,{slot:'1',depth}));
});

test('old firmware uses ASCII lines with CRLF, never the newer # protocol', () => {
  assert.equal(frameCommand('grip open'), 'grip open\r\n');
  assert.equal(buildCommand('pos', {axis:'base', degree:'15'}), 'pos base 15');
  assert.equal(buildCommand('move', {direction:'backward', distance:'50'}), 'chassis backward 50');
});
test('gripper presets show the calibrated pulse widths and keep the wire commands', () => {
  assert.equal(buildCommand('grip-open'), 'grip open');
  assert.equal(buildCommand('grip-catch'), 'grip catch');
  assert.match(commands.find(c => c.id === 'grip-open').label, /2500 µs/);
  assert.match(commands.find(c => c.id === 'grip-catch').label, /1920 µs/);
  assert.equal(commands.find(c => c.id === 'grip-duty').fields[0].value, 9.6);
  assert.match(commands.find(c => c.id === 'grip-duty').note, /1920 µs = 9.6%/);
});
test('reject missing distance, injection, too-long lines and non-finite values', () => {
  for (const value of ['', 'NaN', 'Infinity', '0', '201', '1.5'])
    assert.throws(() => buildCommand('move', {direction:'backward', distance:value}));
  for (const value of ['help\nstop all', 'help\rstop all', '中文', 'x'.repeat(64), 'a b c d e f g h i j'])
    assert.throws(() => frameCommand(value));
  assert.throws(() => buildCommand('pos', {axis:'all', degree:'10'}));
  assert.throws(() => buildCommand('pos', {axis:'x', degree:'361'}));
  assert.throws(() => buildCommand('wheel', {wheel:'fl', rpm:'0'}));
});
test('catalog defaults generate valid old-firmware frames', () => {
  for (const command of commands) {
    const values = Object.fromEntries((command.fields ?? []).map(f => [f.key, String(f.value)]));
    if (command.requiresInput) {
      assert.throws(() => buildCommand(command.id, values), command.id);
      continue;
    }
    assert.ok(frameCommand(buildCommand(command.id, values)).endsWith('\r\n'), command.id);
  }
});

test('manual Z accepts ten turns while Base and X retain single-turn limits', () => {
  for (const degree of [-3600, -1080, 1080, 3600])
    assert.equal(buildCommand('pos', {axis:'z', degree}), `pos z ${degree}`);
  for (const degree of [-3600.1, 3600.1, 0])
    assert.throws(() => buildCommand('pos', {axis:'z', degree}));
  for (const axis of ['base','x']) {
    for (const degree of [-360,360])
      assert.equal(buildCommand('pos', {axis,degree}), `pos ${axis} ${degree}`);
    for (const degree of [-360.1,360.1,3600])
      assert.throws(() => buildCommand('pos', {axis,degree}));
  }
});
test('RX handles fragmented CRLF, multiple lines, prompts and bounded unterminated input', () => {
  const decoder = new LineDecoder();
  assert.deepEqual(decoder.push('arm> OK pos base accepted\r'), ['OK pos base accepted']);
  assert.deepEqual(decoder.push('\nERR busy\nOK state base reached=1\r\narm> '), ['ERR busy','OK state base reached=1']);
  assert.equal(decoder.push('x'.repeat(9000)).join('').length < 5000, true);
});
test('accepted is not completion and state is reported only from explicit feedback', () => {
  assert.match(describeReply('OK pos base degree=15 accepted'), /接受.*到位/);
  assert.match(describeReply('OK state base raw=0x03 en=1 reached=1'), /到位/);
  assert.match(describeReply('ERR busy'), /拒绝/);
  assert.doesNotMatch(describeReply('OK grab calib accepted'), /已完成|已到位/);
});

 test('reset and yaw zero are distinct writable system commands', async () => {
  const {moduleForWire}=await import('../protocol.mjs');
  const {readOnlyCommand}=await import('../bridge.mjs');
  for(const [id,wire] of [['system-reset','system reset'],['imu-zero','imu zero']]) {
    assert.equal(buildCommand(id,{}),wire);
    assert.equal(moduleForWire(wire),'system');
    assert.equal(moduleForWire(`OK ${wire} pending`),'system');
    assert.equal(readOnlyCommand(wire),false);
  }
 });
