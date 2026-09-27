import test from 'node:test';
import assert from 'node:assert/strict';
import { commands, buildCommand, frameCommand, LineDecoder, describeReply } from '../protocol.mjs';

test('old firmware uses ASCII lines with CRLF, never the newer # protocol', () => {
  assert.equal(frameCommand('grip open'), 'grip open\r\n');
  assert.equal(buildCommand('pos', {axis:'base', degree:'15'}), 'pos base 15');
  assert.equal(buildCommand('move', {direction:'backward', distance:'50'}), 'chassis backward 50');
});
test('gripper presets show the calibrated pulse widths and keep the wire commands', () => {
  assert.equal(buildCommand('grip-open'), 'grip open');
  assert.equal(buildCommand('grip-catch'), 'grip catch');
  assert.match(commands.find(c => c.id === 'grip-open').label, /500 µs/);
  assert.match(commands.find(c => c.id === 'grip-catch').label, /800 µs/);
  assert.equal(commands.find(c => c.id === 'grip-duty').fields[0].value, 2.5);
});
test('reject missing distance, injection, too-long lines and non-finite values', () => {
  for (const value of ['', 'NaN', 'Infinity', '0', '201', '1.5'])
    assert.throws(() => buildCommand('move', {direction:'backward', distance:value}));
  for (const value of ['help\nstop all', 'help\rstop all', '中文', 'x'.repeat(64), 'a b c d e f g'])
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
  assert.doesNotMatch(describeReply('OK material auto color=1'), /已完成|已到位/);
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
