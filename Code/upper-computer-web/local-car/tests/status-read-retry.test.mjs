import test from 'node:test';
import assert from 'node:assert/strict';
import {formatTerminalLine} from '../protocol.mjs';

test('wheel status retry warning preserves the actual overlapping X completion',()=>{
  const text=formatTerminalLine('WARN bus read_retry count=1 addr=2 func=0x3A rx_bytes=4 hex=07FD9F6B limit=1 motion_replayed=0');
  assert.match(text,/四轮状态查询漏答.*一次补读/);
  assert.match(text,/07FD9F6B/);
  assert.match(text,/0x3A/);
  assert.doesNotMatch(text,/恢复成功|停稳成功|任务完成/);
});
