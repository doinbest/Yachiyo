import test from 'node:test';
import assert from 'node:assert/strict';
import { parseQrLine, QrState } from '../qr-panel.mjs';

const status='OK qr status valid=1 seq=7 age_ms=25 received=12 accepted=8 rejected=4 stream=off event_dropped=2';
const read='OK qr read valid=1 seq=7 code=123+231+312+123 age_ms=25';
function ready(){const state=new QrState();state.connection(true);assert.equal(state.request('qr status',0),true);assert.equal(state.receive(status,1),true);return state;}

test('QR parser accepts status counters and four explicit code groups',()=>{
  const parsed=parseQrLine(status);assert.equal(parsed.kind,'status');assert.equal(parsed.stats.event_dropped,2);
  assert.equal(parseQrLine(read).code,'123+231+312+123');
  assert.equal(parseQrLine('OK qr read valid=0 seq=0 code=NA age_ms=NA').code,null);
  assert.equal(parseQrLine('EVT qr code seq=4294967295 code=000+999+123+456 age_ms=NA').ageMs,null);
});
test('malformed, unrelated and out of range replies cannot populate QR state',()=>{
  for(const line of ['OK material aligned','OK qr status valid=1 seq=-1 age_ms=0','OK qr status valid=1 seq=4294967296 age_ms=0','OK qr status valid=1 seq=1 age_ms=4294967296',read.replace('123+231+312+123','123+231'),read.replace('123+231+312+123','123+231+312+12x'),`${read} seq=8`,'OK qr status valid=2 seq=0 age_ms=NA'])assert.equal(parseQrLine(line),null,line);
});
test('connection and navigation state send nothing and require explicit support check',()=>{
  const state=new QrState();assert.equal(state.request('qr status',0),false);state.connection(true);
  assert.equal(state.code,null);assert.equal(state.pending,null);assert.equal(state.stream,false);
  assert.equal(state.request('qr read',0),false);assert.equal(state.request('qr stream on',0),false);
  assert.equal(state.receive(status,1),false);assert.equal(state.support,'unchecked');
  assert.equal(state.receive('EVT qr code seq=7 code=123+231+312+123 age_ms=0',1),false);
  state.request('qr status',2);assert.equal(state.receive(read,3),false);assert.equal(state.support,'unchecked');
  assert.equal(state.receive(status,4),true);assert.equal(state.support,'supported');assert.equal(state.code,null);
});
test('query read is local non-consumer state and repeated replies preserve code',()=>{
  const state=ready();state.request('qr read',2);state.receive(read,3);assert.equal(state.codeCurrent,true);
  state.request('qr read',4);state.receive(read,5);assert.equal(state.seq,7);assert.equal(state.updatedAt,5);
  assert.equal(state.code,'123+231+312+123');
});
test('explicit stream acknowledgement must match pending request',()=>{
  const state=ready();state.request('qr stream on',2);assert.equal(state.stream,false);
  assert.equal(state.receive('OK qr stream=off',3),false);assert.ok(state.pending);
  assert.equal(state.receive('OK qr stream=on',4),true);assert.equal(state.stream,true);
  state.request('qr stream off',5);state.receive('OK qr stream=off',6);assert.equal(state.stream,false);
});
test('QR support rejection is scoped to a pending QR check; unrelated errors ignored',()=>{
  const state=new QrState();state.connection(true);state.request('qr status',0);
  assert.equal(state.receive('ERR chassis busy',1),false);assert.ok(state.pending);
  assert.equal(state.receive('ERR unknown command',2),true);assert.equal(state.support,'unsupported');
  assert.equal(state.request('qr read',3),false);assert.equal(state.request('qr status',4),true);
  state.receive(status,5);assert.equal(state.support,'supported');
});
test('timeout rejects late checks and permits retry',()=>{
  const state=new QrState();state.connection(true);state.request('qr status',100);
  assert.equal(state.expire(3099),false);assert.equal(state.expire(3100),true);
  assert.equal(state.receive(status,3101),false);assert.equal(state.support,'unchecked');
  assert.equal(state.request('qr status',3200),true);state.receive(status,3201);assert.equal(state.support,'supported');
});
test('disconnect retains history but revokes support, streaming and current-code validity',()=>{
  const state=ready();state.request('qr read',2);state.receive(read,3);state.connection(false);
  assert.equal(state.code,'123+231+312+123');assert.equal(state.codeCurrent,false);assert.equal(state.pending,null);
  assert.equal(state.receive(read,4),false);state.connection(true);
  assert.equal(state.support,'unchecked');assert.equal(state.stream,false);assert.equal(state.receive(status,5),false);
});
test('sequences wrap at uint32 and duplicate code updates its receipt time',()=>{
  const state=ready();const event=(seq,code='123+231+312+123')=>`EVT qr code seq=${seq} code=${code} age_ms=0`;
  state.receive(event(4294967295),2);state.receive(event(0),3);assert.equal(state.seq,0);assert.equal(state.updatedAt,3);
  state.receive(event(1),4);assert.equal(state.seq,1);assert.equal(state.code,'123+231+312+123');
  state.receive(event(0,'999+999+999+999'),5);assert.equal(state.seq,1);assert.equal(state.code,'123+231+312+123');
});
test('invalid current snapshot keeps previous code explicitly historical',()=>{
  const state=ready();state.request('qr read',2);state.receive(read,3);
  state.request('qr read',4);state.receive('OK qr read valid=0 seq=7 code=NA age_ms=NA',5);
  assert.equal(state.code,'123+231+312+123');assert.equal(state.codeCurrent,false);
});
