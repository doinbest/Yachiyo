import test from 'node:test';
import assert from 'node:assert/strict';
import { SerialLink } from '../serial.mjs';

function device() {
  const writes=[];
  let input;
  const port={
    readable:new ReadableStream({start(c){input=c;}}),
    writable:new WritableStream({write(v){writes.push(new TextDecoder().decode(v));}}),
    async open(options){this.options=options;},
    async close(){assert.equal(this.readable.locked,false);assert.equal(this.writable.locked,false);this.closed=true;},
  };
  return {port,writes,get input(){return input;}};
}
test('connect sends nothing; explicit send frames once; disconnect releases both locks',async()=>{
  const d=device(); const lines=[]; const states=[];
  const link=new SerialLink({receive:l=>lines.push(l),state:s=>states.push(s)});
  await link.connect({requestPort:async()=>d.port},115200);
  assert.deepEqual(d.writes,[]);
  await link.send('state all');
  assert.deepEqual(d.writes,['state all\r\n']);
  d.input.enqueue(new TextEncoder().encode('OK state base reached=1\r\n'));
  await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(lines,['OK state base reached=1']);
  await link.disconnect();
  assert.equal(d.port.closed,true); assert.equal(states.at(-1),'disconnected');
  await assert.rejects(link.send('grip open'),/连接/);
});
test('unexpected disconnection clears state, and open failure releases the selected port',async()=>{
  const d=device(); const states=[];
  const link=new SerialLink({state:s=>states.push(s)});
  await link.connect({requestPort:async()=>d.port},115200);
  d.input.error(new Error('unplugged'));
  await new Promise(resolve=>setTimeout(resolve,0));
  assert.equal(link.connected,false); assert.equal(states.at(-1),'disconnected');
  const fail=device(); fail.port.open=async()=>{throw new Error('occupied');};
  await assert.rejects(link.connect({requestPort:async()=>fail.port},115200),/occupied/);
  assert.equal(link.connected,false);
});
