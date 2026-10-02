import test from 'node:test';
import assert from 'node:assert/strict';
import {trackAttempt} from '../command-attempt.mjs';

test('shows requested wire immediately, before asynchronous write confirmation', async () => {
  const states=[];
  let finish;
  const result=trackAttempt('state all',()=>new Promise(resolve=>{finish=resolve;}),s=>states.push({...s}));
  assert.deepEqual(states,[{wire:'state all',state:'pending'}]);
  finish(true);
  assert.equal(await result,true);
  assert.deepEqual(states[1],{wire:'state all',state:'written'});
});

test('a rejected or failed send never reports a successful write', async () => {
  for (const operation of [async()=>false,async()=>{throw new Error('disconnected');}]) {
    const states=[];
    assert.equal(await trackAttempt('pos base 90 100 0',operation,s=>states.push(s.state)),false);
    assert.deepEqual(states,['pending','failed']);
  }
});
import {createAttemptTracker} from '../command-attempt.mjs';
test('a cancelled older request cannot replace the latest stop operation', async()=>{
  const states=[];
  const tracker=createAttemptTracker(s=>states.push(s));
  let finish;
  const pending=tracker.run('pos base 10',()=>new Promise(resolve=>{finish=resolve;}));
  tracker.supersede();
  finish(false);
  await pending;
  assert.equal(states[0].current,true);
  assert.equal(states[1].state,'failed');
  assert.equal(states[1].current,false);
});
