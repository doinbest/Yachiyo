import test from 'node:test';
import assert from 'node:assert/strict';
import {visibleTerminalLogs,createLogRenderScheduler} from '../terminal-log.mjs';

const rx=text=>({direction:'RX',text});
const report=(extra='')=>rx(`OK grab state=stopping mode=align reason=cancelled missing=none stop_requested=1 stop_confirmed=0 ${extra}`.trim());

test('quiet grab logs hide changing counters but preserve transitions, errors and pixels',()=>{
  const first=report('elapsed=100 x=0');
  const duplicate=report('elapsed=600 x=0.001');
  const pixel=rx('VISION RX fn=B4 color=1 valid=1 fresh=1 cx=334 cy=338');
  const error=rx('ERR grab start rejected');
  const confirmed=rx('OK grab state=idle mode=align reason=cancelled missing=none stop_requested=1 stop_confirmed=1');
  const lines=[first,duplicate,pixel,report('elapsed=1100'),error,confirmed];
  assert.deepEqual(visibleTerminalLogs(lines,true),[first,pixel,error,confirmed]);
  assert.deepEqual(visibleTerminalLogs(lines,false),lines);
  assert.equal(lines.length,6); // Export source remains untouched.
});

test('explicit status queries and changed fault reasons remain visible',()=>{
  const first=report();
  const query={direction:'TX',text:'grab status'};
  const answer=report('elapsed=2000');
  const fault=rx('OK grab state=stopping mode=align reason=motor_request missing=none stop_requested=1 stop_confirmed=0');
  const malformed=rx('OK grab state=stopping stop_confirmed=broken');
  const prefixed=rx(`arm> ${first.text}`);
  assert.deepEqual(visibleTerminalLogs([first,prefixed,query,answer,fault,malformed],true),[first,query,answer,fault,malformed]);
});

test('an MCU restart starts a new reporting sequence',()=>{
  const first=report(),boot=rx('STM32 mechanical arm console ready'),next=report();
  assert.deepEqual(visibleTerminalLogs([first,boot,next],true),[first,boot,next]);
});

test('Chinese view hides only known echoes and prompts; raw entries remain intact',()=>{
  const rows=[{direction:'TX',text:'info'},rx('arm> info'),rx('arm> '),rx('OK config z rpm=500'),rx('unknown response')];
  assert.deepEqual(visibleTerminalLogs(rows,false,true),[rows[0],rows[3],rows[4]]);
  assert.deepEqual(visibleTerminalLogs(rows,false,false),rows);
});
test('batched log rendering schedules one frame and resumes with latest cache',()=>{
  let hidden=false,rendered=0;const callbacks=[];
  const schedule=createLogRenderScheduler(()=>rendered++,fn=>callbacks.push(fn),()=>hidden);
  for(let i=0;i<100;i++)schedule();
  assert.equal(callbacks.length,1);callbacks.shift()();assert.equal(rendered,1);
  hidden=true;for(let i=0;i<100;i++)schedule();assert.equal(callbacks.length,0);
  hidden=false;schedule();assert.equal(callbacks.length,1);callbacks.shift()();assert.equal(rendered,2);
});


test('B2 coordinate validity transitions stay visible while receive counters collapse',()=>{
  const rows=['Wait','Lost','Off','Ok','Ok','Stale','Idle'].map((vision,i)=>rx(`OK grab state=acquire protocol=B2 age_source=rx vision=${vision} rx_seq=${i}`));
  assert.deepEqual(visibleTerminalLogs(rows,true),rows.filter((_,i)=>i!==4));
  assert.equal(rows.length,7); // Raw export still has every input.
});


test('recovery transitions survive filtering but observation counters do not spam logs',()=>{
  const rows=[
    rx('OK grab state=vision_pause reason=vision_pause_stopping recovery_used=0 stop_confirmed=0'),
    rx('OK grab state=vision_pause reason=vision_recover_wait recovery_used=0 recovery_count=0 recovery_left_ms=500 stop_confirmed=1'),
    rx('OK grab state=vision_pause reason=vision_recover_wait recovery_used=0 recovery_count=2 recovery_left_ms=400 stop_confirmed=1'),
    rx('OK grab state=align reason=vision_recovered recovery_used=1'),
    rx('OK grab state=align reason=vision_recovered recovery_used=2')];
  assert.deepEqual(visibleTerminalLogs(rows,true),rows.filter((_,i)=>i!==2));
});
