import test from 'node:test';
import assert from 'node:assert/strict';
import * as live from '../radar-live.mjs';
import {RadarModel,RadarExchange,parseRadar} from '../radar-model.mjs';

const line=frame=>'@RADAR '+JSON.stringify(frame);
const state=(extra={})=>({v:1,k:'live_state',s:9,id:2,q:0,active:true,state:'waiting',reason:'none',pose:[2170,230,180],age_ms:0,points:0,...extra});
const part=(q,o,extra={})=>{
  const bytes=new Uint8Array(96),view=new DataView(bytes.buffer);
  for(let i=0;i<24;i++){view.setUint16(i*4,(o+i)*30,true);view.setUint16(i*4+2,i===0?0:1000,true);}
  return {v:1,k:'live',s:9,id:2,q,o,n:120,d:Buffer.from(bytes).toString('base64'),...extra};
};

test('compact live traffic stays separate from static snapshots and outstanding page exchanges',async()=>{
  const staticModel=new RadarModel(),model=new live.RadarLiveModel(),frames=[state(),part(1,0)];
  model.connection(true);
  let exchange;exchange=new RadarExchange({send:async()=>true,timeoutMs:1000});
  const pending=exchange.fetch('map',7);await new Promise(setImmediate);
  for(const frame of frames){assert.equal(parseRadar(line(frame)),null);assert.equal(staticModel.accept(line(frame)),false);assert.equal(exchange.receive(line(frame)),false);assert.equal(model.accept(line(frame),100),true);}
  assert.deepEqual(staticModel.snapshots,{});exchange.cancel();await assert.rejects(pending,/取消/);
});

test('live LE angle/distance slices assemble out of order with a clearly incomplete preview',()=>{
  const model=new live.RadarLiveModel();model.connection(true);model.accept(line(state()),100);
  for(const offset of [96,0,48,24])model.accept(line(part(1,offset)),200+offset);
  assert.equal(model.frame,null);assert.equal(model.preview.received,96);assert.equal(model.preview.complete,false);
  model.accept(line(part(1,72)),400);
  assert.equal(model.frame.q,1);assert.equal(model.frame.points.length,120);assert.deepEqual(model.frame.points[49],[1470,1000]);
  assert.equal(model.frame.complete,true);assert.deepEqual(model.frame.pose,[2170,230,180]);
  assert.equal(model.frame.receivedAt,400);
});

test('a previous complete live circle stays displayed until all next-circle slices arrive',()=>{
  const model=new live.RadarLiveModel();model.connection(true);model.accept(line(state()),100);
  for(const offset of [0,24,48,72,96])model.accept(line(part(1,offset)),200+offset);
  const previous=model.frame;
  model.accept(line(part(2,0)),1000);assert.equal(model.displayed(),previous);assert.equal(model.displayed().points.length,120);
  assert.match(model.describe(1020),/完整圈.*接收下一圈 24\/120/);
  for(const offset of [24,48,72]){model.accept(line(part(2,offset)),1100+offset);assert.equal(model.displayed(),previous);}
  model.accept(line(part(2,96)),1300);assert.equal(model.displayed().q,2);assert.equal(model.displayed().receivedAt,1300);assert.match(model.describe(1320),/更新 20 ms前/);
  model.accept(line(state({id:3})),1400);model.accept(line(part(1,0,{id:3})),1500);
  assert.equal(model.displayed().id,3);assert.equal(model.displayed().complete,false);assert.equal(model.displayed().received,24);
});

test('latest live sequence never combines old parts, other IDs or sessions',()=>{
  const model=new live.RadarLiveModel();model.connection(true);model.accept(line(state()),1);
  model.accept(line(part(1,0)),2);model.accept(line(part(2,24)),3);
  model.accept(line(part(1,48)),4);model.accept(line(part(2,0,{id:1})),5);model.accept(line(part(2,0,{s:10})),6);
  assert.equal(model.preview.q,2);assert.equal(model.preview.received,24);assert.equal(model.frame,null);
  for(const offset of [0,48,72,96])model.accept(line(part(2,offset)),7);
  assert.equal(model.frame.q,2);
  model.accept(line(state({s:10,id:1,pose:[2330,2170,270]})),8);
  assert.equal(model.frame.historical,true);assert.equal(model.preview,null);
  for(const offset of [0,24,48,72,96])model.accept(line(part(1,offset,{s:10,id:1})),9);
  assert.equal(model.frame.s,10);assert.deepEqual(model.frame.pose,[2330,2170,270]);
  assert.equal(model.accept(line(state()),10),false);assert.equal(model.state.s,10);
  model.accept(line(state({s:10,id:2})),11);assert.equal(model.accept(line(state({s:10,id:1})),12),false);assert.equal(model.state.id,2);
});

test('live rejects broken base64, overlapping offsets and wrong point totals without completing a frame',()=>{
  const model=new live.RadarLiveModel();model.connection(true);model.accept(line(state()),1);
  for(const frame of [part(1,0,{d:'!'}),part(1,1),part(1,0,{n:119}),part(1,0,{d:Buffer.alloc(4).toString('base64')})])assert.equal(model.accept(line(frame),2),false);
  assert.equal(model.frame,null);assert.equal(model.preview,null);
});

test('projection uses millimeters and clockwise lidar angles in the frozen world pose, dropping empty bins',()=>{
  assert.deepEqual(live.projectRadarPoint([0,1000],[2170,230,180]).map(Math.round),[1170,230]);
  assert.deepEqual(live.projectRadarPoint([900,1000],[2170,230,180]).map(Math.round),[2170,1230]);
  assert.deepEqual(live.projectRadarPoint([900,1000],[2330,2170,270]).map(Math.round),[1330,2170]);
  assert.equal(live.projectRadarPoint([1800,0],[2170,230,180]),null);
});

test('stop and connection changes retain complete history and age without automatically sending anything',()=>{
  const model=new live.RadarLiveModel();model.connection(true);model.accept(line(state()),100);
  for(const offset of [0,24,48,72,96])model.accept(line(part(1,offset)),200);
  assert.match(model.describe(300),/100 ms/);assert.match(model.describe(6200),/暂未更新.*6000 ms/);
  model.accept(line(state({active:false,state:'stopped',q:1})),6300);assert.match(model.describe(6400),/已停止.*历史/);
  assert.equal(model.accept(line(state({q:1,state:'streaming'})),6450),false);assert.equal(model.running,false);
  model.connection(false);assert.match(model.describe(6500),/连接已断开.*历史/);assert.equal(model.frame.points.length,120);
  model.connection(true);assert.equal(model.running,false);assert.match(model.describe(6600),/历史/);
});

test('reconnection accepts reset live IDs when the controller reuses its previous session number',()=>{
  const model=new live.RadarLiveModel();model.connection(true);model.accept(line(state({s:13,id:2})),100);
  for(const offset of [0,24,48,72,96])model.accept(line(part(10,offset,{s:13,id:2})),200);
  const previous=model.frame;model.connection(false);model.connection(true);
  assert.equal(model.state,null);assert.equal(previous.historical,true);assert.equal(model.displayed(),previous);
  assert.equal(model.accept(line(state({s:13,id:1})),300),true);assert.equal(model.frame,previous);
  for(const offset of [0,24,48,72,96])model.accept(line(part(1,offset,{s:13,id:1})),400);
  assert.equal(model.frame.id,1);assert.equal(model.frame.q,1);assert.equal(model.frame.historical,false);
});

test('reconnection resets frame ordering even for reused session and live ID, without presenting old points as current',()=>{
  const model=new live.RadarLiveModel();model.connection(true);model.accept(line(state({s:13,id:1})),100);
  for(const offset of [0,24,48,72,96])model.accept(line(part(20,offset,{s:13,id:1})),200);
  const previous=model.frame;model.connection(false);model.connection(true);model.accept(line(state({s:13,id:1})),300);
  assert.equal(model.displayed(),previous);assert.equal(previous.historical,true);
  assert.equal(model.accept(line(part(1,0,{s:13,id:1})),400),true);
  assert.equal(model.displayed().q,1);assert.equal(model.displayed().complete,false);assert.equal(model.frame,previous);
  for(const offset of [24,48,72,96])model.accept(line(part(1,offset,{s:13,id:1})),500);
  assert.equal(model.frame.q,1);assert.equal(model.frame.receivedAt,500);
});

test('reconnection permits a firmware session that was retired earlier in the old connection',()=>{
  const model=new live.RadarLiveModel();model.connection(true);model.accept(line(state({s:13})),100);model.accept(line(state({s:14})),200);
  assert.equal(model.accept(line(state({s:13})),300),false);model.connection(false);model.connection(true);
  assert.equal(model.accept(line(state({s:13,id:1})),400),true);assert.equal(model.running,true);
});
