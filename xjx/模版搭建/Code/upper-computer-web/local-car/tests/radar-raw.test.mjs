import test from 'node:test';
import assert from 'node:assert/strict';
import * as radar from '../radar-model.mjs';
import {rawFrames,rawLine} from './radar-raw-fixture.mjs';

test('raw pages publish only after contiguous bytes and CRC match, accepting trimmed pre-circle prefixes',()=>{
  const model=new radar.RadarModel(),frames=rawFrames();
  for(const frame of frames.slice(0,-1))model.accept(rawLine(frame));
  assert.equal(model.snapshots.raw,undefined);
  model.accept(rawLine(frames.at(-1)));
  assert.equal(model.snapshots.raw?.id,11);
  assert.equal(model.snapshots.raw.hex,'313233343536373839');
  assert.equal(model.snapshots.raw.download_complete,true);
  assert.equal(model.snapshots.raw.complete,true);
  assert.equal(model.snapshots.raw.truncated,true);
  assert.deepEqual(model.snapshots.raw.filter,[100,40000,0,3600,3,0,255]);
});

test('raw rejects bad CRC, byte gaps and incomplete circle windows without replacing the last complete capture',()=>{
  const model=new radar.RadarModel();for(const frame of rawFrames())model.accept(rawLine(frame));
  for(const defect of ['crc','gap','window']){
    const frames=rawFrames({id:12});
    if(defect==='crc')frames[0].crc32='00000000';
    if(defect==='gap')frames[6].offset=4;
    if(defect==='window')frames[3].circle=[90,109];
    for(const frame of frames)model.accept(rawLine(frame));
    assert.equal(model.snapshots.raw?.id,11,defect);
    model.pending.clear();
  }
});

test('raw attempt id prevents pages of two attempts with identical map identities mixing',()=>{
  const model=new radar.RadarModel(),old=rawFrames(),next=rawFrames({id:12});
  for(const frame of old.slice(0,-1))model.accept(rawLine(frame));
  model.accept(rawLine(next.at(-1)));assert.equal(model.snapshots.raw,undefined);
  for(const frame of next.slice(0,-1))model.accept(rawLine(frame));
  assert.equal(model.snapshots.raw?.id,12);
});

test('stopped partial raw can be downloaded and exported without importing a previous map or cloud',()=>{
  const model=new radar.RadarModel(),frames=rawFrames({scan:0,map:0});
  Object.assign(frames[0],{state:'stopped',complete:false});Object.assign(frames[3],{circle:[0,0]});frames[4].cloud_valid=false;
  model.snapshots.map={k:'map',session:9,origin:2,scan:3,map:7,plan:8};model.snapshots.cloud={k:'cloud',session:9,origin:2,scan:3,map:7,plan:8,points:[[0,100,1]]};
  for(const frame of frames)model.accept(rawLine(frame));
  assert.equal(model.snapshots.raw?.download_complete,true);assert.equal(model.snapshots.raw?.complete,false);
  const exported=radar.diagnosticSnapshot(model,11);assert.deepEqual(Object.keys(exported.snapshots),['raw']);assert.equal(exported.diagnostic.replayable,false);
});

test('raw exchange uses the independent attempt id and discovers variable pages continuously',async()=>{
  const frames=rawFrames(),wires=[];let exchange;
  exchange=new radar.RadarExchange({send:async wire=>{wires.push(wire);const page=Number(wire.split(' ').at(-1));queueMicrotask(()=>exchange.receive(rawLine(frames[page])));return true;}});
  assert.equal((await exchange.fetch('raw',11)).length,7);
  assert.deepEqual(wires,frames.map(f=>`radar fetch raw 11 ${f.page}`));
});

test('diagnostic export waits for full same-capture cloud and freezes raw filters without old snapshots',()=>{
  const model=new radar.RadarModel();for(const frame of rawFrames())model.accept(rawLine(frame));
  model.snapshots.map={k:'map',session:9,origin:2,scan:3,map:7,plan:8,pose:[2170,230,180],mask:0x50140,counts:Array(25).fill(0)};
  model.snapshots.path={k:'path',session:9,origin:2,scan:3,map:7,plan:8,points:[],valid:false};
  model.snapshots.points={k:'points',session:9,origin:2,scan:2,map:6,plan:7,points:[[0,1,1]]};
  model.snapshots.cloud={k:'cloud',session:9,origin:2,scan:2,map:6,plan:7,points:[[0,1,1]]};
  model.params={threshold:99};
  assert.throws(()=>radar.diagnosticSnapshot(model,11),/同圈.*点云/);
  model.snapshots.cloud={k:'cloud',session:9,origin:2,scan:3,map:7,plan:8,total:2,points:[[0,100,1],[900,200,2]],complete:true,truncated:false};
  const result=radar.diagnosticSnapshot(model,11);
  assert.deepEqual(Object.keys(result.snapshots).sort(),['cloud','map','path','raw']);
  assert.equal(result.params.threshold,3);
  assert.equal(result.diagnostic.id,11);
  model.snapshots.cloud.points.length=0;assert.equal(result.snapshots.cloud.points.length,2);
});

test('a complete automatic circle with no selected map exports its matching cloud and raw without historical maps',()=>{
  const model=new radar.RadarModel(),frames=rawFrames({map:0});frames[1].start=0;frames[1].auto=true;
  for(const frame of frames)model.accept(rawLine(frame));
  model.snapshots.map={k:'map',session:9,origin:2,scan:2,map:6,plan:7};
  assert.throws(()=>radar.diagnosticSnapshot(model,11),/同圈.*点云/);
  model.snapshots.cloud={k:'cloud',session:9,origin:2,scan:3,map:0,plan:0,points:[[0,100,1],[900,200,2]],complete:true,truncated:false};
  const exported=radar.diagnosticSnapshot(model,11);assert.deepEqual(Object.keys(exported.snapshots).sort(),['cloud','raw']);assert.equal(exported.diagnostic.replayable,true);
});


test('old upper diagnostic pose is preserved while exported reference uses the current corner contract',()=>{
  const model=new radar.RadarModel(),frames=rawFrames({map:0});frames[1].pose=[2330,2170,0];frames[1].start=1;
  for(const f of frames)model.accept(rawLine(f));
  model.snapshots.cloud={k:'cloud',session:9,origin:2,scan:3,map:0,plan:0,pose:[2330,2170,0],points:[[0,100,1],[900,200,2]],complete:true,truncated:false};
  const exported=radar.diagnosticSnapshot(model,11);
  assert.deepEqual([exported.params.lidar_x_mm,exported.params.lidar_y_mm,exported.params.zero_deg],[2170,230,270]);
  assert.deepEqual(exported.snapshots.raw.pose,[2330,2170,0]);assert.deepEqual(exported.snapshots.cloud.pose,[2330,2170,0]);assert.equal(exported.params.angle_max_tenths,3600);
});
