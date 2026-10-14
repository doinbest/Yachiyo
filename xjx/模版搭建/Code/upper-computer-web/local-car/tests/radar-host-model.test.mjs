import assert from 'node:assert/strict';
import test from 'node:test';

test('a spliced radar line with duplicate kind keys is not accepted as a valid page',()=>{
  const line='@RADAR {"v":1,"profile":"f1_host","k":"host_preview","session":1,"config":2,"scan":1,"map":2,"plan":2,"page":5,"pages":11,"offset":55,"total":116,"live_id":976,"crc32":"C109E617","data":"igYMAaIGCQost","k":"host_config","session":1,"config":2,"scan":1,"map":2,"plan":2,"page":14,"pages":27,"offset":14,"total":27,"data":"angle_max,3600"}';
  assert.equal(parseRadarHost(line),null);
});

import {RadarHostModel,parseRadarHost} from '../radar-host-model.mjs';

const base={v:1,profile:'f1_host',session:7,config:3,scan:11,map:5,plan:6};
const line=frame=>'@RADAR '+JSON.stringify({...base,...frame});
const crc32=bytes=>{let crc=0xffffffff;for(const byte of bytes){crc^=byte;for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}return (crc^0xffffffff)>>>0;};
const packedCloud=points=>Uint8Array.from(points.flatMap(([a,d,e])=>[a&255,a>>>8,d&255,d>>>8,e]));
const b64=bytes=>Buffer.from(bytes).toString('base64');
const textCrc=frame=>crc32(new TextEncoder().encode(JSON.stringify({v:frame.v,profile:frame.profile,k:frame.k,
  ...Object.fromEntries(['session','config','scan','map','plan','page','pages','offset','total','data'].map(key=>[key,frame[key]]))}))).toString(16).padStart(8,'0').toUpperCase();
const previewFrames=(previewId,points,identity={})=>{
  const bytes=Uint8Array.from(points.flatMap(([a,d])=>[a&255,a>>>8,d&255,d>>>8]));
  const crc=crc32(bytes).toString(16).padStart(8,'0').toUpperCase();
  return points.map((_,page)=>line({...identity,k:'host_preview',live_id:previewId,page,pages:points.length,offset:page,total:points.length,crc32:crc,data:b64(bytes.slice(page*4,page*4+4))}));
};
const transfer=(kind,data,{identity={},total=data.length}={})=>data.map((value,page)=>line({...identity,k:kind,page,pages:data.length,offset:page,total,data:value}));

function mapFrames(identity={}){
  const records=['s,40,30,2','m,00000002,00000004,00050146','p,1,0,0',
    ...Array.from({length:25},(_,i)=>`c,${i},${i+1}`)];
  return transfer('host_map',records,{identity,total:records.length});
}
function pathFrames(identity={}){
  return transfer('host_path',['w,230,230,545,1,1,0,0','w,775,230,0,1,2,255,255'],{identity,total:2});
}

test('a corrupt text page is excluded before assembly and a good retransmission completes it',()=>{
  const m=new RadarHostModel();m.connection(true);
  const frames=pathFrames();m.accept(frames[1]);
  const frame=JSON.parse(frames[0].slice(7));
  const goodCrc=textCrc(frame);
  m.accept(line({...frame,crc32:goodCrc,data:frame.data.replace('230','231')}));
  assert.equal(m.lastError,'page_crc_mismatch');
  assert.deepEqual(m.missingPages('path',base,2),[0]);
  assert.equal(m.pending.get('host_path').pages.size,1);
  m.accept(line({...frame,crc32:goodCrc}));
  assert.equal(m.snapshots.path.points[0][0],230);
});

test('map text CRC catches plausible numeric damage without replacing the completed map',()=>{
  const m=new RadarHostModel();m.connection(true);const frames=mapFrames();
  const first=JSON.parse(frames[0].slice(7));
  m.accept(line({...first,crc32:'00000000'}));
  assert.equal(m.lastError,'page_crc_mismatch');assert.deepEqual(m.pendingKinds,[]);
  for(const value of frames){const frame=JSON.parse(value.slice(7));
    m.accept(line({...frame,crc32:textCrc(frame)}));}
  assert.equal(m.snapshots.map.counts.length,25);
});

test('result page CRC binds the kind and pagination to the payload, rejecting a spliced map/path header',()=>{
  const m=new RadarHostModel();m.connection(true);
  const frame=JSON.parse(pathFrames()[0].slice(7));
  const protectedFrame={...frame,crc32:textCrc(frame)};
  m.accept(line(protectedFrame));assert.equal(m.lastError,null);
  m.accept(line({...protectedFrame,k:'host_map'}));
  assert.equal(m.lastError,'page_crc_mismatch');assert.equal(m.pending.has('host_map'),false);
  m.accept(line({...protectedFrame,offset:1}));
  assert.equal(m.lastError,'page_crc_mismatch');
  assert.deepEqual(m.missingPages('path',base,2),[1]);
});

test('map and path records can share pages with record-based offsets',()=>{
  const m=new RadarHostModel();m.connection(true);
  const records=['s,4,3,1','m,00000002,00000000,00050142','p,1,0,0',...Array.from({length:25},(_,i)=>`c,${i},${i===1?2:0}`)];
  m.accept(line({k:'host_map',page:0,pages:2,offset:0,total:28,data:records.slice(0,10).join(';')}));
  m.accept(line({k:'host_map',page:1,pages:2,offset:10,total:28,data:records.slice(10).join(';')}));
  assert.equal(m.lastError,null);assert.equal(m.snapshots.map.counts[1],2);assert.equal(m.snapshots.map.outside_points,1);
  m.accept(line({k:'host_path',page:0,pages:1,offset:0,total:2,data:'w,230,230,545,1,1,1,0;w,775,230,0,1,2,255,255'}));
  assert.equal(m.lastError,null);assert.equal(m.snapshots.path.points.length,2);
});

test('parser accepts only the F1 host profile and known kinds',()=>{
  assert.equal(parseRadarHost(line({k:'host_status',a:0,l:0,ss:0,ls:3,cloud:0,preview:0,reason:'idle'})).k,'host_status');
  assert.equal(parseRadarHost('@RADAR '+JSON.stringify({...base,k:'host_unknown'})),null);
  assert.equal(parseRadarHost('@RADAR '+JSON.stringify({...base,profile:'legacy',k:'host_status'})),null);
  assert.equal(parseRadarHost('@RADAR {bad json'),null);
  assert.equal(parseRadarHost('OK radar host idle'),null);
});

test('config map and path assemble out of order and duplicate pages are idempotent',()=>{
  const model=new RadarHostModel();model.connection(true);
  const configs=transfer('host_config',['source,active','threshold,1','protected,00404405','radar_x,230','radar_y,230'],{total:5});
  for(const at of [4,1,1,3,0,2])assert.equal(model.accept(configs[at],100+at),true);
  assert.equal(model.snapshots.config.threshold,1);assert.equal(model.snapshots.config.protected_mask,0x00404405);
  const maps=mapFrames();for(const at of [...maps.keys()].reverse())model.accept(maps[at],200+at);
  assert.deepEqual(model.snapshots.map.counts,Array.from({length:25},(_,i)=>i+1));
  const paths=pathFrames();model.accept(paths[1],300);model.accept(paths[0],301);
  assert.equal(model.snapshots.path.points.length,2);assert.deepEqual(model.snapshots.path.points[0].slice(0,2),[230,230]);
  assert.ok(Object.isFrozen(model.snapshots.path.points[0]));
});

test('mixed identities and incomplete transfers never replace completed results',()=>{
  const model=new RadarHostModel();for(const page of mapFrames())model.accept(page);
  const complete=model.snapshots.map;
  const mixed=mapFrames({map:8});model.accept(mixed[0]);model.accept(line({k:'host_map',page:1,pages:28,offset:1,total:28,data:'m,0,0,0'}));
  assert.equal(model.snapshots.map,complete);
  assert.equal(model.pendingKinds.includes('host_map'),true);
  model.accept(mixed[2]);assert.equal(model.snapshots.map,complete);
});

test('packed cloud requires exact point boundaries, total and whole-stream CRC',()=>{
  const points=[[0,100,7],[901,65535,255],[3599,230,1]],bytes=packedCloud(points),crc=crc32(bytes).toString(16).padStart(8,'0').toUpperCase();
  const model=new RadarHostModel();
  const pages=[bytes.slice(0,10),bytes.slice(10)].map((part,page)=>line({k:'host_cloud',page,pages:2,offset:page*2,total:3,crc32:crc,data:b64(part)}));
  model.accept(pages[1]);assert.equal(model.snapshots.cloud,undefined);model.accept(pages[0]);
  assert.deepEqual(model.snapshots.cloud.points,points.map(([a,d,e])=>({angle_deg:a/10,distance_mm:d,energy:e})));

  const keep=model.snapshots.cloud;
  for(const bad of [
    line({k:'host_cloud',scan:12,page:0,pages:1,offset:0,total:1,crc32:'00000000',data:'%%%'}),
    line({k:'host_cloud',scan:13,page:0,pages:1,offset:0,total:2,crc32:crc,data:b64(bytes.slice(0,5))}),
    line({k:'host_cloud',scan:14,page:0,pages:1,offset:0,total:1,crc32:'DEADBEEF',data:b64(bytes.slice(0,5))}),
    line({k:'host_cloud',scan:15,page:0,pages:1,offset:0,total:1,crc32:crc,data:b64(bytes.slice(0,4))}),
  ])model.accept(bad);
  assert.equal(model.snapshots.cloud,keep);
});

test('a newer scan supersedes unfinished pages; disconnect and session change retain completed history',()=>{
  const model=new RadarHostModel();model.connection(true);
  const old=line({k:'host_cloud',scan:20,page:0,pages:2,offset:0,total:2,crc32:'00000000',data:b64(packedCloud([[0,1,2]]))});
  model.accept(old);assert.equal(model.pendingKinds.includes('host_cloud'),true);
  const bytes=packedCloud([[10,20,30]]),crc=crc32(bytes).toString(16).padStart(8,'0').toUpperCase();
  model.accept(line({k:'host_cloud',scan:21,page:0,pages:1,offset:0,total:1,crc32:crc,data:b64(bytes)}));
  assert.equal(model.snapshots.cloud.scan,21);
  model.connection(false);assert.equal(model.snapshots.cloud.historical,true);assert.deepEqual(model.pendingKinds,[]);
  model.connection(true);model.accept(line({session:8,k:'host_status',a:0,l:0,ss:0,ls:3,cloud:0,preview:0,reason:'idle'}));
  assert.equal(model.snapshots.cloud.scan,21);assert.equal(model.snapshots.cloud.historical,true);
});

test('map and path completion survives an independently corrupt cloud',()=>{
  const model=new RadarHostModel();for(const page of mapFrames())model.accept(page);for(const page of pathFrames())model.accept(page);
  model.accept(line({k:'host_cloud',page:0,pages:1,offset:0,total:1,crc32:'BAD0C0DE',data:b64(packedCloud([[1,2,3]]))}));
  assert.equal(model.snapshots.map.counts.length,25);assert.equal(model.snapshots.path.points.length,2);assert.equal(model.snapshots.cloud,undefined);
});

test('different preview circles sharing the static scan have separate page identities',()=>{
  const model=new RadarHostModel();model.connection(true);
  const old=previewFrames(100,[[0,100],[10,200]]),next=previewFrames(101,[[0,300],[10,400]]);
  model.accept(old[0]);model.accept(next[0]);assert.equal(model.lastError,null);
  model.accept(next[1]);assert.equal(model.lastError,null);
  assert.equal(model.snapshots.preview.live_id,101);
  assert.deepEqual(model.snapshots.preview.points.map(p=>p.distance_mm),[300,400]);
});

test('a dropped preview page keeps the complete old circle until a whole newer one arrives',()=>{
  const model=new RadarHostModel();model.connection(true);
  for(const frame of previewFrames(100,[[0,100],[10,200]]))model.accept(frame);
  const keep=model.snapshots.preview;
  model.accept(previewFrames(101,[[0,300],[10,400]])[0]);assert.equal(model.snapshots.preview,keep);
  const next=previewFrames(102,[[0,500],[10,600]]);
  model.accept(next[1]);assert.equal(model.snapshots.preview,keep);assert.equal(model.lastError,null);
  model.accept(next[0]);assert.equal(model.snapshots.preview.live_id,102);
  assert.deepEqual(model.snapshots.preview.points.map(p=>p.distance_mm),[500,600]);
});

test('late preview pages cannot replace a newer circle and an actual same-page conflict remains an error',()=>{
  const model=new RadarHostModel();model.connection(true);
  const newest=previewFrames(102,[[0,500],[10,600]]);model.accept(newest[0]);
  for(const frame of previewFrames(101,[[0,100],[10,200]]))model.accept(frame);
  assert.equal(model.lastError,null);model.accept(newest[1]);assert.equal(model.snapshots.preview.live_id,102);
  const next=previewFrames(103,[[0,700],[10,800]]);model.accept(next[0]);
  const conflicting=JSON.parse(next[0].slice(7));conflicting.data=b64(Uint8Array.from([0,0,189,2]));
  model.accept('@RADAR '+JSON.stringify(conflicting));
  assert.equal(model.lastError,'duplicate_conflict');
});

test('preview sequence is independent of static scan and can wrap or restart after reconnection',()=>{
  const model=new RadarHostModel();model.connection(true);
  model.accept(line({k:'host_status',scan:100,ss:4,l:1}));
  for(const frame of previewFrames(0xffffffff,[[0,100]],{scan:0}))model.accept(frame);
  assert.equal(model.snapshots.preview.live_id,0xffffffff);
  for(const frame of previewFrames(1,[[0,200]],{scan:0}))model.accept(frame);
  assert.equal(model.snapshots.preview.live_id,1);
  model.connection(false);model.connection(true);
  for(const frame of previewFrames(1,[[0,300]],{scan:0}))model.accept(frame);
  assert.equal(model.snapshots.preview.points[0].distance_mm,300);
});
