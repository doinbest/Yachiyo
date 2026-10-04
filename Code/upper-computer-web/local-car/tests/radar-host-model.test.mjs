import assert from 'node:assert/strict';
import test from 'node:test';

import {RadarHostModel,parseRadarHost} from '../radar-host-model.mjs';

const base={v:1,profile:'f1_host',session:7,config:3,scan:11,map:5,plan:6};
const line=frame=>'@RADAR '+JSON.stringify({...base,...frame});
const crc32=bytes=>{let crc=0xffffffff;for(const byte of bytes){crc^=byte;for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}return (crc^0xffffffff)>>>0;};
const packedCloud=points=>Uint8Array.from(points.flatMap(([a,d,e])=>[a&255,a>>>8,d&255,d>>>8,e]));
const b64=bytes=>Buffer.from(bytes).toString('base64');
const transfer=(kind,data,{identity={},total=data.length}={})=>data.map((value,page)=>line({...identity,k:kind,page,pages:data.length,offset:page,total,data:value}));

function mapFrames(identity={}){
  const records=['s,40,30,2','m,00000002,00000004,00050146','p,1,0,0',
    ...Array.from({length:25},(_,i)=>`c,${i},${i+1}`)];
  return transfer('host_map',records,{identity,total:records.length});
}
function pathFrames(identity={}){
  return transfer('host_path',['w,230,230,545,1,1,0,0','w,775,230,0,1,2,255,255'],{identity,total:2});
}

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
