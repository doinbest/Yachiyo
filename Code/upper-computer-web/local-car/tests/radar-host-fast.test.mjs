import test from 'node:test';
import assert from 'node:assert/strict';
import {RadarHostModel,parseRadarHost} from '../radar-host-model.mjs';
import {RadarHostClient} from '../radar-host-client.mjs';

function checksum(text){let c=0xffffffff;for(const b of Buffer.from(text)){c^=b;for(let n=0;n<8;n++)c=(c>>>1)^((c&1)?0xedb88320:0);}return ((c^0xffffffff)>>>0).toString(16).padStart(8,'0').toUpperCase();}
function frames({id=[1,2,3,4,5],valid=true,live}={}){
  // Independent fixture: little-endian v1 bundle header and two 10-byte waypoints.
  const bytes=Buffer.alloc(valid?94:74);let at=0;
  const u8=n=>{bytes[at++]=n;},u16=n=>{bytes.writeUInt16LE(n,at);at+=2;},u32=n=>{bytes.writeUInt32LE(n,at);at+=4;};
  u8(1);u16(1600);u16(1400);u16(1398);u32(0x00200000);u32(0);u32(0x00250140);
  u8(valid?1:0);u8(valid?0:1);u8(valid?0:5);u16(valid?2:0);
  for(let i=0;i<25;i++)u16(i===21?2:0);
  if(valid)for(const p of [[230,230,558,1,1,1,0],[775,350,0,1,2,255,255]]){p.slice(0,3).forEach(u16);p.slice(3).forEach(u8);}
  const pages=Math.ceil(bytes.length/84),result=[];
  for(let page=0;page<pages;page++){
    const f={v:2,k:'r',i:id,...(live===undefined?{}:{l:live}),p:page,n:pages,t:bytes.length,d:bytes.subarray(page*84,(page+1)*84).toString('base64')};
    result.push('@RADAR '+JSON.stringify({...f,c:checksum(JSON.stringify(f))}));
  }
  const end={v:2,k:'e',i:id,...(live===undefined?{}:{l:live}),n:pages};
  return {pages:result,end:'@RADAR '+JSON.stringify({...end,c:checksum(JSON.stringify(end))})};
}
const tick=()=>new Promise(setImmediate);
test('compact result publishes map and every waypoint together only after all pages',()=>{
  const m=new RadarHostModel();m.connection(true);const f=frames();
  assert.equal(parseRadarHost(f.pages[0])?.k,'host_bundle');
  m.accept(f.pages[1],100);assert.equal(m.snapshots.map,undefined);assert.equal(m.snapshots.path,undefined);
  m.accept(f.pages[0],200);
  assert.equal(m.snapshots.map.receivedAt,200);assert.equal(m.snapshots.path.receivedAt,200);
  assert.equal(m.snapshots.map.raw_points,1600);assert.equal(m.snapshots.map.counts[21],2);
  assert.deepEqual(m.snapshots.path.points,[[230,230,558,1,1,1,0],[775,350,0,1,2,255,255]]);
  assert.deepEqual(m.lastEvent.key,'result');
});
test('compact checksum rejects changed identity before publishing any snapshot',()=>{
  const m=new RadarHostModel();m.connection(true);const f=frames();
  const damaged=JSON.parse(f.pages[0].slice(7));damaged.i[2]=99;
  m.accept('@RADAR '+JSON.stringify(damaged));assert.equal(m.lastError,'bundle_page_crc');m.accept(f.pages[1]);
  assert.equal(m.snapshots.map,undefined);assert.equal(m.snapshots.path,undefined);
});
test('compact unreachable plan still publishes map and exact failure endpoints',()=>{
  const m=new RadarHostModel();m.connection(true);m.accept(frames({valid:false}).pages[0]);
  assert.equal(m.snapshots.map.path_valid,false);assert.equal(m.snapshots.path.failed_from_task,1);assert.equal(m.snapshots.path.failed_to_task,5);
  assert.deepEqual(m.snapshots.path.points,[]);
});
test('fast single command accepts pushed results without a result request',async()=>{
  const commands=[],events=[],f=frames();let client;
  client=new RadarHostClient({fastResults:true,send:async wire=>{commands.push(wire);return {};},onSnapshot:(kind)=>events.push(kind)});
  await client.command('scan');assert.equal(commands[0],'radar host scan fast');
  for(const line of f.pages)client.receive(line);client.receive(f.end);
  for(let i=0;i<8;i++)await tick();
  assert.deepEqual(commands,['radar host scan fast','radar host ack 5']);
  assert.deepEqual(events,['result']);client.cancelTransfer();
});
test('missing compact pages are repaired in one request and no half-result is published',async()=>{
  const commands=[],f=frames();let client;
  client=new RadarHostClient({fastResults:true,send:async wire=>{commands.push(wire);if(wire==='radar host bundle 5 0')queueMicrotask(()=>client.receive(f.pages[0]));return {};}});
  await client.command('scan');client.receive(f.pages[1]);client.receive(f.end);
  for(let i=0;i<12;i++)await tick();
  assert.deepEqual(commands,['radar host scan fast','radar host bundle 5 0','radar host ack 5']);
  assert.equal(client.model.snapshots.map.receivedAt,client.model.snapshots.path.receivedAt);client.cancelTransfer();
});
test('repair waits for the actual wireless response instead of repeating every second',async()=>{
  const commands=[],f=frames();let client;
  client=new RadarHostClient({fastResults:true,timeoutMs:1600,send:async wire=>{
    commands.push(wire);
    if(wire==='radar host bundle 5 0')setTimeout(()=>client.receive(f.pages[0]),1100);
    return {};
  }});
  await client.command('scan');client.receive(f.pages[1]);client.receive(f.end);
  await new Promise(resolve=>setTimeout(resolve,1300));
  assert.deepEqual(commands,['radar host scan fast','radar host bundle 5 0','radar host ack 5']);
  client.cancelTransfer();
});
test('live-to-single uses the requested mode when the completion status is lost',async()=>{
  const f=frames(),client=new RadarHostClient({fastResults:true,send:async()=>({})});
  client.receive('@RADAR '+JSON.stringify({v:1,profile:'f1_host',k:'host_status',session:1,config:2,scan:2,map:3,plan:4,a:0,l:1,ss:4,ls:2,fast:1}));
  await client.command('scan');
  for(const page of f.pages)client.receive(page);
  await tick();
  assert.equal(client.model.status.l,0);assert.equal(client.model.status.a,0);client.cancelTransfer();
});
test('stopping rejects delayed pushed bundles and preserves the completed pair',async()=>{
  const f=frames(),later=frames({id:[1,2,4,5,6]}),client=new RadarHostClient({fastResults:true,send:async()=>({})});
  await client.command('scan');for(const page of f.pages)client.receive(page);await tick();
  const map=client.model.snapshots.map,path=client.model.snapshots.path;
  await client.command('stop');for(const page of later.pages)client.receive(page);client.receive(later.end);
  assert.equal(client.model.snapshots.map,map);assert.equal(client.model.snapshots.path,path);
});
test('late newer live circles cannot masquerade as a requested single scan',async()=>{
  const commands=[],old=frames({live:1}),late=frames({id:[1,2,90,91,92],live:1}),next=frames({id:[1,2,91,92,93],live:0});
  const client=new RadarHostClient({fastResults:true,send:async wire=>{commands.push(wire);return {};}});
  await client.command('live-start');for(const page of old.pages)client.receive(page);await tick();
  const map=client.model.snapshots.map;
  await client.command('scan');
  client.receive('@RADAR '+JSON.stringify({v:1,profile:'f1_host',k:'host_status',session:1,config:2,scan:90,map:91,plan:92,a:0,l:1,ss:4,ls:2,fast:1}));
  for(const page of late.pages)client.receive(page);client.receive(late.end);await tick();
  assert.equal(client.model.snapshots.map,map);assert.ok(client.pendingStart);assert.equal(client.syncing,null);
  client.receive('OK radar host scan_accepted');for(const page of next.pages)client.receive(page);await tick();
  assert.equal(client.model.snapshots.path.plan,93);assert.equal(client.model.status.l,0);
  assert.ok(!commands.some(command=>command.includes('bundle 92')||command==='radar host ack 92'));client.cancelTransfer();
});
