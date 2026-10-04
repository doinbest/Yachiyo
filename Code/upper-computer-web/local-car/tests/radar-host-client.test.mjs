import assert from 'node:assert/strict';
import test from 'node:test';

import {RadarHostClient} from '../radar-host-client.mjs';
import {RadarHostModel} from '../radar-host-model.mjs';
import {frameCommand} from '../protocol.mjs';

const flush=()=>new Promise(setImmediate);
const id={v:1,profile:'f1_host',session:1,config:1,scan:2,map:3,plan:4};
const line=frame=>'@RADAR '+JSON.stringify({...id,...frame});
const mapRecords=['s,4,4,0','m,00000000,00000000,00050140','p,1,0,0',...Array.from({length:25},(_,i)=>`c,${i},0`)];
const pathRecords=['w,230,230,545,1,1,1,0','w,775,230,0,1,2,255,255'];
const crc32=bytes=>{let crc=0xffffffff;for(const byte of bytes){crc^=byte;for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}return (crc^0xffffffff)>>>0;};
const cloudBytes=Uint8Array.from([0,0,100,0,9]);

function fixture(){
  const wires=[],model=new RadarHostModel();let client;
  const send=async wire=>{
    frameCommand(wire);wires.push(wire);
    const match=/^radar host fetch (map|path|cloud):(\d+) (\d+)$/.exec(wire);
    if(match){const [,kind,,pageText]=match,page=Number(pageText),records=kind==='map'?mapRecords:pathRecords;
      queueMicrotask(()=>client.receive(line(kind==='cloud'?{k:'host_cloud',page:0,pages:1,offset:0,total:1,crc32:crc32(cloudBytes).toString(16).padStart(8,'0').toUpperCase(),data:Buffer.from(cloudBytes).toString('base64')}:
        {k:`host_${kind}`,page,pages:records.length,offset:page,total:records.length,data:records[page]})));
    }
    return {kind:'tx',text:wire};
  };
  client=new RadarHostClient({send,model,timeoutMs:100});return {client,model,wires};
}

test('construction, connection and visibility-equivalent changes send nothing',()=>{
  const f=fixture();f.client.connection(true);f.client.connection({state:'connected'});f.client.cancelTransfer();assert.deepEqual(f.wires,[]);
});

test('explicit hardware actions map to exact bounded host commands',async()=>{
  const f=fixture();
  for(const [action,wire] of [['start','radar host start'],['stop','radar host stop'],['scan','radar host scan'],['live-start','radar host live start'],['live-stop','radar host live stop'],['status','radar host status']]){
    await f.client.command(action);assert.equal(f.wires.at(-1),wire);frameCommand(wire);
  }
});

test('ready scan fetches map then path and starts cloud only after both complete',async()=>{
  const f=fixture();await f.client.command('scan');
  f.client.receive(line({k:'host_status',a:0,l:0,ss:4,ls:3,cloud:1,preview:0,reason:'complete'}));
  for(let i=0;i<80&&!f.model.snapshots.cloud;i++)await flush();
  assert.equal(f.model.snapshots.map.counts.length,25);assert.equal(f.model.snapshots.path.points.length,2);assert.equal(f.model.snapshots.cloud.points.length,1);
  const firstMap=f.wires.findIndex(w=>w.startsWith('radar host fetch map:'));
  const lastMap=f.wires.findLastIndex(w=>w.startsWith('radar host fetch map:'));
  const firstPath=f.wires.findIndex(w=>w.startsWith('radar host fetch path:'));
  const lastPath=f.wires.findLastIndex(w=>w.startsWith('radar host fetch path:'));
  const firstCloud=f.wires.findIndex(w=>w.startsWith('radar host fetch cloud:'));
  assert.ok(firstMap>0&&firstPath>lastMap&&firstCloud>lastPath);
});

test('a newer scan cancels an old background cloud and compact results switch identities',async()=>{
  const wires=[],model=new RadarHostModel();let client,releaseCloud;
  const send=async wire=>{wires.push(wire);const match=/fetch (map|path|cloud):(\d+) (\d+)/.exec(wire);if(!match)return {};
    const [,kind,idText,pageText]=match,requested=Number(idText),page=Number(pageText),newer=requested===8||requested===9||requested===12;
    if(kind==='cloud'&&!newer){releaseCloud=()=>client.receive(line({scan:11,map:5,plan:6,k:'host_cloud',page:0,pages:1,offset:0,total:0,crc32:'00000000',data:''}));return {};}
    const records=kind==='map'?mapRecords:pathRecords;
    queueMicrotask(()=>client.receive(line({scan:newer?12:11,map:newer?8:5,plan:newer?9:6,k:`host_${kind}`,page,pages:records.length,offset:page,total:records.length,data:records[page]})));return {};
  };
  client=new RadarHostClient({send,model,timeoutMs:100});
  client.receive(line({scan:11,map:5,plan:6,k:'host_status',a:0,l:0,ss:4,ls:3,cloud:0,preview:0,reason:'complete'}));
  for(let i=0;i<70&&!releaseCloud;i++)await flush();assert.ok(releaseCloud);
  client.receive(line({scan:12,map:8,plan:9,k:'host_status',a:0,l:0,ss:4,ls:3,cloud:0,preview:0,reason:'complete'}));
  releaseCloud();for(let i=0;i<70&&!wires.some(w=>w.startsWith('radar host fetch path:9'));i++)await flush();
  assert.ok(wires.some(w=>w.startsWith('radar host fetch map:8')));assert.ok(wires.some(w=>w.startsWith('radar host fetch path:9')));
  assert.notEqual(model.snapshots.cloud?.scan,11);
});

test('cancelTransfer is local and never sends a radar stop',()=>{
  const f=fixture();f.client.cancelTransfer();assert.deepEqual(f.wires,[]);
});

test('applyConfig stages fields, applies the draft revision and waits for active readback',async()=>{
  const wires=[],model=new RadarHostModel();let client,getCount=0;
  const config=(source,version)=>['source,'+source,'threshold,1','protected,00404405','radar_x,230','radar_y,230'].map((data,page)=>
    line({config:version,k:'host_config',page,pages:5,offset:page,total:5,data}));
  const send=async wire=>{frameCommand(wire);wires.push(wire);
    if(wire==='radar host get draft')for(const page of config('draft',7))queueMicrotask(()=>client.receive(page));
    if(wire==='radar host get active'&&++getCount===1)for(const page of config('active',8))queueMicrotask(()=>client.receive(page));
    return {};
  };
  client=new RadarHostClient({send,model,timeoutMs:100});
  const result=await client.applyConfig({threshold:1,mission:[1,5,4,2,3,4,2,3,1],start:{row:1,column:1,anchor:'radar'}});
  assert.equal(result.config,8);assert.deepEqual(wires,[
    'radar host draft reset','radar host get draft','radar host set threshold 1',
    'radar host set mission 1-5-4-2-3-4-2-3-1','radar host set start 1,1,radar',
    'radar host apply 7','radar host get active',
  ]);for(const wire of wires)frameCommand(wire);
});

test('failed apply retains page draft and completed device results',async()=>{
  const f=fixture();f.model.accept(line({k:'host_config',page:0,pages:1,offset:0,total:1,data:'source,active'}));
  f.client.send=async wire=>{f.wires.push(wire);throw new Error('write failed');};
  await assert.rejects(f.client.applyConfig({threshold:1}),/write failed/);
  assert.deepEqual(f.client.draft,{threshold:1});assert.equal(f.model.snapshots.config.source,'active');
});
