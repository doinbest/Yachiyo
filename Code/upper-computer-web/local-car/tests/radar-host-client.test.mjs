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
const previewLine=(previewId,distance=100)=>{
  const bytes=Uint8Array.from([0,0,distance&255,distance>>>8]);
  return line({k:'host_preview',live_id:previewId,page:0,pages:1,offset:0,total:1,
    crc32:crc32(bytes).toString(16).padStart(8,'0').toUpperCase(),data:Buffer.from(bytes).toString('base64')});
};

function fixture(){
  const wires=[],model=new RadarHostModel();let client;
  const send=async wire=>{
    frameCommand(wire);wires.push(wire);
    if(wire==='radar host result 4')for(const kind of ['map','path']){
      const records=kind==='map'?mapRecords:pathRecords;
      records.forEach((data,page)=>queueMicrotask(()=>client.receive(line({k:`host_${kind}`,page,pages:records.length,offset:page,total:records.length,data}))));
    }
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

test('ready scan requests the map/path pair once, never automatic config or cloud',async()=>{
  const f=fixture();await f.client.command('scan');
  f.client.receive(line({k:'host_status',a:0,l:0,ss:4,ls:3,cloud:1,preview:0,reason:'complete'}));
  for(let i=0;i<80&&!f.model.snapshots.path;i++)await flush();
  assert.equal(f.model.snapshots.map.counts.length,25);assert.equal(f.model.snapshots.path.points.length,2);assert.equal(f.model.snapshots.cloud,undefined);
  assert.ok(f.wires.includes('radar host result 4'));
  assert.ok(!f.wires.some(w=>/^radar host fetch (map|path):/.test(w)));
  assert.ok(!f.wires.some(w=>/^radar host (?:get|fetch (?:config|cloud):)/.test(w)));
});

test('a newer scan cancels old path reads and compact results switch identities',async()=>{
  const wires=[],model=new RadarHostModel();let client,releasePath;
  const send=async wire=>{wires.push(wire);const match=/^radar host result (6|9)$/.exec(wire);if(!match)return {};
    const newer=Number(match[1])===9;
    for(const kind of ['map','path']){
      const records=kind==='map'?mapRecords:pathRecords;
      const deliver=()=>records.forEach((data,page)=>client.receive(line({scan:newer?12:11,map:newer?8:5,plan:newer?9:6,k:`host_${kind}`,page,pages:records.length,offset:page,total:records.length,data})));
      if(kind==='path'&&!newer)releasePath=deliver;else queueMicrotask(deliver);
    }return {};
  };
  client=new RadarHostClient({send,model,timeoutMs:100});
  client.receive(line({scan:11,map:5,plan:6,k:'host_status',a:0,l:0,ss:4,ls:3,cloud:0,preview:0,reason:'complete'}));
  for(let i=0;i<70&&!releasePath;i++)await flush();assert.ok(releasePath);
  client.receive(line({scan:12,map:8,plan:9,k:'host_status',a:0,l:0,ss:4,ls:3,cloud:0,preview:0,reason:'complete'}));
  releasePath();for(let i=0;i<70&&!wires.includes('radar host ack 9');i++)await flush();
  assert.ok(wires.includes('radar host result 9'));assert.ok(wires.includes('radar host ack 9'));
  assert.notEqual(model.snapshots.path?.scan,11);
});

test('continuous updates wait for the in-flight map and path before reading the latest circle',async()=>{
  const wires=[],model=new RadarHostModel();let client,releaseFirst;
  const send=async wire=>{
    wires.push(wire);const match=/^radar host result (6|9)$/.exec(wire);if(!match)return {};
    const newer=Number(match[1])===9;
    const deliver=()=>{for(const kind of ['map','path'])client.receive(line({scan:newer?12:11,map:newer?8:5,plan:newer?9:6,k:`host_${kind}`,page:0,pages:1,offset:0,total:kind==='map'?28:2,data:(kind==='map'?mapRecords:pathRecords).join(';')}));};
    if(!newer)releaseFirst=deliver;else queueMicrotask(deliver);
    return {};
  };
  client=new RadarHostClient({send,model,timeoutMs:100});client.connection(true);
  client.receive(line({scan:11,map:5,plan:6,k:'host_status',a:0,l:1,ss:4,ls:2,reason:'complete'}));
  await flush();assert.ok(releaseFirst);
  client.receive(line({scan:12,map:8,plan:9,k:'host_status',a:0,l:1,ss:4,ls:2,reason:'complete'}));
  assert.ok(!wires.includes('radar host result 9'));
  releaseFirst();for(let i=0;i<25&&model.snapshots.path?.scan!==12;i++)await flush();
  assert.ok(wires.includes('radar host ack 6'));
  assert.ok(wires.includes('radar host result 9'));
  assert.equal(model.snapshots.path?.scan,12);
});

test('a dropped page retries only that page and still completes without a cloud read',async()=>{
  const f=fixture(),send=f.client.send;let dropped=false;
  f.client.timeoutMs=15;
  f.client.send=async wire=>{
    if(wire==='radar host fetch map:3 7'&&!dropped){dropped=true;f.wires.push(wire);return {};}
    return send(wire);
  };
  const result=await f.client.fetch('map',3);
  assert.equal(result.counts.length,25);
  assert.equal(f.wires.filter(w=>w==='radar host fetch map:3 7').length,2);
  assert.ok(!f.wires.some(w=>w.includes('cloud:')));
});

test('cancelTransfer is local and never sends a radar stop',()=>{
  const f=fixture();f.client.cancelTransfer();assert.deepEqual(f.wires,[]);
});

test('an explicit preview fetch uses its own circle id, not the static scan id',async()=>{
  const f=fixture();
  f.client.send=async wire=>{f.wires.push(wire);if(wire==='radar host fetch preview:101 0')queueMicrotask(()=>f.client.receive(previewLine(101)));return {};};
  const result=await f.client.fetch('preview',101);
  assert.equal(result.live_id,101);assert.equal(result.scan,2);
  assert.deepEqual(f.wires,['radar host fetch preview:101 0']);
});

test('a valid newer preview clears a preview error but cannot hide a map error',()=>{
  const f=fixture();f.client.connection(true);
  const bad=JSON.parse(previewLine(100).slice(7));bad.crc32='00000000';
  f.client.receive('@RADAR '+JSON.stringify(bad));assert.match(f.client.lastError,/crc_mismatch/);
  f.client.receive(previewLine(101));assert.equal(f.client.lastError,null);
  f.client.receive(line({k:'host_map',page:0,pages:1,offset:0,total:1,data:'s,1,1,0'}));
  assert.match(f.client.lastError,/map_incomplete/);
  f.client.receive(previewLine(102));assert.match(f.client.lastError,/map_incomplete/);
});

test('explicit status reads RAM config even while idle without reading points or unavailable results',async()=>{
  const f=fixture();
  f.client.send=async wire=>{
    f.wires.push(wire);
    if(wire==='radar host status')queueMicrotask(()=>f.client.receive(line({k:'host_status',ss:0,a:0,l:0,ls:3,reason:'idle'})));
    if(wire==='radar host fetch config:1 0')queueMicrotask(()=>f.client.receive(line({k:'host_config',page:0,pages:1,offset:0,total:1,data:'source,active'})));
    return {};
  };
  await f.client.command('status');for(let i=0;i<10&&!f.model.snapshots.config;i++)await flush();
  assert.equal(f.model.snapshots.config.source,'active');
  assert.deepEqual(f.wires,['radar host status','radar host fetch config:1 0']);
});

test('a rejected serial write is not reported as a successful radar command',async()=>{
  const f=fixture();f.client.send=async()=>false;
  await assert.rejects(f.client.command('scan'),/未写出/);
});

test('single scan ignores a late ready status for the previous circle while its command is being written',async()=>{
  const f=fixture();f.client.connection(true);
  f.model.accept(line({k:'host_status',a:0,l:0,ss:4,ls:3,reason:'complete'}));
  let finishWrite;
  f.client.send=wire=>{f.wires.push(wire);return new Promise(resolve=>{finishWrite=resolve;});};
  const scan=f.client.command('scan');await flush();
  f.client.receive(line({k:'host_status',a:0,l:0,ss:4,ls:3,reason:'complete'}));
  await flush();assert.deepEqual(f.wires,['radar host scan']);
  finishWrite({});await scan;f.client.cancelTransfer();
});

test('a scan begun by another client cancels old result repair and preserves completed history',async()=>{
  const wires=[],errors=[],model=new RadarHostModel();
  const client=new RadarHostClient({send:async wire=>{wires.push(wire);return {};},model,onError:e=>errors.push(e.message),timeoutMs:50});
  client.connection(true);
  model.accept(line({k:'host_path',page:0,pages:1,offset:0,total:2,data:pathRecords.join(';')}));
  const history=model.snapshots.path;
  client.receive(line({k:'host_status',a:0,l:0,ss:4,ls:3,reason:'complete'}));await flush();
  client.receive(line({k:'host_status',a:1,l:0,ss:1,ls:3,reason:'configuring'}));
  client.receive(line({k:'host_result_end',mp:28,pp:2}));
  for(let i=0;i<5;i++)await flush();
  assert.deepEqual(wires,['radar host result 4']);assert.deepEqual(errors,[]);
  assert.equal(model.snapshots.path,history);
  client.cancelTransfer();
});

test('a live restart preparing an old ready circle does not request that circle again',async()=>{
  const f=fixture();f.client.connection(true);
  f.client.receive(line({k:'host_status',a:0,l:1,ss:4,ls:0,reason:'configuring'}));
  await flush();assert.deepEqual(f.wires,[]);f.client.cancelTransfer();
});

test('new single scan waits for the current radar write and cannot acknowledge cancelled old results',async()=>{
  const wires=[],errors=[],model=new RadarHostModel();let client,finishResult;
  client=new RadarHostClient({model,onError:e=>errors.push(e.message),send:wire=>{
    wires.push(wire);
    if(wire==='radar host result 4')return new Promise(resolve=>{finishResult=resolve;});
    return Promise.resolve({});
  }});
  client.connection(true);client.receive(line({k:'host_status',a:0,l:0,ss:4,ls:3,reason:'complete'}));
  await flush();
  for(const kind of ['map','path'])client.receive(line({k:`host_${kind}`,page:0,pages:1,offset:0,total:kind==='map'?28:2,data:(kind==='map'?mapRecords:pathRecords).join(';')}));
  const scan=client.command('scan');await flush();
  assert.deepEqual(wires,['radar host result 4']);
  finishResult({});await scan;for(let i=0;i<5;i++)await flush();
  assert.deepEqual(wires,['radar host result 4','radar host scan']);assert.deepEqual(errors,[]);
  client.cancelTransfer();
});

test('a cancelled old write failing later cannot replace the new scan with an obsolete error',async()=>{
  const wires=[],errors=[];let failOld;
  const client=new RadarHostClient({onError:e=>errors.push(e.message),send:wire=>{
    wires.push(wire);
    if(wire==='radar host result 4')return new Promise((resolve,reject)=>{failOld=reject;});
    return Promise.resolve({});
  }});
  client.connection(true);client.receive(line({k:'host_status',a:0,l:0,ss:4,ls:3,reason:'complete'}));await flush();
  const scan=client.command('scan');failOld(new Error('old queued read cancelled'));
  await scan;await flush();
  assert.deepEqual(wires,['radar host result 4','radar host scan']);
  assert.deepEqual(errors,[]);assert.equal(client.lastError,null);client.cancelTransfer();
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
