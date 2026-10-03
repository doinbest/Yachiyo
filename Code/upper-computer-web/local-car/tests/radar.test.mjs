import test from 'node:test';
import assert from 'node:assert/strict';
import * as radar from '../radar-model.mjs';
import {BridgeLink,readOnlyCommand,scopedStopCommand} from '../bridge.mjs';
import {frameCommand,moduleForWire} from '../protocol.mjs';
import {chineseTerminalLine} from '../terminal-format.mjs';

test('complete radar pages are assembled out of order without mixing maps or erasing history',()=>{
  const model=new radar.RadarModel(),base={v:1,k:'map',session:9,origin:2,scan:3,map:7,plan:0,pages:2};
  model.accept('@RADAR '+JSON.stringify({...base,page:1,offset:5,counts:[6,7,8,9,10]}),100);
  assert.equal(model.snapshots.map,undefined);
  model.accept('@RADAR '+JSON.stringify({...base,page:0,offset:0,counts:[1,2,3,4,5]}),110);
  assert.deepEqual(model.snapshots.map.counts,[1,2,3,4,5,6,7,8,9,10]);
  model.connection(false);
  assert.equal(model.snapshots.map.map,7);
  assert.equal(model.describe(model.snapshots.map,1110).includes('历史'),true);
  model.accept('@RADAR '+JSON.stringify({...base,session:10,map:8,page:0,offset:0,counts:[22]}),1200);
  assert.equal(model.snapshots.map.map,7);
});

test('serial page fetch waits for its exact reply and ignores unrelated and old data',async()=>{
  const wires=[];let fetcher;
  fetcher=new radar.RadarExchange({send:async wire=>{
    wires.push(wire);
    const page=Number(wire.split(' ').at(-1));
    queueMicrotask(()=>{
      fetcher.receive('@RADAR '+JSON.stringify({v:1,k:'map',session:1,origin:2,scan:1,map:8,plan:0,page,pages:2,counts:[],offset:page*5}));
      fetcher.receive('@RADAR '+JSON.stringify({v:1,k:'map',session:1,origin:2,scan:1,map:7,plan:0,page,pages:2,counts:[page],offset:page}));
    });return true;
  },intervalMs:0});
  const result=await fetcher.fetch('map',7);
  assert.equal(result.length,2);
  assert.deepEqual(wires,['radar fetch map 7 0','radar fetch map 7 1']);
});

test('radar write rejected before get preserves the device reason',async()=>{
  const wires=[];let exchange;
  exchange=new radar.RadarExchange({send:async wire=>{wires.push(wire);queueMicrotask(()=>exchange.receive('ERR radar busy'));return true;},timeoutMs:20});
  await assert.rejects(exchange.set('energy_min',21),/ERR radar busy/);
  assert.deepEqual(wires,['radar set energy_min 21']);
});

test('radar parameter set requires device readback and never treats TX as application',async()=>{
  const wires=[];let exchange;
  exchange=new radar.RadarExchange({send:async wire=>{
    wires.push(wire);
    if(wire.startsWith('radar set '))queueMicrotask(()=>exchange.receive('OK radar parameter_updated_ram'));
    if(wire==='radar get')queueMicrotask(()=>exchange.receive('@RADAR '+JSON.stringify({v:1,k:'params',session:1,origin:1,scan:0,map:0,plan:0,page:0,pages:1,key:'energy_min',value:20})));
    return true;
  },intervalMs:0});
  assert.deepEqual(await exchange.set('energy_min',21),{key:'energy_min',value:20,confirmed:false});
  assert.deepEqual(wires,['radar set energy_min 21','radar get']);
});

test('reference coordinates and cell mask rotate 90 degrees counterclockwise only on import',()=>{
  const imported=radar.importSnapshot({coordinate_frame:'reference',mask:1,counts:Array.from({length:25},(_,i)=>i),points:[[100,200,5,0]],pose:[100,200,0]});
  assert.deepEqual(imported.points[0],[2200,100,5,0]);
  assert.equal(imported.mask,1<<4);
  assert.equal(imported.counts[4],0);
  assert.equal(imported.coordinate_frame,'current');
});

test('radar readbacks and cancel remain available while stop is latched; motion stays explicit',()=>{
  for(const wire of ['radar status','radar get','radar fetch map 4294967295 0','radar nav status']){
    assert.equal(readOnlyCommand(wire),true);assert.equal(moduleForWire(wire),'map');frameCommand(wire);
  }
  assert.equal(scopedStopCommand('radar nav cancel'),true);
  assert.equal(scopedStopCommand('radar stop'),true);
  assert.equal(readOnlyCommand('radar nav start 100'),false);
  frameCommand('radar set angle_offset -900');
});

test('firmware metadata page precedes five count pages; params use id zero and clouds use scan id',async()=>{
  const model=new radar.RadarModel(),base={v:1,k:'map',session:1,origin:2,scan:3,map:7,plan:8,pages:6};
  model.accept('@RADAR '+JSON.stringify({...base,page:0,mask:4,pose:[2170,230,180],applicable:true}),0);
  for(let page=1;page<=5;page++)model.accept('@RADAR '+JSON.stringify({...base,page,offset:(page-1)*5,counts:[0,1,2,3,4]}),page);
  assert.equal(model.snapshots.map.counts.length,25);
  const wires=[];let exchange;
  exchange=new radar.RadarExchange({intervalMs:0,send:async wire=>{wires.push(wire);queueMicrotask(()=>exchange.receive('@RADAR '+JSON.stringify({...base,k:'cloud',page:0,pages:1,offset:0,points:[[900,100,50]]})));return true;}});
  await exchange.fetch('cloud',3);
  assert.deepEqual(wires,['radar fetch cloud 3 0']);
});

test('radar summary explains current stop and invalid map reasons while retaining unknown diagnostics',()=>{
  assert.equal(radar.radarStateText('wait_wrap'),'等待完整圈起点');
  assert.match(chineseTerminalLine('ERR radar map_origin_or_parameters_changed'),/请重新扫描/);
  assert.equal(radar.radarReasonText('future_reason'),'future_reason');
  assert.equal(moduleForWire('ERR radar leg_unreachable'),'map');
});

test('parameter pages always fetch id zero and changing revision never confirms mixed readback',async()=>{
  let exchange;const wires=[];
  exchange=new radar.RadarExchange({intervalMs:0,send:async wire=>{
    wires.push(wire);const page=wire==='radar get'?0:Number(wire.split(' ').at(-1));
    queueMicrotask(()=>exchange.receive('@RADAR '+JSON.stringify({v:1,k:'params',session:99,origin:1,scan:0,map:0,plan:0,page,pages:2,rev:page+1,key:page?'threshold':'energy_min',value:3})));return true;
  }});
  await assert.rejects(exchange.get(),/设备结果已更新/);
  assert.deepEqual(wires,['radar get','radar fetch params 0 1']);
});

test('radar transport reaches the snapshot consumer independently of terminal RX history',()=>{
  const lines=[],bridge=new BridgeLink({receive:line=>lines.push(line)});
  bridge.deliver({id:1,kind:'radar',text:'@RADAR {"v":1}'});
  assert.deepEqual(lines,['@RADAR {"v":1}']);
});

test('a new scan keeps the previous complete cloud and labels it historical even when the old map remains',()=>{
  const model=new radar.RadarModel(),base={v:1,session:1,origin:2,scan:3,map:7,plan:0,page:0,pages:1};
  model.connection(true);
  model.accept('@RADAR '+JSON.stringify({...base,k:'map',mask:0,pose:[2170,230,180],counts:Array(25).fill(0),offset:0}));
  model.accept('@RADAR '+JSON.stringify({...base,k:'cloud',points:[[900,200,50]],offset:0,total:1}));
  assert.deepEqual(model.snapshots.cloud.pose,[2170,230,180]);
  model.accept('@RADAR '+JSON.stringify({...base,k:'status',scan:4,state:'collecting',points_valid:false}));
  assert.equal(model.snapshots.cloud.scan,3);
  assert.match(model.describe(model.snapshots.cloud),/历史/);
});

test('three-sample firmware pages assemble without assuming the previous four-sample page size',async()=>{
  const model=new radar.RadarModel(),base={v:1,session:4294967295,origin:4294967294,scan:4294967293,map:4294967292,plan:4294967291};
  const samples=[[0,100,50],[900,200,60],[1800,300,70],[2700,400,80],[3599,500,90]],wires=[];let exchange;
  exchange=new radar.RadarExchange({intervalMs:0,send:async wire=>{
    wires.push(wire);const page=Number(wire.split(' ').at(-1)),line='@RADAR '+JSON.stringify({...base,k:'points',page,pages:2,offset:page*3,total:5,truncated:false,points:samples.slice(page*3,page*3+3)});
    queueMicrotask(()=>{model.accept(line);exchange.receive(line);});return true;
  }});
  await exchange.fetch('points',base.scan);
  assert.deepEqual(model.snapshots.points.points,samples);
  assert.deepEqual(wires,[`radar fetch points ${base.scan} 0`,`radar fetch points ${base.scan} 1`]);
});

test('reconnecting preserves historical labels until that kind is fully read again, even if IDs coincide',()=>{
  const model=new radar.RadarModel(),base={v:1,session:1,origin:2,scan:3,map:7,plan:0,page:0,pages:1};
  const map='@RADAR '+JSON.stringify({...base,k:'map',mask:0,pose:[2170,230,180],counts:Array(25).fill(0),offset:0});
  model.connection(true);model.accept(map);
  model.accept('@RADAR '+JSON.stringify({...base,k:'cloud',points:[[900,200,50]],offset:0,total:1}));
  assert.doesNotMatch(model.describe(model.snapshots.map),/历史/);
  model.connection(false);model.connection(true);
  assert.equal(model.status,null);
  assert.match(model.describe(model.snapshots.map),/历史/);
  model.accept('@RADAR '+JSON.stringify({...base,k:'status',state:'ready',points_valid:true}));
  assert.match(model.describe(model.snapshots.map),/历史/);
  model.accept('@RADAR '+JSON.stringify({...base,k:'map',pages:6,mask:0,pose:[2170,230,180]}));
  assert.match(model.describe(model.snapshots.map),/历史/);
  model.accept(map);
  assert.doesNotMatch(model.describe(model.snapshots.map),/历史/);
  assert.match(model.describe(model.snapshots.cloud),/历史/);
});

test('firmware maps require all 25 counts independently of compact page capacities',()=>{
  const model=new radar.RadarModel(),base={v:1,k:'map',session:1,origin:2,scan:3,map:7,plan:8,pages:3};
  const frame=f=>'@RADAR '+JSON.stringify({...base,...f});
  model.accept(frame({page:0,mask:4,pose:[2170,230,180]}));
  model.accept(frame({page:1,offset:0,counts:Array(7).fill(1)}));
  model.accept(frame({page:2,offset:7,counts:Array(17).fill(2)}));
  assert.equal(model.snapshots.map,undefined,'a short compact map must not publish a partial grid');
  model.accept(frame({page:2,offset:7,counts:Array(18).fill(2)}));
  assert.deepEqual(model.snapshots.map.counts,[...Array(7).fill(1),...Array(18).fill(2)]);
});

test('default page transport immediately requests the next page after the matched reply',async()=>{
  const wires=[];let exchange;
  exchange=new radar.RadarExchange({send:async wire=>{
    wires.push(wire);const page=Number(wire.split(' ').at(-1));
    queueMicrotask(()=>exchange.receive('@RADAR '+JSON.stringify({v:1,k:'path',session:1,origin:2,scan:3,map:7,plan:8,page,pages:2,offset:page,points:[[page,100,0,0]]})));return true;
  }});
  const reading=exchange.fetch('path',8);reading.catch(()=>{});
  try{await new Promise(setImmediate);assert.equal(wires.length,2,'matched pages should have no one-second browser sleep');await reading;}
  finally{exchange.cancel();await reading.catch(()=>{});}
});

test('start classification pages publish atomically and retain both candidate scores',()=>{
  const model=new radar.RadarModel(),base={v:1,k:'start',session:9,origin:2,scan:3,map:7,plan:8,id:3,pages:2};
  model.connection(true);
  model.accept('@RADAR '+JSON.stringify({...base,page:1,qr:[0,4],raw:[0,5],conflicts:[0,2]}));assert.equal(model.snapshots.start,undefined);
  model.accept('@RADAR '+JSON.stringify({...base,page:0,auto:true,selected:1,reason:'clear_regions'}));
  assert.ok(model.snapshots.start,'classification should publish after all score pages arrive');
  assert.equal(model.snapshots.start.selected,1);assert.deepEqual(model.snapshots.start.qr,[0,4]);assert.deepEqual(model.snapshots.start.conflicts,[0,2]);
  model.connection(false);assert.equal(model.snapshots.start.historical,true);
});

test('start classification supports an extra budget page without mixing completed circles',()=>{
  const model=new radar.RadarModel(),base={v:1,k:'start',session:4294967295,origin:4294967294,scan:4294967293,map:4294967292,plan:4294967291,id:4294967293,pages:3};
  const frame=value=>'@RADAR '+JSON.stringify({...base,...value});
  model.accept(frame({page:0,auto:true,selected:0,reason:'both_conflict'}));
  model.accept(frame({page:1,qr:[1,2],raw:[3,4]}));model.accept(frame({page:2,id:22,conflicts:[2,2]}));assert.equal(model.snapshots.start,undefined);
  model.accept(frame({page:2,conflicts:[2,2]}));assert.ok(model.snapshots.start,'classification should assemble variable score pages');assert.equal(model.snapshots.start.reason,'both_conflict');assert.deepEqual(model.snapshots.start.raw,[3,4]);
});

test('offline copies retain only the start and path belonging to their complete map',()=>{
  const base={session:9,origin:2,scan:3,map:7,plan:8},map={...base,mask:0,counts:Array(25).fill(0),pose:[2330,2170,0],coordinate_frame:'current'};
  const upper=radar.importSnapshot({snapshots:{map,start:{...base,selected:1},path:{...base,start:1,points:[[2250,2250,1,0]]}},params:{lidar_x_mm:2170,lidar_y_mm:230,zero_deg:180}});
  assert.equal(upper.start_id,1);assert.equal(upper.start_mode,'upper');assert.equal(radar.importSnapshot(JSON.parse(JSON.stringify(upper))).start_id,1);
  const untouchedOffline=radar.importSnapshot({snapshots:{map,start:{...base,selected:1}},offline:{mask:0x50140,coordinate_frame:'current'}});
  assert.equal(untouchedOffline.start_id,1);assert.equal(untouchedOffline.mask,0);
  const lower=radar.importSnapshot({snapshots:{map,start:{...base,scan:4,selected:0},path:{...base,start:2,points:[[2250,150,1,0]]}}});
  assert.equal(lower.start_id,2);assert.equal(lower.start_mode,'lower');
  const unrelated=radar.importSnapshot({snapshots:{map,start:{...base,scan:4,selected:1},path:{...base,map:8,start:1,points:[[2250,2250,1,0]]}}});
  assert.equal(unrelated.start_id,undefined);assert.equal(unrelated.start_mode,'manual');assert.deepEqual(unrelated.points,[]);
});

test('exported offline obstacle edits and selected corner take precedence over the original device snapshot',()=>{
  const copy=radar.importSnapshot({snapshots:{map:{mask:0,counts:Array(25).fill(0),coordinate_frame:'current'}},offline:{mask:1<<21,manual_mask:1<<21,start_id:1,start_mode:'upper',pose:[2330,2170,0],coordinate_frame:'current'}});
  assert.equal(copy.mask,1<<21);assert.equal(copy.manual_mask,1<<21);assert.equal(copy.start_id,1);
});


test('a matching complete raw cloud takes precedence over a newer display sample on offline import',()=>{
  const base={session:9,origin:2,scan:3,map:7,plan:8};
  const full={...base,k:'cloud',complete:true,truncated:false,receivedAt:100,points:[[90,800,30],[100,800,30]]};
  const sample={...base,k:'points',complete:true,truncated:false,receivedAt:200,points:[[90,800,30]]};
  const result=radar.importSnapshot({snapshots:{map:{...base,mask:0,pose:[2250,150,180]},cloud:full,points:sample}});
  assert.equal(result.cloud,full);
});

test('bare polar points can declare cloud completeness without an inferred point-count rule',()=>{
  const points=[[90,800,30],[100,800,30]];
  const result=radar.importSnapshot({mask:0,start_id:2,pose:[2250,150,180],points,cloud_complete:true});
  assert.deepEqual(result.cloud?.points,points);assert.equal(result.cloud?.cloud_complete,true);
});
