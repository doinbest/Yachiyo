import test from 'node:test';
import assert from 'node:assert/strict';
import * as radar from '../radar-model.mjs';
import {BridgeLink,readOnlyCommand,scopedStopCommand} from '../bridge.mjs';
import {frameCommand,moduleForWire} from '../protocol.mjs';
import {chineseTerminalLine} from '../terminal-format.mjs';

test('view zoom preserves the pointer anchor and panning preserves the current map orientation',()=>{
  const view={x:-320,y:-370,width:3170,height:2990},anchor={x:725,y:1600};
  const next=radar.zoomView(view,anchor,.8);
  assert.equal((anchor.x-next.x)/next.width,(anchor.x-view.x)/view.width);
  assert.equal((anchor.y-next.y)/next.height,(anchor.y-view.y)/view.height);
  assert.deepEqual(radar.panView(next,{x:20,y:-30}),{...next,x:next.x-20,y:next.y+30});
});

test('complete radar pages are assembled out of order without mixing maps or erasing history',()=>{
  const model=new radar.RadarModel(),base={v:1,k:'map',session:9,origin:2,scan:3,map:7,plan:0,pages:2,mask:4,pose:[2250,150,0]};
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

test('radar parameter set requires device readback and never treats TX as application',async()=>{
  const wires=[];let exchange;
  exchange=new radar.RadarExchange({send:async wire=>{
    wires.push(wire);
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
  model.accept('@RADAR '+JSON.stringify({...base,k:'map',mask:0,pose:[2170,230,180],counts:[0],offset:0}));
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
  const map='@RADAR '+JSON.stringify({...base,k:'map',mask:0,pose:[2170,230,180],counts:[0],offset:0});
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
