import test from 'node:test';
import assert from 'node:assert/strict';
import {RADAR_INSTALLATION_PRESETS,installationParameters,projectFrozenRadarPoint,stableFieldPoint} from '../radar-installation.mjs';
import {RadarHostModel} from '../radar-host-model.mjs';
import {RadarModel,RadarExchange,parseRadar} from '../radar-model.mjs';

const checksum=bytes=>{let crc=0xffffffff;for(const b of bytes){crc^=b;for(let bit=0;bit<8;bit++)crc=(crc>>>1)^((crc&1)?0xedb88320:0);}return ((crc^0xffffffff)>>>0).toString(16).padStart(8,'0').toUpperCase();};
const withCrc=body=>'@RADAR '+body.slice(0,-1)+',"crc32":"'+checksum(Buffer.from(body))+'"}';

test('formal CRC protects original float literals and rejects a changed page',()=>{
  const body='{"v":1,"k":"map","i":[1,2,3,4,5],"page":0,"pages":1,"pose":[2250.0,150.0,180.0],"offset":0,"counts":[0]}';
  assert.deepEqual(parseRadar(withCrc(body)).pose,[2250,150,180]);
  assert.equal(parseRadar(withCrc(body).replace('2250.0','2251.0')),null);
});

test('formal frozen result repairs CRC-damaged map and missing stable pages before ACK while rolling advances',async()=>{
  const model=new RadarModel(),wires=[],identity={session:1,origin:2,scan:3,map:4,plan:5};model.connection(true);
  const bytes=Buffer.alloc(20);for(let i=0;i<2;i++){[1800,900,1300+i*10,320].forEach((v,j)=>bytes.writeUInt16LE(v,i*10+j*2));bytes[i*10+8]=4;bytes[i*10+9]=1;}
  const base={v:1,...identity};
  const map0={...base,k:'map',page:0,pages:2,g:4,mask:0,pose:[2280,314,180]};
  const map1={...base,k:'map',page:1,pages:2,algorithm:1,window:[1,15,15],geometry:[65809,680,-407,66623,149407997,20569846,1800],offset:0,counts:Array(25).fill(0)};
  const path={...base,k:'path',page:0,pages:1,offset:0,valid:true,points:[[2250,150,1,0]]};
  const stable=page=>({...base,k:'stable',id:3,page,pages:2,offset:page,total:2,crc32:checksum(bytes),data:bytes.subarray(page*10,(page+1)*10).toString('base64')});
  let exchange;const emit=line=>{model.accept(line);exchange.receive(line);};
  exchange=new RadarExchange({timeoutMs:100,send:async wire=>{wires.push(wire);queueMicrotask(()=>{
    if(wire==='radar result 5'){
      emit(withCrc(JSON.stringify(map0)));emit(withCrc(JSON.stringify(map1)).replace(/"crc32":"[A-F0-9]+"/, '"crc32":"00000000"'));emit(withCrc(JSON.stringify(path)));emit('@RADAR '+JSON.stringify(stable(0)));
      emit('@RADAR '+JSON.stringify({...base,k:'status',state:'collecting',scan:4,map:5,plan:6,live_active:true,mapping:true}));
      emit('@RADAR '+JSON.stringify({...base,k:'result_end',mp:2,pp:1,sp:2}));
    }else if(wire==='radar fetch map 4 1')emit(withCrc(JSON.stringify(map1)));
    else if(wire==='radar fetch stable 3 1')emit('@RADAR '+JSON.stringify(stable(1)));
  });return true;}});
  await exchange.result(identity);
  assert.deepEqual(wires,['radar result 5','radar fetch map 4 1','radar fetch stable 3 1','radar ack 5']);
  assert.equal(model.status.scan,4);assert.equal(model.snapshots.map.scan,3);assert.equal(model.snapshots.stable.points.length,2);
});

test('formal stable CRC failure retains the device freeze without sending ACK',async()=>{
  const identity={session:1,origin:2,scan:3,map:4,plan:5},wires=[];let exchange;
  exchange=new RadarExchange({timeoutMs:100,send:async wire=>{wires.push(wire);queueMicrotask(()=>{
    if(wire!=='radar result 5')return;
    for(const frame of [{k:'map',page:0,pages:1,g:4,algorithm:1,offset:0,counts:Array(25).fill(0)},{k:'path',page:0,pages:1,offset:0,points:[]},{k:'stable',id:3,page:0,pages:1,offset:0,total:1,crc32:'00000000',data:Buffer.alloc(10).toString('base64')}])exchange.receive('@RADAR '+JSON.stringify({v:1,...identity,...frame}));
  });return true;}});
  await assert.rejects(exchange.result(identity),/稳定XY校验失败/);assert.deepEqual(wires,['radar result 5']);
});
test('case2 keeps the exact Q16 origin and affine coefficients',()=>{
  assert.deepEqual(RADAR_INSTALLATION_PRESETS.case2.matrix_q16,[65809,680,-407,66623]);
  const values=installationParameters('case2');
  assert.equal(values.lidar_x_mm,2280);assert.equal(values.lidar_y_mm,314);assert.equal(values.tx_q16,149407997);assert.equal(values.ty_q16,20569846);assert.equal(values.zero_tenths,1800);
  assert.equal(installationParameters('upper').m00_q16,65536);assert.equal(installationParameters('upper').ty_q16,2250*65536);
});
test('stable XY is used directly and old raw points use their frozen geometry',()=>{
  assert.deepEqual(stableFieldPoint([1800,900,1300,320,4,1]),[1300,320]);
  assert.deepEqual(projectFrozenRadarPoint([0,100,40],{pose:[2250,150,180]}),[2150,150]);
  const frozen={pose:[2280,314,180],geometry:{matrix_q16:[65536,0,0,65536],origin_q16:[100*65536,200*65536],zero_tenths:0}};
  assert.deepEqual(projectFrozenRadarPoint([0,100,40],frozen),[200,200]);
});

test('a raw cloud downloaded before its map retains calibration when a later map replaces that capture',()=>{
  const model=new RadarModel();model.connection(true);
  const base={v:1,session:1,origin:2,scan:3,map:4,plan:0,page:0,pages:1};
  model.accept('@RADAR '+JSON.stringify({...base,k:'cloud',pose:[2280,314,180],offset:0,total:1,points:[[0,1000,40]]}));
  const geometry=[65809,680,-407,66623,149407997,20569846,1800];
  model.accept('@RADAR '+JSON.stringify({...base,k:'map',plan:5,g:4,pose:[2280,314,180],geometry,offset:0,counts:Array(25).fill(0)}));
  model.accept('@RADAR '+JSON.stringify({...base,k:'map',scan:6,map:7,plan:8,g:4,pose:[2050,2250,270],geometry:[65536,0,0,65536,134348800,147456000,2700],offset:0,counts:Array(25).fill(0)}));
  const cloud=model.snapshots.cloud;
  assert.equal(cloud.scan,3);assert.deepEqual(cloud.geometry,geometry);
  assert.deepEqual(projectFrozenRadarPoint(cloud.points[0],cloud),[1275.6194610595703,320.081268310547]);
});
test('host stable packed records preserve votes and field XY after CRC validation',()=>{
  const bytes=Buffer.alloc(10);[1800,900,1300,320].forEach((v,i)=>bytes.writeUInt16LE(v,i*2));bytes[8]=4;bytes[9]=1;
  const model=new RadarHostModel();model.connection(true);
  const frame={v:1,profile:'f1_host',k:'host_stable',session:1,config:2,scan:3,map:4,plan:5,stable_id:3,page:0,pages:1,offset:0,total:1,crc32:checksum(bytes),data:bytes.toString('base64')};
  assert.equal(model.accept('@RADAR '+JSON.stringify(frame)),true);
  assert.deepEqual(model.snapshots.stable.points,[{angle_deg:180,distance_mm:900,x_mm:1300,y_mm:320,votes:4,flags:1}]);
});
test('formal metadata pages retain frozen geometry, counts and compact acquisition progress',()=>{
  const model=new RadarModel();model.connection(true);const base={v:1,k:'map',session:1,origin:2,scan:3,map:4,plan:5,pages:3};
  for(const frame of [{...base,page:0,g:4,mask:0,pose:[2280,314,180]}, {...base,page:1,geometry:[65809,680,-407,66623,149407997,20569846,1800],window:[1,15,15],algorithm:1}, {...base,page:2,offset:0,counts:Array(25).fill(0)}])model.accept('@RADAR '+JSON.stringify(frame));
  assert.deepEqual(model.snapshots.map.geometry,[65809,680,-407,66623,149407997,20569846,1800]);
  model.accept('@RADAR '+JSON.stringify({v:1,k:'progress',i:[1,2,3,4,5],live_active:false,mapping:true,window:[1,15,15],c:[15,15]}));
  assert.equal(model.status.live_active,false);assert.deepEqual(model.status.window,[1,15,15]);
});
test('explicit-profile layout2 uses 72-byte pages and decodes wide totals and frozen affine',()=>{
  const bytes=Buffer.alloc(158);bytes[0]=2;bytes.writeUInt32LE(100000,1);bytes.writeUInt32LE(99900,5);bytes.writeUInt32LE(50,9);bytes.writeUInt16LE(15,80);bytes[82]=15;bytes[83]=2;bytes[84]=4;bytes[85]=1;bytes.writeUInt32LE(10,96);bytes.writeUInt32LE(24,100);
  [65809,680,-407,66623,149407997,20569846].forEach((v,i)=>bytes.writeInt32LE(v,104+4*i));bytes.writeUInt16LE(1800,128);[150,550,1000,1400,1850,2250,150,550,1000,1400,1850,2250].forEach((v,i)=>bytes.writeInt16LE(v,130+2*i));bytes.writeUInt16LE(2280,154);bytes.writeUInt16LE(314,156);
  const model=new RadarHostModel();model.connection(true);
  for(let p=0;p<3;p++){const f={v:2,profile:'f1_host',k:'r',i:[1,2,3,4,5],p,n:3,t:158,d:bytes.subarray(p*72,(p+1)*72).toString('base64')};model.accept('@RADAR '+JSON.stringify({...f,c:checksum(Buffer.from(JSON.stringify(f)))}));}
  assert.equal(model.snapshots.map.raw_points,100000);assert.equal(model.snapshots.map.window_count,15);assert.deepEqual(model.snapshots.map.geometry.matrix_q16,[65809,680,-407,66623]);
});
