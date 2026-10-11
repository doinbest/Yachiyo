import test from 'node:test';
import assert from 'node:assert/strict';
import {installationParameters,projectFrozenRadarPoint} from '../radar-installation.mjs';
import {importSnapshot} from '../radar-model.mjs';
import {backupInstallation} from '../monitor-radar.mjs';

test('monitor converts reference case1 once and retains the measured current case2 installation',()=>{
  const p=installationParameters('case1','current');
  assert.deepEqual([p.lidar_x_mm,p.lidar_y_mm,p.zero_deg],[2170,230,180]);
  assert.deepEqual([p.tx_q16,p.ty_q16,p.zero_tenths],[142213120,15073280,1800]);
  assert.deepEqual(installationParameters('case2','current'),installationParameters('case2'));
});

test('reference import rotates affine, stable XY and route together without changing raw polar evidence',()=>{
  const source={coordinate_frame:'reference',mask:1,counts:Array(25).fill(0),pose:[100,200,0],
    points:[[300,200,5,0]],geometry:[131072,0,0,65536,6553600,13107200,0],
    params:{lidar_x_mm:100,lidar_y_mm:200,zero_deg:0,m00_q16:131072,m01_q16:0,m10_q16:0,m11_q16:65536,tx_q16:6553600,ty_q16:13107200,zero_tenths:0},
    stable:{points:[[0,100,300,200,4,1],{angle_deg:0,distance_mm:100,x_mm:300,y_mm:200,votes:4}]},
    cloud:{pose:[100,200,0],points:[[0,100,40]],complete:true}};
  const before=structuredClone(source),result=importSnapshot(source);
  assert.deepEqual(result.points,[[2200,300,5,0]]);
  assert.deepEqual(result.geometry,[65536,0,0,131072,144179200,6553600,900]);
  assert.deepEqual(result.stable.points[0],[0,100,2200,300,4,1]);
  assert.deepEqual([result.stable.points[1].x_mm,result.stable.points[1].y_mm],[2200,300]);
  assert.deepEqual(result.cloud.points,[[0,100,40]]);
  assert.deepEqual(result.cloud.pose,[2200,100,90]);
  assert.deepEqual(projectFrozenRadarPoint(result.cloud.points[0],result),[2200,300]);
  assert.equal(result.params.m00_q16,65536);assert.equal(result.params.m11_q16,131072);
  assert.equal(result.params.tx_q16,144179200);assert.equal(result.params.zero_tenths,900);
  assert.deepEqual(source,before);
  const again=importSnapshot(result);
  assert.deepEqual(again.geometry,result.geometry);assert.deepEqual(again.stable,result.stable);
});

test('a wrapper reference frame also rotates its coherent frozen snapshots',()=>{
  const id={session:1,origin:2,scan:3,map:4,plan:5};
  const imported=importSnapshot({coordinate_frame:'reference',snapshots:{map:{...id,mask:1,counts:Array(25).fill(0),pose:[230,230,90]},path:{...id,points:[[300,230,5,0]]},stable:{...id,points:[[0,100,230,330,4,1]]}}});
  assert.deepEqual(imported.pose,[2170,230,180]);
  assert.deepEqual(imported.stable.points[0],[0,100,2070,230,4,1]);
});

test('copying a calibrated reference installation preserves affine precision and raw filters',()=>{
  const config={coordinate_frame:'reference',installation:{lidar_x_mm:100,lidar_y_mm:200,zero_deg:0,zero_tenths:0,m00_q16:131072,m01_q16:0,m10_q16:0,m11_q16:65536,tx_q16:6553600,ty_q16:13107200},map:{radarXmm:100,radarYmm:200,obstaclePointThreshold:2},filter:{minAngleDeg:15,maxAngleDeg:250,minDistanceMm:100,maxDistanceMm:3000,minEnergy:2,maxEnergy:90}};
  const copied=backupInstallation(config);
  assert.equal(copied.zero_deg,90);assert.equal(copied.m00_q16,65536);assert.equal(copied.m11_q16,131072);
  assert.equal(copied.tx_q16,144179200);assert.equal(copied.angle_min_tenths,150);
  const current=backupInstallation({...config,coordinate_frame:'current'});
  assert.equal(current.zero_deg,0);assert.equal(current.tx_q16,6553600);
});
