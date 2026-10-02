import test from 'node:test';
import assert from 'node:assert/strict';
import {RouteRunner,validateRoute,defaultRoute,bodyCorners,turntableGap,footprintIssue,routeResultText,originalStartZone} from '../route-model.mjs';
const segment=(extra={})=>({name:'扫码停车',vx:100,vy:0,omega:0,accel:0.5,hold:10,decel:0.5,dwell:1,...extra});
const start={x_mm:2250,y_mm:150,yaw_rad:Math.PI/2};
const near=(a,b,t=.01)=>assert.ok(Math.abs(a-b)<t,`${a} != ${b}`);
test('each segment ramps, stops, dwells then advances, without endpoint snapping',()=>{
  const r=new RouteRunner([segment(),segment({name:'下一站'})],start);
  r.advance(.25);near(r.body[0],50);assert.equal(r.phase,'accel');
  r.advance(10.75);near(r.pose.y_mm,1200);assert.equal(r.stops.length,1);
  assert.deepEqual(r.body,[0,0,0]);assert.equal(r.phase,'dwell');
  r.advance(1);near(r.pose.y_mm,1200);assert.equal(r.index,1);
  r.advance(12);assert.equal(r.state,'done');near(r.pose.y_mm,2250);
  assert.equal(r.stops.length,2);const end={...r.pose};r.advance(100);assert.deepEqual(r.pose,end);
});
test('integer model integrates quantized ramp at each step and keeps exact finite duration',()=>{
  const ideal=new RouteRunner([segment()],start);
  const integer=new RouteRunner([segment()],start,{integer:true});
  ideal.advance(12);integer.advance(12);
  assert.equal(integer.state,'done');near(integer.elapsed,12);
  assert.ok(Math.abs(integer.pose.y_mm-ideal.pose.y_mm)>.1);
  assert.ok(Math.abs(integer.pose.y_mm-ideal.pose.y_mm)<10);
});
test('snapshot isolation, partial milliseconds and stationary dwell',()=>{
  const route=[segment({hold:.013,accel:.027,decel:.031,dwell:.009})];
  const r=new RouteRunner(route,start);route[0].vx=999;
  r.advance(1);assert.equal(r.state,'done');near(r.elapsed,.08,1e-8);
  assert.ok(r.pose.y_mm<160);
});
test('whole 300 mm body stops before its outer edge leaves the field',()=>{
  const r=new RouteRunner([segment({hold:30})],start);r.advance(40);
  assert.equal(r.state,'error');assert.equal(r.reason,'boundary');
  assert.ok(r.pose.y_mm<=2250);assert.ok(r.pose.y_mm>2247);
  assert.deepEqual(r.body,[0,0,0]);assert.equal(r.stops.length,0);
});
test('turning changes subsequent body direction and keeps stop headings',()=>{
  const r=new RouteRunner([segment({vx:0,omega:.1,hold:2,dwell:0}),segment({hold:1,dwell:0})],{...start,x_mm:2100,y_mm:300});
  r.advance(5);assert.equal(r.state,'done');near(r.stops[0].pose.yaw_rad,Math.PI/2+.25,.0001);
  assert.ok(r.pose.x_mm<2100);assert.ok(r.pose.y_mm>300);
});
test('invalid geometry/rows/durations/initial pose are rejected before running',()=>{
  assert.throws(()=>new RouteRunner([segment()],{...start,y_mm:2500}));
  for(const row of [segment({vx:NaN}),segment({hold:-1}),segment({accel:0}),segment({decel:0}),segment({hold:1000})])assert.throws(()=>validateRoute([row]));
  assert.throws(()=>validateRoute([]));
});


test('default diagonal, scan, early-left and turntable stops have real body clearance',()=>{
  const expected=[[2100,300],[2100,1200],[2100,2100],[1200,2100]];
  for(const integer of [false,true]){
    const r=new RouteRunner(defaultRoute().slice(0,4),start,{integer});r.advance(100);
    assert.equal(r.state,'done');assert.equal(r.stops.length,4);
    r.stops.forEach((stop,i)=>{near(stop.pose.x_mm,expected[i][0],integer?15:.01);near(stop.pose.y_mm,expected[i][1],integer?15:.01);near(stop.pose.yaw_rad,Math.PI/2);});
    assert.ok(r.path.every(p=>footprintIssue(p)===null));
    assert.ok(turntableGap(r.pose)>50);
    const scanRight=Math.max(...bodyCorners(r.stops[1].pose).map(c=>c[0]));assert.ok(2400-scanRight>=140);
    if(!integer){near(turntableGap(r.pose),70);near(2400-scanRight,150);}
  }
});
test('turntable and forbidden area stop the envelope before overlap',()=>{
  const disk=new RouteRunner([segment({hold:3,dwell:0})],{x_mm:1200,y_mm:2100,yaw_rad:Math.PI/2});disk.advance(10);
  assert.equal(disk.state,'error');assert.equal(disk.reason,'turntable');assert.ok(turntableGap(disk.pose)>0);
  const block=new RouteRunner([segment({vx:0,vy:100,hold:3,dwell:0})],{x_mm:2100,y_mm:1600,yaw_rad:Math.PI/2});block.advance(10);
  assert.equal(block.reason,'forbidden_area');assert.equal(footprintIssue(block.pose),null);
});
test('rotation needs space for the corners, not just the wheel-center rectangle',()=>{
  assert.equal(footprintIssue(start),null);
  assert.equal(footprintIssue({...start,yaw_rad:Math.PI/4}),'boundary');
  const corners=bodyCorners({x_mm:1200,y_mm:1200,yaw_rad:Math.PI/4});
  near(Math.max(...corners.map(c=>c[0]))-Math.min(...corners.map(c=>c[0])),300*Math.SQRT2);
});

test('two batches visit every station in order, reload after processing and return in 170 seconds',()=>{
  const rows=defaultRoute();assert.equal(rows.length,16);
  for(const i of [4,9])assert.match(rows[i].name,/放齐3件再按序装仓/);
  assert.match(rows[6].name,/第一批.*平放/);assert.match(rows[11].name,/第二批.*同色码垛/);
  const expected=[[2100,300],[2100,1200],[2100,2100],[1200,2100],[1200,350],[1200,1200],[350,1200],[1200,1200],[1200,2100],[1200,350],[1200,1200],[350,1200],[1200,1200],[2100,1200],[2100,300],[2250,150]];
  const r=new RouteRunner(rows,start);assert.equal(r.returnedToStart,false);r.advance(169.5);
  assert.equal(r.state,'running');assert.equal(r.returnedToStart,false);r.advance(.5);
  assert.equal(r.state,'done');near(r.elapsed,170);assert.equal(r.stops.length,16);
  r.stops.forEach((s,i)=>{near(s.pose.x_mm,expected[i][0]);near(s.pose.y_mm,expected[i][1]);near(s.pose.yaw_rad,Math.PI/2);});
  assert.ok(r.path.every(p=>footprintIssue(p)===null));
  for(const i of [3,8])near(turntableGap(r.stops[i].pose),70);
  for(const i of [4,9])near(Math.min(...bodyCorners(r.stops[i].pose).map(c=>c[1]))-150,50);
  for(const i of [6,11])near(Math.min(...bodyCorners(r.stops[i].pose).map(c=>c[0]))-150,50);
  assert.equal(r.returnedToStart,true);assert.match(routeResultText(r),/外框已进入原启停区/);
});

test('integer route preserves accumulated error and stops on the final return boundary',()=>{
  const r=new RouteRunner(defaultRoute(),start,{integer:true});r.advance(200);
  assert.equal(r.state,'error');assert.equal(r.reason,'boundary');assert.equal(r.index,15);
  assert.equal(r.stops.length,15);assert.equal(r.returnedToStart,false);
  assert.ok(r.path.every(p=>footprintIssue(p)===null));assert.notDeepEqual(r.pose,start);
  assert.match(routeResultText(r),/第 16 段.*返回阶段需末端精定位/);
  assert.doesNotMatch(routeResultText(r),/外框已进入/);
});

test('completion alone or a single-row run cannot qualify a full return',()=>{
  const r=new RouteRunner(defaultRoute().slice(0,4),start);r.advance(100);
  assert.equal(r.state,'done');assert.equal(r.returnedToStart,false);
  assert.match(routeResultText(r),/外框尚未进入/);
  assert.match(routeResultText(r,{wholeRoute:false}),/本段模拟结束/);
  assert.deepEqual(originalStartZone(start),{x:2100,y:0});
  assert.deepEqual(originalStartZone({...start,y_mm:2250}),{x:2100,y:2100});
  assert.equal(originalStartZone({...start,x_mm:2249}),null);
  assert.equal(originalStartZone({...start,yaw_rad:Math.PI/4}),null);
});
