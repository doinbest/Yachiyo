import test from 'node:test';
import assert from 'node:assert/strict';
import {footprintIssue} from '../route-model.mjs';
import {DEFAULT_GEOMETRY} from '../map-model.mjs';
const module=await import('../route-profile.mjs').catch(()=>({}));
const phase=(kind,target,extra={})=>({kind,phase:kind,direction:kind==='turn'?'ccw':'forward',segment:1,station:0,visit:0,target,speed_mm_s:200,...extra});
const make=(phases,points=[[2100,300,0,0]])=>({points,phases,speed_mm_s:200,turn_speed_deg_s:100});
test('current profile follows supplied C turn and move phases, with distance/angle completion',()=>{
  assert.equal(typeof module.RouteProfileRunner,'function');
  for(const integer of [false,true]){
    const r=new module.RouteProfileRunner(make([phase('move',[2100,1200,90]),phase('turn',[2100,1200,180]),phase('move',[1200,1200,180],{segment:2})]),{integer});
    r.advance(120);assert.equal(r.state,'done');assert.equal(r.stops.length,3);
    assert.ok(Math.abs(r.pose.x_mm-1200)<1e-6);assert.ok(Math.abs(r.pose.y_mm-1200)<1e-6);assert.ok(Math.abs(r.pose.yaw_rad-Math.PI)<1e-6);
    assert.ok(r.path.every(p=>footprintIssue(p)===null));assert.ok(r.path.every(p=>Object.values(p).every(Number.isFinite)));
  }
});
test('preview stops before a swept narrow turn and identifies first target, phase and obstacle',()=>{
  assert.equal(typeof module.RouteProfileRunner,'function');
  const r=new module.RouteProfileRunner(make([phase('turn',[350,1200,0])],[[350,1200,0,0]]));
  r.advance(30);assert.equal(r.state,'error');assert.equal(r.issue.segment,1);assert.equal(r.issue.phase,'turn');
  assert.equal(r.issue.reason,'forbidden_area');assert.match(r.issue.obstacle,/非行车区|障碍/);
  assert.equal(footprintIssue(r.pose),null);assert.ok(r.pose.yaw_rad>0);
  assert.match(module.profileResultText(r),/目标 1.*转向.*非行车区/);
});
test('preview body size can be smaller without changing the formal 300 mm check',()=>{
  const profile=make([phase('turn',[1200,350,115.8])],[[1200,350,0,0]]);
  const formal=new module.RouteProfileRunner(profile);formal.advance(3);
  assert.equal(formal.state,'error');assert.equal(formal.issue.obstacle,'非行车区 6');
  const preview=new module.RouteProfileRunner(profile,{bodySize:{length_mm:299,width_mm:299}});preview.advance(3);
  assert.equal(preview.state,'done');assert.equal(preview.issue,null);
});
test('QR approach leaves diagonally then follows a straight lane at 200 mm/s',()=>{
  assert.equal(typeof module.RouteProfileRunner,'function');
  const r=new module.RouteProfileRunner(make([phase('move',[2100,300,90],{direction:'diagonal'}),phase('move',[2100,1200,90])],[[2250,150,0,0]]));
  for(let i=0;i<400&&r.state==='running';i++){r.advance(.02);assert.ok(Math.hypot(r.body[0],r.body[1])<=200+1e-8);}
  assert.equal(r.state,'done');assert.equal(r.issue,null);assert.ok(r.path.every(p=>footprintIssue(p)===null));
  assert.equal(r.stops.length,2);
  assert.deepEqual([r.stops[0].pose.x_mm,r.stops[0].pose.y_mm],[2100,300]);
  assert.ok(r.path.filter(p=>p.y_mm>=300).every(p=>Math.abs(p.x_mm-2100)<1e-6));
});
test('grid reachability can coexist with a blocked footprint and mask obstacles are identified',()=>{
  assert.equal(typeof module.RouteProfileRunner,'function');
  const r=new module.RouteProfileRunner(make([phase('move',[2100,1500,90])]),{mask:1<<19});
  r.advance(30);assert.equal(r.state,'error');assert.match(r.issue.obstacle,/R4C5/);assert.equal(r.issue.phase,'move');
});

test('playback completes a central turn with a coarse-grid warning while the formal check reports it',()=>{
  const profile=make([phase('turn',[1200,1200,180],{segment:4})],[[1200,1200,0,0]]);
  const formal=new module.RouteProfileRunner(profile,{mask:1<<11});formal.advance(20);
  assert.equal(formal.state,'error');assert.equal(formal.issue.obstacle,'地图障碍 R3C2');
  assert.ok(formal.pose.yaw_rad>Math.PI/2&&formal.pose.yaw_rad<Math.PI);
  const playback=new module.RouteProfileRunner(profile,{mask:1<<11,gridPolicy:'warn'});playback.advance(20);
  assert.equal(playback.state,'done');assert.equal(playback.issue,null);
  assert.equal(playback.warnings.length,1);assert.equal(playback.warnings[0].segment,4);
  assert.equal(playback.warnings[0].obstacle,'地图障碍 R3C2');
  assert.ok(Math.abs(playback.pose.yaw_rad-Math.PI)<1e-8);
  assert.match(module.profileResultText(playback),/风险.*R3C2/);
});

test('coarse-grid playback warnings retain stops at known physical obstacles',()=>{
  const r=new module.RouteProfileRunner(make([phase('turn',[350,1200,0])],[[350,1200,0,0]]),{gridPolicy:'warn'});
  r.advance(20);assert.equal(r.state,'error');assert.equal(r.issue.obstacle,'非行车区 5');
  assert.equal(footprintIssue(r.pose),null);
});
test('integer endpoint settling preserves even the minimum 10 mm/s speed norm',()=>{
  const r=new module.RouteProfileRunner(make([phase('move',[2100,400,90],{speed_mm_s:10})]),{integer:true});
  for(let i=0;i<1000&&r.state==='running';i++){
    const distance=r.distance;r.advance(.02);assert.ok(r.distance-distance<=.2+1e-8,`step travelled ${r.distance-distance} mm`);
  }
  assert.equal(r.state,'done');
});
test('current model uses 300 mm/s² ramp and 3000 RPM scaling even with explicit 5000 mm/s input',()=>{
  const profile=make([phase('move',[2100,1200,90],{speed_mm_s:5000})]);
  const ramp=new module.RouteProfileRunner(profile);ramp.advance(.02);
  assert.ok(Math.abs(Math.hypot(ramp.body[0],ramp.body[1])-6)<1e-8);
  const capped=new module.RouteProfileRunner(profile,{geometry:{...DEFAULT_GEOMETRY,gear_ratio:50}});
  capped.advance(2);assert.ok(capped.rpm.every(value=>Math.abs(value)<=3000+1e-8));assert.ok(Math.max(...capped.rpm.map(Math.abs))>2999);
  const maxStep=3000*Math.PI*80/(60*50)*.02;
  for(let i=0;i<1000&&capped.state==='running';i++){const distance=capped.distance;capped.advance(.02);assert.ok(capped.distance-distance<=maxStep+1e-8);}
  assert.equal(capped.state,'done');
});

// Captured from the shared C CLI default profile on 2026-10-03; no JS heading selection.
const fixedCPhases=[
  ["move","move","diagonal",1,5,1,[2100,300,180],200],
  ["turn","turn","turn",1,5,1,[2100,300,90],0],
  ["move","move","forward",1,5,1,[2100,1200,90],200],
  ["move","move","forward",2,0,0,[2100,2100,90],200],
  ["turn","turn","turn",3,4,2,[2100,2100,180],0],
  ["move","move","forward",3,4,2,[1200,2100,180],200],
  ["turn","arrival_turn","turn",3,4,2,[1200,2100,90],0],
  ["move","move","reverse",4,2,3,[1200,350,90],200],
  ["move","move","forward",5,0,0,[1200,1200,90],200],
  ["turn","turn","turn",6,3,4,[1200,1200,180],0],
  ["move","dock_approach","forward",6,3,4,[425,1200,180],200],
  ["turn","arrival_turn","turn",6,3,4,[425,1200,90],0],
  ["move","dock","lateral",6,3,4,[350,1200,90],50],
  ["move","undock","lateral",7,0,0,[425,1200,90],50],
  ["turn","turn","turn",7,0,0,[425,1200,180],0],
  ["move","move","reverse",7,0,0,[1200,1200,180],200],
  ["turn","turn","turn",8,4,5,[1200,1200,90],0],
  ["move","move","forward",8,4,5,[1200,2100,90],200],
  ["move","move","reverse",9,2,6,[1200,350,90],200],
  ["move","move","forward",10,0,0,[1200,1200,90],200],
  ["turn","turn","turn",11,3,7,[1200,1200,180],0],
  ["move","dock_approach","forward",11,3,7,[425,1200,180],200],
  ["turn","arrival_turn","turn",11,3,7,[425,1200,90],0],
  ["move","dock","lateral",11,3,7,[350,1200,90],50],
  ["move","undock","lateral",12,0,0,[425,1200,90],50],
  ["turn","turn","turn",12,0,0,[425,1200,180],0],
  ["move","move","reverse",12,0,0,[1200,1200,180],200],
  ["move","move","reverse",13,0,0,[2100,1200,180],200],
  ["turn","arrival_turn","turn",13,0,0,[2100,1200,90],0],
  ["move","move","reverse",14,0,0,[2100,300,90],200],
  ["move","move","diagonal",15,1,8,[2250,150,90],200]
].map(([kind,phase,direction,segment,station,visit,target,speed_mm_s])=>({kind,phase,direction,segment,station,visit,target,speed_mm_s}));
test("default fixed15 C profile clears the complete swept footprint in both playback models",()=>{
  for(const integer of [false,true]){
    const r=new module.RouteProfileRunner({points:[[2250,150,1,0]],initial_yaw_deg:180,speed_mm_s:200,turn_speed_deg_s:100,phases:fixedCPhases},{integer});
    r.advance(1000);assert.equal(r.state,"done");assert.equal(r.issue,null);assert.equal(r.rows.at(-1).segment,15);
    assert.deepEqual([r.pose.x_mm,r.pose.y_mm],[2250,150]);assert.ok(r.path.every(p=>footprintIssue(p)===null));
  }
});


test('preview consumes the turn speed supplied by C instead of the old 0.15 rad/s ceiling',()=>{
  for(const limit of [60,100]) {
    const profile=make([phase('turn',[1200,1200,-90])],[[1200,1200,0,0]]);
    profile.turn_speed_deg_s=limit;
    const r=new module.RouteProfileRunner(profile);let peak=0;
    for(let i=0;i<300 && r.state==='running';i++){r.advance(.02);peak=Math.max(peak,Math.abs(r.body[2]));}
    assert.equal(r.state,'done');assert.ok(peak>1);assert.ok(peak<=limit*Math.PI/180+1e-8);
  }
});

test("C preview preserves the left-facing initial heading before lane entry",()=>{
  const profile=make([phase("move",[2100,300,180]),phase("turn",[2100,300,90]),phase("move",[2100,1200,90])],[[2250,150,1,0]]);
  profile.initial_yaw_deg=180;
  const r=new module.RouteProfileRunner(profile);
  assert.equal(r.pose.yaw_rad,Math.PI);r.advance(.1);assert.equal(r.pose.yaw_rad,Math.PI);
  r.advance(100);assert.equal(r.state,"done");assert.equal(r.issue,null);
});
