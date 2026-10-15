import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {RadarModel,RadarExchange,parseRadar,radarStateText,radarReasonText} from '../radar-model.mjs';
import {moduleForWire} from '../protocol.mjs';
import {chineseTerminalLine} from '../terminal-format.mjs';
import {readOnlyCommand,scopedStopCommand} from '../bridge.mjs';
import {radarProbeText} from '../radar-format.mjs';
import {RadarLiveModel} from '../radar-live.mjs';

const identity={v:1,session:13,origin:0,scan:0,map:0,plan:0,page:0,pages:1};
const statusFrame={...identity,k:'status',state:'error',reason:'scan_timeout',elapsed:10000,applicable:false,points_valid:false};
const diagFrame={...identity,k:'diag',bytes:45000,packets:20,bad:9,overflow:123,alarm:0,unit:'mm'};
const wire=frame=>'@RADAR '+JSON.stringify(frame);

test('console prompts cannot hide radar status or prevent a requested page completing',async()=>{
  const model=new RadarModel();model.connection(true);
  assert.equal(model.accept('arm> '+wire(statusFrame)),true);
  assert.equal(model.status.reason,'scan_timeout');
  model.accept(wire(diagFrame));model.accept(wire(statusFrame));
  assert.equal(model.status.bad,9);
  let exchange;
  exchange=new RadarExchange({intervalMs:0,timeoutMs:30,send:async()=>{
    queueMicrotask(()=>exchange.receive('arm> arm> '+wire({...identity,k:'points',scan:1,map:1,offset:0,total:1,points:[[900,1234,88]]})));
    return true;
  }});
  const pages=await exchange.fetch('points',1);
  assert.deepEqual(pages[0].points,[[900,1234,88]]);
});

// Execute the production app's receive handler. DOM sinks are isolated; the
// real model, parser and frame dispatch run without opening a serial port.
const app=readFileSync(new URL('../app.mjs',import.meta.url),'utf8');
const stopSource=app.slice(app.indexOf('async function requestStop(){'),app.indexOf('\nfunction showAttempt('));

test('global stop cancels an active radar page read before waiting for the stop response',async()=>{
  const wires=[],elements=new Map();let finishStop;
  const exchange=new RadarExchange({timeoutMs:40,send:async wire=>{wires.push(wire);return true;}});
  const context={radarExchange:exchange,presetCancelled:false,paramsView:null,prepView:null,stackingPanel:null,chassisTestSession:null,chassisTestResult:null,stopRecovery:null,
    attempts:{supersede(){}},appendLog(){},communicationError(){},
    $:id=>{if(!elements.has(id))elements.set(id,{textContent:'',dataset:{}});return elements.get(id);},
    bridge:{stop:()=>new Promise(resolve=>{finishStop=resolve;})}};
  vm.createContext(context);vm.runInContext(stopSource,context);
  const reading=exchange.fetch('cloud',3),rejection=assert.rejects(reading,/停止操作已取消雷达读取/);await new Promise(setImmediate);
  const stopping=context.requestStop();await rejection;
  assert.equal(exchange.busy,false);assert.deepEqual(wires,['radar fetch cloud 3 0']);
  exchange.receive(wire({...identity,k:'cloud',scan:3,map:7,plan:8,pages:2,offset:0,total:2,points:[[900,1234,88]]}));await new Promise(setImmediate);
  assert.deepEqual(wires,['radar fetch cloud 3 0']);finishStop();assert.equal(await stopping,true);
  assert.equal(elements.get('attempt-wire').textContent,'chassis stop');
});
const receiveSource=app.slice(app.indexOf('function receiveRadarLiveState(frame){'),app.indexOf('\nfunction connection('));
function receiver(raw=false){
  const model=new RadarModel();model.connection(true);
  const live=new RadarLiveModel();live.connection(true);
  const logs=[],elements=new Map(),context={
    radarModel:model,radarDirectView:null,radarHostClient:null,radarView:{receive:line=>{
      const before=live.state;
      if(live.accept(line)){if(before!==live.state)context.receiveRadarLiveState(live.state);return true;}
      return model.accept(line);
    }},savedRadarSnapshot:null,
    localStorage:{setItem(){}},parseRadar,radarStateText,radarReasonText,radarProbeText,
    status:new Map(),bridge:{connected:true},routeLocked:false,
    $:id=>{if(!elements.has(id))elements.set(id,{textContent:''});return elements.get(id);},
    renderStatus(){},updateControls(){},appendLog:(direction,text)=>logs.push({direction,text}),
    chineseLogs:()=>!raw,radarExchange:{busy:false},paramsView:null,bootProgress:()=>'',bootStage:'',
    wheelView:null,mapView:null,qrView:null,prepView:null,stopRecovery:null,moduleForWire
  };
  vm.createContext(context);vm.runInContext(receiveSource,context);
  return {receive:context.receive,model,live,logs,elements,status:context.status};
}

test('live state updates the main summary while compact pages stay silent in both terminal modes',()=>{
  for(const raw of [false,true]){
    const ui=receiver(raw),state={v:1,k:'live_state',s:13,id:2,q:1,active:true,state:'streaming',reason:'complete',pose:[2170,230,180],age_ms:0,points:120};
    ui.receive('arm> '+wire(state));
    assert.match(ui.elements.get('map-recent-result').textContent,/实时点云.*实时更新.*圈 1.*120 点/);
    assert.equal(ui.status.get('map').live,true);assert.equal(ui.model.identity,null);
    ui.receive('arm> arm> '+wire({v:1,k:'live',s:13,id:2,q:2,o:0,n:120,d:Buffer.alloc(96).toString('base64')}));
    assert.equal(ui.live.preview.received,24);assert.equal(ui.logs.length,0);
    ui.receive(wire({...state,active:false,state:'stopped',reason:'operator_stop'}));
    assert.match(ui.elements.get('map-recent-result').textContent,/实时点云.*已停止.*人工停止/);assert.equal(ui.logs.length,0);
  }
});

test('live command replies translate their actual acceptance and stop scope',()=>{
  assert.match(chineseTerminalLine('OK radar live_start_accepted'),/实时采集已受理/);
  assert.match(chineseTerminalLine('OK radar live_stop_requested'),/已请求停止实时采集/);
  assert.match(chineseTerminalLine('ERR radar live_in_progress'),/实时采集中.*停止实时点云/);
});

test('live stop remains available during preparation, queued writes and parameter batches',()=>{
  const preparationSource=app.slice(app.indexOf('const preparationStop='),app.indexOf('\nasync function requestStop('));
  const controlSource=app.slice(app.indexOf('function canSend(wire){'),app.indexOf('\nfunction updateControls('));
  const context={readOnlyCommand,scopedStopCommand,routeLocked:false,bridge:{connected:true,disconnecting:false},
    applyingGrabPreset:false,parameterBusy:true,sending:true,stopped:()=>true,prepView:{busy:true,pending:true},chassisTestSession:null,stopRecovery:null};
  vm.createContext(context);vm.runInContext(preparationSource+'\n'+controlSource,context);
  assert.equal(context.canSend('radar live stop'),true);
  assert.equal(context.canSend('radar live start lower'),false);
});

test('diagnostics reach terminal history while downloaded cloud pages stay separate',()=>{
  const ui=receiver();ui.receive(wire(diagFrame));
  assert.equal(ui.logs.length,1);assert.equal(ui.logs[0].text,wire(diagFrame));
  ui.receive(wire({...identity,k:'cloud',scan:1,map:1,offset:0,total:1,points:[[900,1234,88]]}));
  assert.equal(ui.logs.length,1);
  assert.equal(ui.model.snapshots.cloud.points[0][1],1234);
});

test('prompt-prefixed radar status updates the page and raw mode retains its original wire text',()=>{
  const ui=receiver(true),line='arm> '+wire(statusFrame);ui.receive(line);
  assert.match(ui.elements.get('map-recent-result')?.textContent??'',/完整圈采集超时/);
  assert.equal(ui.logs[0].text,line);
  assert.equal(ui.model.status.elapsed,10000);
});

test('Chinese radar diagnostic summary exposes reception, corruption and overflow counters',()=>{
  const summary=chineseTerminalLine('arm> '+wire(diagFrame));
  assert.match(summary,/雷达诊断/);assert.match(summary,/45000/);
  assert.match(summary,/有效测量包\s*20/);assert.match(summary,/校验\/长度异常\s*9/);
  assert.match(summary,/溢出\s*123/);assert.match(summary,/单位\s*mm/);
});

test('split diagnostics retain every counter and display the actual alarm code',()=>{
  const ui=receiver(),rx={...diagFrame,pages:2};
  delete rx.alarm;delete rx.unit;
  const alarm={...identity,k:'diag',page:1,pages:2,alarm:189570,last_alarm:0x0100,unit:'unknown'};
  ui.receive(wire(rx));ui.receive('arm> '+wire(alarm));ui.receive(wire(statusFrame));
  assert.equal(ui.model.status.bytes,45000);assert.equal(ui.model.status.last_alarm,0x0100);
  assert.equal(ui.logs.length,2);
  const reception=chineseTerminalLine(wire(rx));
  assert.match(reception,/45000/);assert.doesNotMatch(reception,/—|单位/);
  const fault=chineseTerminalLine(wire(alarm));
  assert.match(fault,/189570/);assert.match(fault,/0x0100/);assert.match(fault,/电源故障/);
  assert.doesNotMatch(fault,/—|有效测量包/);
  const unknown=chineseTerminalLine(wire({...alarm,last_alarm:0xcece}));
  assert.match(unknown,/0xCECE/);assert.doesNotMatch(unknown,/电源故障/);
  const otherOrder=chineseTerminalLine(wire({...alarm,last_alarm:1}));
  assert.match(otherOrder,/0x0001/);assert.doesNotMatch(otherOrder,/电源故障/);
});

test('probe pages reach logs and combine by query identity without stale success',()=>{
  const ui=receiver();
  const frames=[{...identity,k:'probe',page:0,pages:3,id:1,state:'confirmed',reason:'sn_reply',tx:true},
    {...identity,k:'probe',page:1,pages:3,id:1,bytes:123,alarm:15,last_alarm:256},
    {...identity,k:'probe',page:2,pages:3,id:1,sn:'ABC123456',hex:'CECECECE0001'}];
  for(const f of frames)ui.receive('arm> '+wire(f));
  assert.equal(ui.logs.length,3);assert.equal(ui.model.probe.sn,'ABC123456');
  assert.equal(ui.model.probe.tx,true);assert.equal(ui.model.probe.alarm,15);
  assert.match(ui.elements.get('map-recent-result').textContent,/往返通信已确认/);
  assert.match(chineseTerminalLine(wire(frames[0])),/往返通信已确认/);
  assert.match(chineseTerminalLine(wire(frames[2])),/ABC123456/);
  assert.match(chineseTerminalLine(wire(frames[2])),/CE CE CE CE 00 01/);
  ui.receive(wire({...frames[0],id:2,state:'waiting',reason:'query_waiting',tx:true}));
  assert.equal(ui.model.probe.sn,undefined);assert.equal(ui.model.probe.alarm,undefined);
  ui.receive(wire(frames[2]));assert.equal(ui.model.probe.id,2);assert.equal(ui.model.probe.sn,undefined);
  ui.model.connection(false);assert.equal(ui.model.probe.historical,true);
  ui.model.connection(true);assert.equal(ui.model.probe.historical,true);
  ui.receive(wire({...statusFrame,session:14}));assert.equal(ui.model.probe.historical,true);
});

test('diagnostic query remains usable while stopped and cannot admit a motion command',()=>{
  assert.equal(readOnlyCommand('radar probe'),true);assert.equal(readOnlyCommand('radar probe status'),true);
  assert.equal(readOnlyCommand('radar probe LSTARH'),false);assert.equal(readOnlyCommand('radar scan'),false);
  const summary=chineseTerminalLine(wire({...identity,k:'probe',id:1,state:'unconfirmed',reason:'alarm_only',tx:true}));
  assert.match(summary,/MCU.*发送完成/);assert.match(summary,/报警/);assert.doesNotMatch(summary,/往返通信已确认/);
});
