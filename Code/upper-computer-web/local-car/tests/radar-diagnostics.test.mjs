import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import {RadarModel,RadarExchange,parseRadar,radarStateText,radarReasonText} from '../radar-model.mjs';
import {moduleForWire} from '../protocol.mjs';
import {chineseTerminalLine} from '../terminal-format.mjs';

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
const receiveSource=app.slice(app.indexOf('function receive(line){'),app.indexOf('\nfunction connection('));
function receiver(raw=false){
  const model=new RadarModel();model.connection(true);
  const logs=[],elements=new Map(),context={
    radarModel:model,radarView:{receive:line=>model.accept(line)},savedRadarSnapshot:null,
    localStorage:{setItem(){}},parseRadar,radarStateText,radarReasonText,
    status:new Map(),bridge:{connected:true},routeLocked:false,
    $:id=>{if(!elements.has(id))elements.set(id,{textContent:''});return elements.get(id);},
    renderStatus(){},updateControls(){},appendLog:(direction,text)=>logs.push({direction,text}),
    chineseLogs:()=>!raw,radarExchange:{busy:false},paramsView:null,bootProgress:()=>'',bootStage:'',
    wheelView:null,mapView:null,qrView:null,prepView:null,moduleForWire
  };
  vm.createContext(context);vm.runInContext(receiveSource,context);
  return {receive:context.receive,model,logs,elements};
}

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
