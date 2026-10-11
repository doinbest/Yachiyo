import test from 'node:test';
import assert from 'node:assert/strict';
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import * as protocol from '../protocol.mjs';
import {BridgeLink,readOnlyCommand,scopedStopCommand} from '../bridge.mjs';

// Execute the real app handlers against a small DOM and an in-memory transport.
// No serial service, browser hooks, or physical command endpoint is involved.
const source=readFileSync(new URL('../app.mjs',import.meta.url),'utf8');
const section=(start,end)=>{
  const first=source.indexOf(start),last=source.indexOf(end,first+start.length);
  assert.ok(first>=0&&last>first,`app handler ${start}`);return source.slice(first,last);
};
function consoleFixture(){
  const elements=[],logs=[],wires=[],messages=[];
  const node=(tag,text,cls)=>{
    const e={tag,textContent:text??'',className:cls??'',children:[],options:[],dataset:{},attributes:{},value:'',disabled:false,isConnected:true,
      classList:{toggle(){},add(){}},setAttribute(k,v){this.attributes[k]=String(v);},
      append(...items){this.children.push(...items);if(this.tag==='select')this.options.push(...items);},
      replaceChildren(...items){this.children=[...items];},querySelector(selector){return this.children.find(e=>e.className===selector.slice(1))??null;},
      get lastElementChild(){return this.children.at(-1);}};
    elements.push(e);return e;
  };
  const $=id=>elements.find(e=>e.id===id)||Object.assign(node(id==='serial-port'?'select':'div'),{id});
  const bridge={connected:true,disconnecting:false,available:true,stopLatched:false,port:'COM90',
    async send(wire){wires.push(wire);return {text:wire};}};
  const empty=()=>{};
  const ctx=vm.createContext({...protocol,node,$,bridge,readOnlyCommand,scopedStopCommand,
    document:{getElementById:id=>elements.find(e=>e.id===id)||null,querySelectorAll:()=>elements.filter(e=>e.dataset.send),createElement:node},
    window:{confirm:()=>false},toast:t=>messages.push(t),appendLog:(d,t)=>logs.push({d,t}),
    renderStatus:empty,displayReply:protocol.formatTerminalLine,visualStateText:t=>t??'—',bootProgress:(_l,s)=>s,
    parseRadar:()=>null,radarModel:{snapshots:{},params:{}},localStorage:{setItem:empty},
    radarExchange:{busy:false,cancel:empty},radarHostClient:null,radarDirectView:null,
    paramsView:{receive:empty,cancel:empty,update:empty},trackingPanel:null,stackingPanel:null,
    radarView:{receive:empty,connection:empty,render:empty},mapView:null,wheelView:null,
    qrView:{receive:empty,connection:empty,refreshControls:empty},
    prepView:{busy:false,pending:null,connection(){this.busy=false;this.pending=null;},cancel(){this.busy=false;this.pending=null;},receive:empty},
    attempts:{run:(_w,work)=>work()},requestStop:empty,copyText:empty,
    setTimeout,clearTimeout,console});
  vm.runInContext(`
    let routeLocked=false,sending=0,applyingGrabPreset=false,parameterBusy=false,presetCancelled=false;
    let connectionState='connected',lastStopEvent=null,trackingConnected=false,axis='base',active='vision',bootStage='';
    let grabSnapshot=null,grabRouteSnapshot=null,grabReply=null,savedRadarSnapshot=null,grabCalibrationSnapshot=null;
    let grabPanel=null;
    const status=new Map(),forms=[];
    const preparationStop=wire=>wire==='bus recover'||/^(?:chassis (?:stop|route cancel)|stop (?:all|base|z|x)|radar live stop|grab stop)$/.test(wire);
    const stopped=()=>bridge.stopLatched;
    ${section('function mayQuery(wire)','async function requestStop()')}
    ${section('async function send(wire','async function refreshPorts()')}
    ${section('function definition(id)','async function copyText(text)')}
    ${section('function receive(line)','function communicationError(error)')}
    ${section('function renderGrabCalibrationStatus()','function buildVisionWorkspace(')}
    ${section('function renderGrabStatus()','function buildPages()')}
    globalThis.sim={canSend,send,receive,connection,controlForm,updateControls,buildGrabCalibrationPanel,
      get grabSnapshot(){return grabSnapshot;},get calibrationSnapshot(){return grabCalibrationSnapshot;},
      mountGrabStatus(){grabPanel=$('grab-task-status');renderGrabStatus();},
      configure(p){
        if('connected'in p)bridge.connected=p.connected;if('disconnecting'in p)bridge.disconnecting=p.disconnecting;
        if('stopLatched'in p)bridge.stopLatched=p.stopLatched;if('sending'in p)sending=p.sending;
        if('parameterBusy'in p)parameterBusy=p.parameterBusy;if('routeLocked'in p)routeLocked=p.routeLocked;
        if('axis'in p)axis=p.axis;
        if('preparing'in p)prepView.busy=p.preparing;if('pending'in p)prepView.pending=p.pending;
        updateControls();
      },get sending(){return sending;},get routeLocked(){return routeLocked;}};
  `,ctx);
  return {sim:ctx.sim,bridge,elements,wires,messages,logs,$};
}

test('relative motion form updates Z bounds and blocks oversized moves after switching axes', async()=>{
  const f=consoleFixture();f.sim.controlForm('pos',{sharedAxis:true});
  const input=f.$('pos-degree'),form=f.elements.find(e=>e.tag==='form');
  f.sim.configure({axis:'z'});
  assert.equal(Number(input.min),-3600);assert.equal(Number(input.max),3600);
  input.value='-3600';input.oninput();
  assert.deepEqual(f.wires,[]);
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['pos z -3600']);
  f.sim.configure({axis:'x'});
  assert.equal(Number(input.min),-360);assert.equal(Number(input.max),360);
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['pos z -3600']);
  f.sim.configure({axis:'z',stopLatched:true});
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['pos z -3600']);
});

for(const state of ['idle','prepare','acquire','align','vision_grace','vision_pause','hold','error']){
  test(`device ${state} reply cannot permanently disable manual recovery or retry`,async()=>{
    const f=consoleFixture();
    f.sim.receive(`${state==='error'?'ERR':'OK'} grab state=${state} mode=align operation=ring_align reason=${state==='error'?'wheel_disabled_or_stall':'running'} missing=none stop_confirmed=1`);
    for(const wire of ['grab stop','grab status','camera status','bus status','bus recover','grab ring align 1 -100 stay','grip open'])
      assert.equal(f.sim.canSend(wire),true,wire);
    await f.sim.send('grab status');assert.deepEqual(f.wires,['grab status']);
  });
}
for(const state of [{connected:false},{disconnecting:true}]){
  test(`disconnected/closing blocks serial sends without issuing a command ${JSON.stringify(state)}`,()=>{
    const f=consoleFixture();f.sim.configure(state);
    for(const wire of ['grab status','grab stop','grab ring align 1 -100'])assert.equal(f.sim.canSend(wire),false);
    assert.equal(f.$('service-stop').disabled,false);assert.deepEqual(f.wires,[]);
  });
}
test('stop latch keeps queries and scoped recovery available while motion stays blocked',()=>{
  const f=consoleFixture();f.sim.configure({stopLatched:true});
  for(const wire of ['grab status','grab status x','camera status','chassis route status','bus recover','grab stop','turntable stop'])
    assert.equal(f.sim.canSend(wire),true,wire);
  for(const wire of ['grab ring align 3 -60 stay','grip open','camera ring 3','turntable home'])assert.equal(f.sim.canSend(wire),false,wire);
  f.sim.configure({stopLatched:false});assert.equal(f.sim.canSend('grab ring align 3 -60 stay'),true);
  assert.deepEqual(f.wires,[]); // Unlocking never resumes old work.
});
test('mission and first-item status remain readable under the stop latch',()=>{
  const f=consoleFixture();f.sim.configure({stopLatched:true});
  for(const wire of ['grab mission status','grab route status'])assert.equal(f.sim.canSend(wire),true,wire);
});
test('a stale running route is released when the connection changes',()=>{
  const f=consoleFixture();f.sim.receive('OK chassis route state=running segment=1');
  assert.equal(f.sim.canSend('chassis route start 200'),false);
  f.bridge.connected=false;f.sim.connection('disconnected');
  f.bridge.connected=true;f.sim.connection('connected');
  assert.equal(f.sim.canSend('chassis route start 200'),true);
  assert.deepEqual(f.wires,[]);
});
test('route completion with a console prompt releases the start button',()=>{
  const f=consoleFixture();f.sim.receive('OK chassis route state=running segment=1');
  f.sim.receive('arm> OK chassis route state=done segment=15');
  assert.equal(f.sim.canSend('chassis route start 200'),true);
});
for(const terminal of ['idle','done','error','cancelled']){
  test(`route ${terminal} reply releases only the old route start lock`,()=>{
    const f=consoleFixture();f.sim.receive('OK chassis route state=running segment=1');
    f.sim.receive(`OK chassis route state=${terminal} segment=1`);
    assert.equal(f.sim.canSend('chassis route start 200'),true);
    assert.equal(f.sim.canSend('grab stop'),true);
  });
}
test('parameter and preparation waits retain stop paths and release cleanly',()=>{
  const f=consoleFixture();
  for(const state of [{parameterBusy:true},{preparing:true,pending:{wire:'imu status'}},{sending:1}]){
    f.sim.configure({parameterBusy:false,preparing:false,pending:null,sending:0,...state});
    assert.equal(f.sim.canSend('grab ring align 1 -100'),false);
    for(const wire of ['grab stop','stop all','imu cal cancel'])assert.equal(f.sim.canSend(wire),true,wire);
  }
  f.sim.configure({parameterBusy:false,preparing:false,pending:null,sending:0});
  assert.equal(f.sim.canSend('grab ring align 1 -100'),true);
});
test('a rejected send releases the real sending gate and keeps editable height',async()=>{
  const f=consoleFixture();f.bridge.send=async()=>{throw new Error('模拟写出失败');};
  assert.equal(await f.sim.send('grab ring align 3 -60 stay'),false);
  assert.equal(f.sim.sending,0);assert.equal(f.sim.canSend('grab status'),true);
  assert.equal(f.sim.canSend('grab ring align 3 -60 stay'),true);
});
test('forms still being constructed remain registered for the later connection update',()=>{
  const f=consoleFixture();f.bridge.connected=false;
  const form=f.sim.controlForm('grab-ring-align',{compact:true,alternateCommand:{id:'grab-ring-align-stay',label:'保持低位'}});
  const buttons=f.elements.filter(e=>e.type==='submit'||e.textContent==='保持低位');
  assert.ok(buttons.every(e=>e.disabled));
  form.isConnected=false;f.sim.updateControls(); // Nested tabs render before attaching their body.
  form.isConnected=true;f.bridge.connected=true;f.sim.connection('connected');
  assert.ok(buttons.every(e=>!e.disabled));
  f.sim.configure({stopLatched:true});assert.ok(buttons.every(e=>e.disabled));
  f.sim.configure({stopLatched:false});assert.ok(buttons.every(e=>!e.disabled));
  assert.deepEqual(f.wires,[]);
});
for(const phase of ['headers','body'])test(`a stalled HTTP ${phase} response releases actual app buttons without replay`,async()=>{
  const f=consoleFixture();let calls=0;
  const link=new BridgeLink({requestTimeoutMs:20,fetch:async()=>{
    calls++;const never=new Promise(()=>{});
    return phase==='headers'?never:{ok:true,json:()=>never};
  }});
  link.connected=true;f.bridge.send=link.send.bind(link);
  const sending=f.sim.send('grab ring align 3 -100 stay');
  assert.equal(f.sim.sending,1);assert.equal(f.sim.canSend('grab stop'),true);
  let timer;
  try {
    const result=await Promise.race([sending,new Promise(resolve=>{timer=setTimeout(()=>resolve('hung'),500);})]);
    assert.equal(result,false);assert.equal(f.sim.sending,0);
    assert.equal(f.sim.canSend('grab status'),true);assert.equal(calls,1);
    assert.ok(f.logs.some(e=>e.t.includes('超时')));
  } finally {clearTimeout(timer);}
});
for(const reason of ['imu_invalid','feedback_stale','stop_unconfirmed','wheel_disabled_or_stall','ring_alignment_required','missing_params','vision_timeout']){
  test(`all command forms recover after ${reason}, stop, busy and reconnect without losing inputs`,()=>{
    const f=consoleFixture();
    for(const c of protocol.commands)f.sim.controlForm(c.id);
    const submits=f.elements.filter(e=>e.type==='submit'),inputs=f.elements.filter(e=>e.tag==='input'||e.tag==='select');
    const initial=submits.map(e=>e.disabled),values=inputs.map(e=>e.value);
    f.sim.receive(`ERR grab state=error mode=align reason=${reason} missing=ref_u,j00 stop_confirmed=1`);
    assert.deepEqual(submits.map(e=>e.disabled),initial);
    f.sim.configure({stopLatched:true});
    f.sim.configure({stopLatched:false,parameterBusy:true});
    f.sim.configure({parameterBusy:false,preparing:true,pending:{wire:'imu status'}});
    f.sim.configure({preparing:false,pending:null,sending:1});
    f.sim.configure({sending:0});
    f.bridge.connected=false;f.sim.connection('disconnected');
    assert.ok(submits.every(e=>e.disabled));
    f.bridge.connected=true;f.sim.connection('connected');
    assert.deepEqual(submits.map(e=>e.disabled),initial);
    assert.deepEqual(inputs.map(e=>e.value),values);assert.deepEqual(f.wires,[]);
  });
}
test('ring buttons share edited values and recover after invalid input and transient send',async()=>{
  const f=consoleFixture();f.sim.controlForm('grab-ring-align',{compact:true,alternateCommand:{id:'grab-ring-align-stay',label:'对准后保持低位'}});
  const field=id=>f.elements.find(e=>e.id===id),buttons=f.elements.filter(e=>e.tag==='button');
  const stay=buttons.find(e=>e.textContent==='对准后保持低位'),raise=buttons.find(e=>e.type==='submit');
  field('grab-ring-align-ring').value='3';field('grab-ring-align-z').value='-60';field('grab-ring-align-z').oninput();
  assert.equal(stay.title,'grab align ring 3 -60 stay');assert.equal(raise.title,'grab align ring 3 -60');
  field('grab-ring-align-z').value='-151';field('grab-ring-align-z').oninput();assert.equal(stay.disabled,true);assert.equal(raise.disabled,true);
  field('grab-ring-align-z').value='-100';field('grab-ring-align-z').oninput();assert.equal(stay.disabled,false);
  f.sim.configure({sending:1});assert.equal(stay.disabled,true);f.sim.configure({sending:0});assert.equal(stay.disabled,false);
  await stay.onclick();await new Promise(setImmediate);assert.deepEqual(f.wires,['grab align ring 3 -100 stay']);
});

test('real calibration controls send only explicit operations and preserve the task snapshot',async()=>{
  const f=consoleFixture();
  f.sim.buildGrabCalibrationPanel(f.$('system-body'));
  assert.deepEqual(f.wires,[]);
  const fields=f.elements.filter(e=>e.tag==='select');
  const target=fields.find(e=>e.options.some(o=>o.value==='3'));assert.ok(target);
  target.value='3';target.onchange?.();
  const buttons=f.elements.filter(e=>e.tag==='button');
  const start=buttons.find(e=>e.title==='grab calib 3');assert.ok(start);
  const forms=f.elements.filter(e=>e.tag==='form');
  forms[1].onsubmit({preventDefault(){}});await new Promise(setImmediate);
  assert.deepEqual(f.wires,['grab calib 3']);
  f.sim.receive('OK grab state=align mode=align operation=color_align reason=running missing=none');
  const original=f.sim.grabSnapshot;
  f.sim.receive('arm> OK grab calib phase=collect ref=1 calibrated=1 reason=none fdx=2 fdy=0.5 xdx=0 xdy=3');
  assert.equal(f.sim.grabSnapshot,original);
  assert.match(f.$('grab-calibration-status').textContent,/上次成功响应/);
  assert.match(f.$('grab-calibration-status').textContent,/2 \/ 0.5 px\/mm/);
  f.sim.configure({stopLatched:true});
  assert.equal(start.disabled,true);
  const query=buttons.find(e=>e.title==='grab calib status');assert.equal(query.disabled,false);
  query.onclick();await new Promise(setImmediate);assert.deepEqual(f.wires,['grab calib 3','grab calib status']);
  const stop=buttons.find(e=>e.title==='grab stop');assert.equal(stop.disabled,false);
  stop.onclick();await new Promise(setImmediate);assert.deepEqual(f.wires,['grab calib 3','grab calib status','grab stop']);
  f.bridge.connected=false;f.sim.connection('disconnected');
  f.bridge.connected=true;f.sim.connection('connected');
  assert.match(f.$('grab-calibration-status').textContent,/历史读回/);
  assert.equal(f.wires.length,3);
  f.sim.receive('OK grab calib phase=complete ref=1 calibrated=1 reason=calibrated fdx=2 fdy=0.5 xdx=0 xdy=3');
  assert.equal(f.sim.calibrationSnapshot.live,true);
  f.sim.receive('STM32 mechanical arm console ready');
  assert.equal(f.sim.calibrationSnapshot.live,false);
  assert.match(f.$('grab-calibration-status').textContent,/历史读回/);
  assert.equal(f.wires.length,3);
});

test('real calibration completion uses the firmware complete state',()=>{
  const f=consoleFixture();f.sim.mountGrabStatus();
  f.sim.receive('OK grab state=complete mode=align operation=calibration result=calibrated reason=calibrated missing=none');
  assert.match(f.$('grab-task-status').children.find(e=>e.tag==='h3').textContent,/标定完成/);
  assert.deepEqual(f.wires,[]);
});

test('pose rejection guides an explicit query and the task card displays the returned cause and missing parameters',async()=>{
  const f=consoleFixture();f.sim.mountGrabStatus();
  f.sim.receive('ERR grab pose rejected; inspect grab status');
  const text=e=>[e.textContent,...e.children.map(text)].join(' ');
  assert.match(text(f.$('grab-task-status')),/抓取任务状态.*视觉任务.*点击“读取抓取任务状态”/);
  assert.deepEqual(f.wires,[]);
  await f.sim.send('grab status');
  assert.deepEqual(f.wires,['grab status']);
  f.sim.receive('OK grab state=idle operation=pose reason=config_missing missing=z_ppm ref_cause=base_reference_changed');
  const feedback=text(f.$('grab-task-status'));
  assert.match(feedback,/抓取任务状态.*原因：缺少参数.*缺少参数：/);
  assert.match(feedback,/完整反馈与诊断.*参考状态.*Base.*参考失效/);
  assert.equal(f.sim.grabSnapshot.data.missing,'z_ppm');
  assert.deepEqual(f.wires,['grab status']);
});

test('actual task and calibration status cards translate causes without changing snapshots or sending',()=>{
  const f=consoleFixture();f.sim.mountGrabStatus();f.sim.buildGrabCalibrationPanel(f.$('system-body'));
  f.sim.receive('OK grab state=error mode=align operation=stack_align reason=matrix_rank ref_cause=transfer_off_pose missing=none');
  const text=e=>[e.textContent,...e.children.map(text)].join(' ');
  assert.match(text(f.$('grab-task-status')),/视觉响应矩阵.*无效/);
  assert.match(text(f.$('grab-task-status')),/Base.*观察姿态/);
  assert.doesNotMatch(text(f.$('grab-task-status')),/matrix_rank|transfer_off_pose/);
  assert.equal(f.sim.grabSnapshot.data.reason,'matrix_rank');
  f.sim.receive('OK grab calib phase=error ref=1 calibrated=1 reason=response_invalid fdx=2 fdy=0 xdx=0 xdy=3');
  assert.match(f.$('grab-calibration-status').textContent,/标定响应.*无效/);
  assert.equal(f.sim.calibrationSnapshot.data.reason,'response_invalid');
  assert.deepEqual(f.wires,[]);
});
