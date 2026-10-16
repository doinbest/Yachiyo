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
    paramsView:{receive:empty,cancel:empty,update:empty},staticAlignGain:null,trackingPanel:null,materialTrackingPanel:null,stackingPanel:null,missionPanel:null,headingPanel:null,chassisTestResult:null,chassisTestSession:null,stopRecovery:null,
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
    ${section('function buildGrabPreparation(body)','function parameterLink(')}
    ${section('function receive(line)','function communicationError(error)')}
    ${section('function renderGrabCalibrationStatus()','function buildVisionWorkspace(')}
    ${section('function renderGrabStatus()','function buildPages()')}
    globalThis.sim={canSend,send,receive,connection,controlForm,updateControls,buildGrabCalibrationPanel,buildGrabPreparation,
      setStaticAlignGain(panel){staticAlignGain=panel;},
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

test('Base quick HOME stays passive until clicked and sends nearest mode',async()=>{
  const f=consoleFixture();f.sim.buildGrabPreparation(f.$('home-test-root'));
  const button=f.elements.find(e=>e.tag==='button'&&e.title==='home base near');
  assert.ok(button);assert.equal(button.textContent,'就近回零');
  assert.ok(f.elements.some(e=>e.title==='home x collision'));
  assert.ok(f.elements.some(e=>e.title==='home z collision'));
  assert.deepEqual(f.wires,[]);
  button.onclick();await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['home base near']);
});

test('gripper microsecond edits stay passive and apply once through existing firmware',async()=>{
  const f=consoleFixture();f.sim.controlForm('grip-pulse',{compact:true});
  const input=f.$('grip-pulse-pulse'),form=f.elements.find(e=>e.tag==='form');
  assert.equal(Number(input.value),2450);assert.deepEqual(f.wires,[]);
  input.value='2300';input.oninput();assert.deepEqual(f.wires,[]);
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['grip duty 11.5']);
  input.value='2501';input.oninput();form.onsubmit({preventDefault(){}});
  await new Promise(resolve=>setTimeout(resolve,0));assert.equal(f.wires.length,1);
  f.sim.configure({stopLatched:true});input.value='2100';input.oninput();
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.equal(f.wires.length,1);
});
test('standalone chassis test remains passive until submit and respects stop lock',async()=>{
  const f=consoleFixture();f.sim.controlForm('chassis-test',{compact:true});
  const direction=f.$('chassis-test-direction'),distance=f.$('chassis-test-distance');
  const profile=f.$('chassis-test-profile');assert.equal(profile.value,'linear');
  const form=f.elements.find(e=>e.tag==='form');
  assert.deepEqual(f.wires,[]);direction.value='left';direction.onchange();
  distance.value='1000';distance.oninput();assert.deepEqual(f.wires,[]);
  profile.value='cosine';profile.onchange();assert.deepEqual(f.wires,[]);
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['chassis test 0 1000 1000 cosine 200 0.1 0']);
  f.sim.configure({stopLatched:true});distance.value='500';distance.oninput();
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.equal(f.wires.length,1);
});

test('single-test form delegates one asymmetric motion to its lifecycle',async()=>{
 const f=consoleFixture(),started=[];let busy=false;
 f.sim.controlForm('chassis-test',{compact:true,onExecute:w=>started.push(w),isBusy:()=>busy});
 const form=f.elements.find(e=>e.tag==='form');
 form.onsubmit({preventDefault(){}});
 assert.deepEqual(started,['chassis test 500 0 1000 linear 500 0.1 0 1 300']);assert.deepEqual(f.wires,[]);
 busy=true;form.onsubmit({preventDefault(){}});assert.equal(started.length,1);
});
test('deceleration input is independent and inactive under cosine',()=>{
 const f=consoleFixture(),started=[];f.sim.controlForm('chassis-test',{onExecute:w=>started.push(w)});
 const form=f.elements.find(e=>e.tag==='form'),decel=f.$('chassis-test-decel'),profile=f.$('chassis-test-profile');
 const label=f.elements.find(e=>e.tag==='label'&&e.children.includes(decel));
 assert.equal(Number(decel.value),300);decel.value='250';decel.oninput();form.onsubmit({preventDefault(){}});
 assert.equal(started.at(-1),'chassis test 500 0 1000 linear 500 0.1 0 1 250');
 profile.value='cosine';profile.onchange();assert.equal(decel.disabled,true);assert.equal(label.hidden,true);
 profile.value='linear';profile.onchange();assert.equal(decel.value,'250');assert.equal(decel.disabled,false);
});
test('linear acceleration input is per test and cosine hides it without losing its draft',()=>{
 const f=consoleFixture(),started=[];f.sim.controlForm('chassis-test',{onExecute:w=>started.push(w)});
 const form=f.elements.find(e=>e.tag==='form'),accel=f.$('chassis-test-accel'),profile=f.$('chassis-test-profile');
 const label=f.elements.find(e=>e.tag==='label'&&e.children.includes(accel));
 accel.value='800';accel.oninput();assert.deepEqual(started,[]);assert.deepEqual(f.wires,[]);
 form.onsubmit({preventDefault(){}});assert.equal(started.at(-1),'chassis test 500 0 1000 linear 800 0.1 0 1 300');
 profile.value='cosine';profile.onchange();assert.equal(accel.disabled,true);assert.equal(label.hidden,true);
 form.onsubmit({preventDefault(){}});assert.equal(started.at(-1),'chassis test 500 0 1000 cosine 200 0.1 0');
 profile.value='linear';profile.onchange();assert.equal(label.hidden,false);assert.equal(accel.value,'800');
 accel.value='0';accel.oninput();form.onsubmit({preventDefault(){}});assert.equal(started.length,2);
});

test('ring comparison stays passive and issues one explicit fixed-height alignment',async()=>{
 const f=consoleFixture();f.sim.controlForm('grab-ring-align-top',{compact:true});
 const input=f.$('grab-ring-align-top-ring'),form=f.elements.find(e=>e.tag==='form');
 input.value='2';input.onchange();assert.deepEqual(f.wires,[]);
 form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
 assert.deepEqual(f.wires,['grab align ring 2 0 stay']);
 assert.equal(readOnlyCommand('bus recovery'),true);
});

test('lidar servo manual input stays passive and scan/stow remain explicit motions',async()=>{
  const f=consoleFixture();f.sim.controlForm('lidar-servo-pulse',{compact:true});
  const input=f.$('lidar-servo-pulse-pulse'),form=f.elements.find(e=>e.tag==='form');
  assert.equal(Number(input.value),1450);assert.deepEqual(f.wires,[]);
  input.value='1537';input.oninput();assert.deepEqual(f.wires,[]);
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['lidarservo pulse 1537']);
  input.value='5000';input.oninput();
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['lidarservo pulse 1537','lidarservo pulse 5000']);
  input.value='5001';input.oninput();
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['lidarservo pulse 1537','lidarservo pulse 5000']);
  f.sim.configure({stopLatched:true,sending:1,parameterBusy:true,preparing:true,pending:'imu status'});
  assert.equal(f.sim.canSend('lidarservo out'),false);
  assert.equal(f.sim.canSend('lidarservo in'),false);
  assert.equal(f.sim.canSend('lidarservo off'),false);
  assert.equal(f.sim.canSend('lidarservo pulse 500'),false);
  f.sim.configure({stopLatched:false,sending:0,parameterBusy:false,preparing:false,pending:null});
  await f.sim.send('lidarservo out');await f.sim.send('lidarservo in');
  assert.deepEqual(f.wires,['lidarservo pulse 1537','lidarservo pulse 5000','lidarservo out','lidarservo in']);
});

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
test('car pickup form stays passive and submits the selected tray with a 40mm default',async()=>{
  const f=consoleFixture();f.sim.controlForm('grab-take-work',{compact:true});
  const slot=f.$('grab-take-work-slot'),depth=f.$('grab-take-work-depth');
  assert.equal(Number(depth.value),40);assert.deepEqual(f.wires,[]);
  slot.value='2';
  const form=f.elements.find(e=>e.tag==='form');
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['grab take 2 work 40']);
  f.sim.configure({stopLatched:true});depth.value='50';
  form.onsubmit({preventDefault(){}});await new Promise(resolve=>setTimeout(resolve,0));
  assert.deepEqual(f.wires,['grab take 2 work 40']);
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
test('ring options edit without sending and submit one selected alignment command',async()=>{
  const f=consoleFixture();const form=f.sim.controlForm('grab-ring-align-options',{compact:true});
  const height=f.$('grab-ring-align-options-height'),finish=f.$('grab-ring-align-options-finish'),z=f.$('grab-ring-align-options-z');
  const button=f.elements.find(e=>e.type==='submit');
  assert.equal(button.title,'grab align ring 1 0 stay');
  assert.equal(z.disabled,true);
  height.value='raised';height.onchange();assert.equal(z.disabled,true);
  finish.value='stay';finish.onchange();assert.equal(button.title,'grab align ring 1 0 stay');
  assert.deepEqual(f.wires,[]);
  form.onsubmit({preventDefault(){}});await Promise.resolve();await Promise.resolve();
  assert.deepEqual(f.wires,['grab align ring 1 0 stay']);
  height.value='lowered';height.onchange();assert.equal(z.disabled,false);assert.equal(z.value,-100);
  assert.equal(button.title,'grab align ring 1 -100 stay');
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

test('manual tray pickup reports unspecified color without asking for stock correction',()=>{
  const f=consoleFixture();f.sim.mountGrabStatus();
  f.sim.receive('OK grab state=prepare mode=fixed operation=car_pick color=0 slot=2 reason=car_pick_prepare missing=none');
  const text=e=>[e.textContent,...e.children.map(text)].join(' ');
  assert.match(text(f.$('grab-task-status')),/无需指定颜色/);
  assert.equal(f.sim.grabSnapshot.data.color,0);assert.deepEqual(f.wires,[]);
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

test('ring tray placement form is passive and sends one selected-tray command',async()=>{
 const f=consoleFixture();f.sim.controlForm('grab-ring-take',{compact:true});
 assert.deepEqual(f.wires,[]);f.$('grab-ring-take-slot').value='3';
 const form=f.elements.find(e=>e.tag==='form');form.onsubmit({preventDefault(){}});
 await new Promise(resolve=>setTimeout(resolve,0));assert.deepEqual(f.wires,['grab ring take 3 110 25']);
 f.sim.configure({stopLatched:true});form.onsubmit({preventDefault(){}});
 await new Promise(resolve=>setTimeout(resolve,0));assert.deepEqual(f.wires,['grab ring take 3 110 25']);
});

test('standalone ring task status omits unused route and QR fields',()=>{
 const f=consoleFixture();f.sim.mountGrabStatus();
 f.sim.receive('OK grab route state=car_pick mode=ring_place source=qr code=none slot=3 ring=2 elapsed_ms=100 reason=car_pick');
 const card=f.$('grab-task-status').children.find(e=>e.className==='grab-route-status');
 const text=card.children.map(e=>e.textContent).join(' ');
 assert.match(text,/编号环自动放置.*2号环.*3号料盘/);
 assert.doesNotMatch(text,/第 — 段|来源 qr|none|路线原因/);
 assert.deepEqual(f.wires,[]);
});


test('static gain hooks receive replies and cancel on stop, reboot and disconnect',async()=>{
  const f=consoleFixture(),calls=[];
  f.sim.setStaticAlignGain({receive:line=>calls.push(['reply',line]),cancel:reason=>calls.push(['cancel',reason]),connection:connected=>calls.push(['connection',connected]),update(){}});
  f.sim.receive('OK grab config key=lambda value=4');
  assert.ok(calls.some(([kind,line])=>kind==='reply'&&line==='OK grab config key=lambda value=4'));
  await f.sim.send('grab stop');
  assert.ok(calls.some(([kind,reason])=>kind==='cancel'&&reason.includes('停止')));
  f.sim.receive('STM32 mechanical arm console ready');
  assert.ok(calls.some(([kind,reason])=>kind==='cancel'&&reason.includes('重启')));
  f.sim.connection('disconnected');
  assert.ok(calls.some(([kind,value])=>kind==='connection'&&value===false));
  assert.deepEqual(f.wires,['grab stop']);
});
