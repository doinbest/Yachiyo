import test from 'node:test';
import assert from 'node:assert/strict';
import {mountMap} from '../map-view.mjs';

function surface(){
  const nodes=new Map();let now=0,nextFrame;
  const matches=(node,selector)=>{
    if(selector==='input'||selector==='button'||selector==='tr')return node.tag===selector;
    const match=selector.match(/^\[data-([a-z-]+)(?:="([^"]*)")?\]$/);
    if(!match)return false;
    const key=match[1].replace(/-([a-z])/g,(_,letter)=>letter.toUpperCase());
    return match[2]===undefined?node.dataset[key]!==undefined:node.dataset[key]===match[2];
  };
  class Element{
    constructor(tag='div'){this.tag=tag;this.children=[];this.dataset={};this.style={};this.attributes={};this.events={};this.value='';this.hidden=false;this.classList={toggle(){}};}
    set id(value){this._id=value;nodes.set(value,this);} get id(){return this._id;}
    append(...children){for(const child of children){child.parent=this;this.children.push(child);}}
    replaceChildren(...children){this.children=[];this.append(...children);}
    setAttribute(key,value){this.attributes[key]=String(value);} getAttribute(key){return this.attributes[key]??null;}
    addEventListener(type,handler){this.events[type]=handler;}
    querySelectorAll(selector){return this.children.flatMap(child=>[...(matches(child,selector)?[child]:[]),...child.querySelectorAll(selector)]);}
    querySelector(selector){return this.querySelectorAll(selector)[0]??null;}
    closest(selector){return matches(this,selector)?this:this.parent?.closest(selector)??null;}
    reportValidity(){return true;}
  }
  const get=id=>{if(id==='map-source')return null;if(!nodes.has(id)){const node=new Element();node.id=id;}return nodes.get(id);};
  const buttons={};
  for(const [layer,context]of [['command','online'],['feedback','online'],['device'],['offline','offline'],['sim','offline']]){
    const button=new Element('button');button.dataset.mapLayer=layer;if(context)button.dataset.mapContext=context;button.setAttribute('aria-pressed','true');get('map-legend').append(button);buttons[layer]=button;
  }
  get('sim-model').value='ideal';get('sim-rate').value='1';
  const saved={document:globalThis.document,performance:globalThis.performance,requestAnimationFrame:globalThis.requestAnimationFrame,ResizeObserver:globalThis.ResizeObserver};
  globalThis.document={hidden:false,getElementById:get,createElement:tag=>new Element(tag),createElementNS:(_,tag)=>new Element(tag),addEventListener(){}};
  globalThis.performance={now:()=>now};globalThis.requestAnimationFrame=handler=>{nextFrame=handler;};globalThis.ResizeObserver=undefined;
  return {get,buttons,frame(ms=120){now+=ms;nextFrame(now);},click(id){return get(id).events.click({target:get(id)});},legend(layer){const target=buttons[layer];get('map-legend').events.click({target});},start(){get('sim-form').events.submit({preventDefault(){}});},restore(){Object.assign(globalThis,saved);}};
}
const config={v:1,kind:'config',session:42,rev:'map-test',geometry:{wheelbase_mm:210,track_mm:240,wheel_diameter_mm:80,gear_ratio:1},wheel_ids:[1,2,3,4],forward_dir:[0,0,1,1]};
const state=(t,valid=true)=>({v:1,kind:'state',session:42,t_ms:t,target:{body:[0,0,0],rpm:[0,0,0,0]},feedback:{rpm:[0,0,0,0],pos:[0,0,0,0],speed_valid:[true,true,true,true],position_valid:[valid,true,true,true],speed_ms:[t,t,t,t],position_ms:[t,t,t,t]},yaw:{valid:true,rad:Math.PI/2},pose:{command:{valid:true,x_mm:2000,y_mm:1000,yaw_rad:0},feedback:{valid:true,x_mm:2250,y_mm:150,yaw_rad:Math.PI/2}}});
const receive=(view,packet)=>view.receive('@CHASSIS '+JSON.stringify(packet));
function online(view){view.beginPreparedTelemetry(42);receive(view,config);receive(view,state(20));}

test('compact monitor has one explicit update toggle and hides empty live data and stop history',async()=>{
  const ui=surface(),wires=[];try{
    const view=mountMap({send:async wire=>{wires.push(wire);return true;}});
    assert.equal(ui.get('telemetry-status').textContent,'未连接设备');
    assert.equal(ui.get('monitor-live-data').hidden,true);
    assert.equal(ui.get('monitor-last-stop').hidden,true);
    assert.equal(ui.get('monitor-stop-history').hidden,true);
    assert.equal(ui.get('monitor-toggle').disabled,true);
    await ui.click('monitor-toggle');assert.deepEqual(wires,[]);
    view.connection(true);assert.deepEqual(wires,[]);
    assert.equal(ui.get('monitor-toggle').textContent,'开始监控');
    await ui.click('monitor-toggle');assert.match(wires[0],/^chassis stream on \d+$/);
    assert.equal(ui.get('monitor-toggle').textContent,'暂停更新');
    await ui.click('monitor-toggle');assert.equal(wires[1],'chassis stream off');
    assert.equal(ui.get('monitor-toggle').textContent,'开始监控');
    assert.equal(ui.get('telemetry-status').textContent,'更新已暂停');
    view.connection(false);view.connection(true);assert.equal(wires.length,2);
  }finally{ui.restore();}
});

test('compact monitor uses fresh motion data, preserves historical stops and never infers parking from zero speed',()=>{
  const ui=surface();try{
    const view=mountMap({send:async()=>true});view.connection(true);online(view);
    const packet=state(40);packet.target.body=[30,40,Math.PI/6];
    packet.route={state:'running',reason:'none',segment:3,total:15,action_id:12,target:[1200,2100,90],error:[1,2,0],feedback_valid:true,stop_confirmed:false,phase:'dock',phase_target:[1200,2100,90],speed_mm_s:200,phase_speed_mm_s:50};
    receive(view,packet);ui.frame();
    assert.equal(ui.get('monitor-live-data').hidden,false);
    assert.match(ui.get('monitor-current-target').textContent,/3.*15.*对接/);
    assert.equal(ui.get('monitor-stop-state').textContent,'运行中');
    assert.equal(ui.get('monitor-plan-speed').textContent,'50 mm/s');
    assert.match(ui.get('monitor-command-speed').textContent,/50\.0 mm\/s.*30\.0°\/s/);
    packet.t_ms=60;packet.target.body=[0,0,0];receive(view,packet);ui.frame();
    assert.equal(ui.get('monitor-stop-state').textContent,'运行中');
    ui.frame(1000);assert.equal(ui.get('monitor-live-data').hidden,true);
    assert.match(ui.get('telemetry-status').textContent,/更新中断/);
    packet.t_ms=80;packet.feedback.speed_ms=packet.feedback.position_ms=[80,80,80,80];
    packet.route.state='waiting';packet.route.reason='reached';packet.route.stop_confirmed=true;
    packet.pose.feedback.x_mm=1201;packet.pose.feedback.y_mm=2102;
    receive(view,packet);ui.frame();
    assert.equal(ui.get('monitor-stop-state').textContent,'停车已确认');
    assert.equal(ui.get('monitor-last-stop').hidden,false);assert.equal(ui.get('monitor-stop-history').hidden,false);
    assert.match(ui.get('monitor-final-stop').textContent,/X 1201\.0 · Y 2102\.0/);
    assert.match(ui.get('monitor-stop-delta').textContent,/ΔX 1\.0.*ΔY 2\.0/);
    view.connection(false);
    assert.equal(ui.get('monitor-live-data').hidden,true);assert.equal(ui.get('monitor-last-stop').hidden,false);
    assert.equal(ui.get('monitor-stop-results').children.length,1);
  }finally{ui.restore();}
});

test('failed monitor toggle does not invent subscription changes or send a parking command',async()=>{
  const ui=surface(),wires=[];let accepted=false;try{
    const view=mountMap({send:async wire=>{wires.push(wire);return accepted;}});view.connection(true);
    await ui.click('monitor-toggle');assert.equal(view.snapshot().session,null);
    assert.equal(ui.get('monitor-toggle').textContent,'开始监控');
    accepted=true;await ui.click('monitor-toggle');const session=view.snapshot().session;
    accepted=false;await ui.click('monitor-toggle');assert.equal(view.snapshot().session,session);
    assert.equal(ui.get('monitor-toggle').textContent,'暂停更新');
    assert.match(ui.get('telemetry-status').textContent,/暂停更新未发送/);
    assert.ok(wires.every(wire=>wire.startsWith('chassis stream ')));
  }finally{ui.restore();}
});

test('formal field defaults to the inset nonuniform grid and retains its 2400 mm outer frame',()=>{
  const ui=surface();try{
    mountMap({send:async()=>true});
    const grid=ui.get('field-static').children.filter(node=>node.attributes['data-radar-grid']!==undefined);
    assert.deepEqual(grid.map(node=>node.attributes.d),[150,550,1000,1400,1850,2250].map(mm=>`M ${mm} 150 V 2250 M 150 ${mm} H 2250`));
    assert.deepEqual(ui.get('field-static').children[0].attributes,{x:'0',y:'0',width:'2400',height:'2400',fill:'#c5c9ce',stroke:'#607486','stroke-width':'8'});
    assert.equal(ui.get('field-static').children.some(node=>node.attributes.d==='M 300 0 V 2400 M 0 300 H 2400'),false);
  }finally{ui.restore();}
});

test('field grid and cell labels follow frozen capture geometry without redrawing the field frame',()=>{
  const ui=surface();try{
    const view=mountMap({send:async()=>true}),frame=ui.get('field-static').children[0];
    view.setRadarGrid([0,550,1000,1400,1850,2400]);
    const grid=ui.get('field-static').children.filter(node=>node.attributes['data-radar-grid']!==undefined);
    const labels=ui.get('field-static').children.filter(node=>node.attributes.class==='monitor-grid-label');
    assert.equal(grid[0].attributes.d,'M 0 0 V 2400 M 0 0 H 2400');
    assert.equal(labels[0].attributes.transform,'translate(275 420) scale(1 -1)');
    assert.equal(labels[4].attributes.transform,'translate(2125 420) scale(1 -1)');
    assert.match(ui.get('radar-grid-legend').attributes.title,/0、550、1000、1400、1850、2400/);
    view.setRadarGrid([150,550,1000,1400,1850,2250]);
    assert.equal(grid[0].attributes.d,'M 150 150 V 2250 M 150 150 H 2250');
    assert.equal(labels[0].attributes.transform,'translate(350 420) scale(1 -1)');
    assert.equal(labels[4].attributes.transform,'translate(2050 420) scale(1 -1)');
    assert.equal(ui.get('field-static').children[0],frame);
  }finally{ui.restore();}
});

test('map mounts without a vehicle-source selector and defaults to one feedback vehicle',()=>{
  const ui=surface(),wires=[];try{
    let view;assert.doesNotThrow(()=>{view=mountMap({send:async wire=>{wires.push(wire);return true;}});});
    assert.match(ui.get('map-source-label').textContent,/反馈/);assert.doesNotMatch(ui.get('map-source-label').textContent,/离线/);
    online(view);ui.frame();
    assert.match(ui.get('map-pose').textContent,/X 2250\.0 · Y 150\.0/);
    assert.equal(ui.get('command-path').attributes.d,'M2000.00,1000.00 ');assert.equal(ui.get('feedback-path').attributes.d,'M2250.00,150.00 ');
    assert.notEqual(ui.get('command-path').style.display,'none');assert.notEqual(ui.get('feedback-path').style.display,'none');
    view.setSource('command');assert.match(ui.get('map-pose').textContent,/X 2250\.0 · Y 150\.0/);
    receive(view,state(40,false));ui.frame();assert.equal(ui.get('map-car').children.length,0);assert.equal(ui.get('map-pose').textContent,'不可用');
    assert.deepEqual(wires,[]);
  }finally{ui.restore();}
});

test('legend toggles only visibility and preserves preferences and collected tracks across modes',()=>{
  const ui=surface(),wires=[];try{
    const view=mountMap({send:async wire=>{wires.push(wire);return true;}});online(view);ui.frame();
    const before=view.snapshot().paths.command.length;
    ui.legend('command');assert.equal(ui.buttons.command.attributes['aria-pressed'],'false');assert.equal(ui.get('command-path').style.display,'none');
    ui.legend('device');assert.equal(ui.get('radar-device-path').style.display,'none');assert.equal(ui.get('radar-stations').style.display,'none');
    ui.get('radar-device-path').setAttribute('d','M2250,150 L2100,1200');receive(view,state(40));ui.frame();
    assert.equal(ui.get('radar-device-path').style.display,'none');assert.ok(view.snapshot().paths.command.length>before);
    view.setSource('offline');assert.equal(ui.buttons.command.hidden,true);assert.equal(ui.buttons.feedback.hidden,true);assert.equal(ui.buttons.sim.hidden,false);assert.equal(ui.buttons.offline.hidden,false);
    ui.legend('offline');assert.equal(ui.get('radar-offline-path').style.display,'none');
    view.setSource('feedback');assert.equal(ui.buttons.command.hidden,false);assert.equal(ui.buttons.offline.hidden,true);assert.equal(ui.get('command-path').style.display,'none');assert.notEqual(ui.get('feedback-path').style.display,'none');
    assert.equal(ui.get('command-path').attributes.d,'M2000.00,1000.00 L2000.00,1000.00 ');
    ui.legend('command');assert.notEqual(ui.get('command-path').style.display,'none');assert.equal(ui.buttons.command.attributes['aria-pressed'],'true');
    view.setSource('offline');assert.equal(ui.get('radar-offline-path').style.display,'none');assert.deepEqual(wires,[]);
  }finally{ui.restore();}
});

test('offline route remains a separate simulation and changing mode pauses until manual resume',()=>{
  const ui=surface(),wires=[];try{
    const view=mountMap({send:async wire=>{wires.push(wire);return true;}});view.setSource('offline');ui.start();ui.frame();
    const simulated=ui.get('sim-path').attributes.d;assert.ok(simulated.startsWith('M2250.00,150.00 L'));assert.match(ui.get('map-source-label').textContent,/历史16段时间示意/);
    view.setSource('feedback');ui.frame();assert.equal(ui.get('sim-path').style.display,'none');
    view.setSource('offline');ui.frame();assert.equal(ui.get('sim-path').attributes.d,simulated);assert.equal(ui.get('sim-resume').disabled,false);
    ui.click('sim-resume');ui.frame();assert.notEqual(ui.get('sim-path').attributes.d,simulated);
    ui.click('sim-reset');assert.equal(ui.get('sim-path').attributes.d,'M2250.00,150.00 ');assert.deepEqual(wires,[]);
  }finally{ui.restore();}
});

test('current C profile playback preserves historical editor rows and displays its stages without TX',()=>{
  const ui=surface(),wires=[];try{
    const view=mountMap({send:async wire=>{wires.push(wire);return true;}}),rows=ui.get('route-rows').children;
    const profile={points:[[2100,300,0,0]],speed_mm_s:200,turn_speed_deg_s:100,phases:[{kind:'move',phase:'dock',direction:'forward',segment:3,station:5,visit:1,target:[2100,1200,90],speed_mm_s:100}]};
    assert.equal(typeof view.setPreviewSource,'function');view.setSource('offline');view.setPreviewSource(profile);
    assert.match(ui.get('map-source-label').textContent,/当前固件 C.*未播放/);assert.deepEqual(wires,[]);
    view.startProfile(profile);ui.frame();assert.match(ui.get('map-source-label').textContent,/当前固件 C/);
    assert.match(ui.get('route-progress').textContent,/目标 3.*对接.*100/);assert.equal(ui.get('route-rows').children,rows);
    for(let i=0;i<100;i++)ui.frame();assert.match(ui.get('sim-status').textContent,/外框检查通过/);
    assert.match(ui.get('map-pose').textContent,/X 2100\.0 · Y 1200\.0/);assert.deepEqual(wires,[]);
  }finally{ui.restore();}
});

test('simulation legend hides stop footprints and labels while keeping the vehicle visible',()=>{
  const ui=surface();try{
    const view=mountMap({send:async()=>true});view.setSource('offline');ui.start();
    for(let i=0;i<30;i++)ui.frame();
    assert.ok(ui.get('route-stops').children.some(node=>node.tag==='text'));
    const position=ui.get('map-pose').textContent;
    ui.legend('sim');assert.equal(ui.get('sim-path').style.display,'none');assert.equal(ui.get('route-stops').style.display,'none');
    assert.ok(ui.get('map-car').children.length>0);assert.equal(ui.get('map-pose').textContent,position);
    ui.legend('sim');assert.notEqual(ui.get('sim-path').style.display,'none');assert.notEqual(ui.get('route-stops').style.display,'none');
    assert.ok(ui.get('route-stops').children.some(node=>node.tag==='text'));
  }finally{ui.restore();}
});


test('map angular speed is displayed in degrees per second without changing telemetry units',()=>{
  const ui=surface();try{
    const view=mountMap({send:async()=>true});online(view);
    const packet=state(40);packet.target.body[2]=Math.PI/6;receive(view,packet);ui.frame();
    assert.match(ui.get('map-body-speed').textContent,/ω 30\.0 °\/s/);
    view.setSource('offline');ui.start();ui.frame();
    assert.match(ui.get('map-body-speed').textContent,/°\/s/);
  }finally{ui.restore();}
});
