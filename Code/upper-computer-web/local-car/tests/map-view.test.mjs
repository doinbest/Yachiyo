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
  return {get,buttons,frame(ms=120){now+=ms;nextFrame(now);},click(id){get(id).events.click({target:get(id)});},legend(layer){const target=buttons[layer];get('map-legend').events.click({target});},start(){get('sim-form').events.submit({preventDefault(){}});},restore(){Object.assign(globalThis,saved);}};
}
const config={v:1,kind:'config',session:42,rev:'map-test',geometry:{wheelbase_mm:210,track_mm:240,wheel_diameter_mm:80,gear_ratio:1},wheel_ids:[1,2,3,4],forward_dir:[0,0,1,1]};
const state=(t,valid=true)=>({v:1,kind:'state',session:42,t_ms:t,target:{body:[0,0,0],rpm:[0,0,0,0]},feedback:{rpm:[0,0,0,0],pos:[0,0,0,0],speed_valid:[true,true,true,true],position_valid:[valid,true,true,true],speed_ms:[t,t,t,t],position_ms:[t,t,t,t]},yaw:{valid:true,rad:Math.PI/2},pose:{command:{valid:true,x_mm:2000,y_mm:1000,yaw_rad:0},feedback:{valid:true,x_mm:2250,y_mm:150,yaw_rad:Math.PI/2}}});
const receive=(view,packet)=>view.receive('@CHASSIS '+JSON.stringify(packet));
function online(view){view.beginPreparedTelemetry(42);receive(view,config);receive(view,state(20));}

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
