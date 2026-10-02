import test from 'node:test';
import assert from 'node:assert/strict';
import {mountParameterPage} from '../parameter-page.mjs';
import {parameterFields} from '../parameter-schema.mjs';
import {parameterSurface} from './parameter-surface.mjs';
const tick=()=>new Promise(resolve=>setImmediate(resolve));

test('read all refreshes successful fields and unset, keeps failed input, gets radar once',async()=>{
  const ui=parameterSurface(),wires=[];let page,radarGets=0;
  try{
    page=mountParameterPage({root:ui.root,canSend:()=>true,send:async wire=>{
      wires.push(wire);queueMicrotask(()=>{
        if(wire==='grab get x_car')page.receive('ERR grab unknown_key');
        else if(wire.startsWith('config '))page.receive(`OK config ${wire.split(' ')[1]} rpm=30 acc=8 limit_pulses=${wire==='config z'?'unlimited':'3200'}`);
        else{const [kind,,key]=wire.split(' ');page.receive(`OK ${kind} config key=${key} value=${key==='z_proc_grab'||key.startsWith('slot2')?'unset':key==='z_grab'?'-80':key==='z_lift'?'-40':'1'}`);}
      });return true;
    },radarExchange:{async get(){radarGets++;return parameterFields.filter(p=>p.kind==='radar').map(p=>({key:p.key,value:5}));},cancel(){}}});
    ui.input('grab','x_car').value='42';ui.input('grab','x_car').oninput();
    assert.equal(ui.input('grab','z_grab').value,'');await page.readAll();
    assert.equal(ui.input('grab','z_grab').value,'80');assert.equal(ui.input('grab','z_lift').value,'40');
    assert.equal(ui.input('grab','z_proc_grab').value,'');assert.equal(ui.input('grab','x_car').value,'42');
    assert.equal(radarGets,1);assert.equal(wires.some(w=>w.includes('route_speed')),false);
    assert.equal(ui.all().filter(e=>e.tag==='details').length,0);
    assert.ok(ui.all().some(e=>e.textContent==='未支持'));
  }finally{ui.restore();}
});

test('apply current tab writes paired absolute Z and reads each back; other drafts persist',async()=>{
  const ui=parameterSurface(),wires=[],device={};let page;
  try{
    page=mountParameterPage({root:ui.root,canSend:()=>true,send:async wire=>{
      wires.push(wire);const [,action,key,value]=wire.split(' ');
      queueMicrotask(()=>{if(action==='set'){device[key]=Number(value);page.receive('OK grab set');}else page.receive(`OK grab config key=${key} value=${device[key]}`);});return true;
    }});
    for(const [key,value]of [['z_grab','70'],['z_lift','40'],['ref_u','334']]){ui.input('grab',key).value=value;ui.input('grab',key).oninput();}
    await page.apply();assert.deepEqual(wires,['grab set z_grab -70','grab get z_grab','grab set z_lift -30','grab get z_lift']);
    page.selectTab('vision');assert.equal(ui.input('grab','ref_u').value,'334');assert.equal(ui.input('grab','z_lift').value,'40');
    assert.ok(ui.all().some(e=>e.textContent==='读回一致'));
  }finally{ui.restore();}
});

test('stop cancels remaining requests without clearing drafts and disconnect marks historical',async()=>{
  const ui=parameterSurface(),wires=[];let page;
  try{
    page=mountParameterPage({root:ui.root,canSend:()=>true,send:async wire=>{wires.push(wire);return true;}});
    ui.input('grab','x_car').value='42';ui.input('grab','x_car').oninput();const reading=page.readAll();await tick();page.receive('OK grab config key=z_ppm value=480');await tick();page.cancel('停止');await reading;
    assert.equal(wires.length,2);assert.equal(ui.input('grab','x_car').value,'42');assert.equal(ui.input('grab','z_ppm').value,'480');
    page.cancel('断线',true);assert.ok(ui.all().some(e=>e.textContent.includes('历史设备值')));
  }finally{ui.restore();}
});

test('rejected apply retains draft, reports the reason and never reports readback consistency',async()=>{
  const ui=parameterSurface(),wires=[];let page;
  try{
    page=mountParameterPage({root:ui.root,canSend:()=>true,send:async wire=>{wires.push(wire);queueMicrotask(()=>page.receive('ERR grab busy'));return true;}});
    page.selectTab('motion');ui.input('grab','close_ms').value='700';ui.input('grab','close_ms').oninput();await page.apply();
    assert.deepEqual(wires,['grab set close_ms 700']);assert.equal(ui.input('grab','close_ms').value,'700');
    assert.ok(ui.all().some(e=>e.textContent.includes('任务忙')&&e.textContent.includes('grab busy')));assert.equal(ui.all().some(e=>e.textContent==='读回一致'),false);
  }finally{ui.restore();}
});

test('inventory accepts all existing B2 color IDs without reclassifying them',async()=>{
  const {buildCommand,grabColorOptions}=await import('../protocol.mjs');
  for(let color=1;color<=6;color++)assert.equal(buildCommand('turntable-inventory-set',{slot:2,content:color}),`turntable inventory 2 ${color}`);
  assert.equal(grabColorOptions[3][1],'绿色 · 4');assert.equal(grabColorOptions[4][1],'黑色 · 5');
});

test('successful paired descent read does not discard a lift draft when the lift read fails',async()=>{
  const ui=parameterSurface();let page,secondRead=false;
  try{
    page=mountParameterPage({root:ui.root,canSend:()=>true,send:async wire=>{
      queueMicrotask(()=>{
        const [kind,,key]=wire.split(' ');
        if(wire==='grab get z_lift'&&secondRead)page.receive('ERR grab busy');
        else if(kind==='config')page.receive(`OK config ${wire.split(' ')[1]} rpm=30 acc=8 limit_pulses=${wire==='config z'?'unlimited':'3200'}`);
        else page.receive(`OK ${kind} config key=${key} value=${key==='z_grab'?'-70':key==='z_lift'?'-40':'1'}`);
      });return true;
    },radarExchange:{async get(){return parameterFields.filter(p=>p.kind==='radar').map(p=>({key:p.key,value:5}));},cancel(){}}});
    await page.readAll();assert.equal(ui.input('grab','z_lift').value,'30');
    const lift=ui.input('grab','z_lift');lift.value='55';lift.oninput();page.cancel('断线',true);secondRead=true;await page.readAll();
    assert.equal(lift.value,'55');assert.ok(ui.all().some(e=>e.textContent.includes('任务忙')&&e.textContent.includes('grab busy')));
    const row=ui.all().find(e=>e.id==='parameter-row-grab-z_lift');
    assert.ok(row.children.some(e=>e.textContent.includes('历史设备值')));
  }finally{ui.restore();}
});

test('a deep link for a shared key scrolls only the requested parameter category',()=>{
  const ui=parameterSurface();
  try{
    const page=mountParameterPage({root:ui.root,canSend:()=>false,send:async()=>true}),scrolled=[];
    for(const row of ui.all().filter(e=>e.className==='parameter-field'))row.scrollIntoView=()=>scrolled.push(row.id);
    page.selectTab('turntable','stable_ms');
    assert.deepEqual(scrolled,['parameter-row-turntable-stable_ms']);
    assert.equal(new Set(ui.all().filter(e=>e.id).map(e=>e.id)).size,ui.all().filter(e=>e.id).length);
  }finally{ui.restore();}
});

test('material height fields stay in six aligned scenarios with all parameter inputs intact',()=>{
  const ui=parameterSurface();
  try{
    mountParameterPage({root:ui.root,canSend:()=>false,send:async()=>true});
    const all=ui.all(),scenarios=all.filter(e=>e.className==='parameter-height-row');
    assert.equal(scenarios.length,6);
    assert.deepEqual(scenarios.map(e=>e.children[0].textContent),['原料夹取','粗加工夹取','车载取放','粗加工释放','暂存第一层','暂存第二层']);
    assert.equal(all.filter(e=>e.className==='parameter-empty-cell').length,3);
    assert.equal(all.filter(e=>e.dataset.parameterKey).length,parameterFields.length);
    for(const row of all.filter(e=>e.className==='parameter-field')){
      const label=row.children.find(e=>e.tag==='label');
      assert.ok(all.some(e=>e.id===label.htmlFor));
    }
    assert.ok(all.some(e=>e.className==='parameter-unit'&&e.textContent==='ms'));
    assert.ok(all.some(e=>e.textContent.includes('发出开合指令后')));
  }finally{ui.restore();}
});
