import test from 'node:test';
import assert from 'node:assert/strict';
import {parameterSurface} from './parameter-surface.mjs';

async function mount(options){
  const module=await import('../static-align-gain.mjs').catch(error=>{
    if(error.code==='ERR_MODULE_NOT_FOUND')return {};
    throw error;
  });
  assert.equal(typeof module.mountStaticAlignGain,'function','静态协同对准应提供独立λ控件');
  return module.mountStaticAlignGain(options);
}

test('static gain mounting, connecting and editing never send; bounds follow firmware',async()=>{
  const ui=parameterSurface(),wires=[];
  try{
    const panel=await mount({root:ui.root,send:async wire=>{wires.push(wire);return true;},canSend:()=>true});
    panel.connection(true);
    const input=ui.root.querySelector('#static-align-lambda');
    assert.equal(Number(input.value),2.5);assert.equal(Number(input.min),.001);assert.equal(Number(input.max),10);
    input.value='4';input.oninput();panel.update();
    assert.deepEqual(wires,[]);
    assert.ok(ui.all().some(item=>item.textContent.includes('原料追踪')));
  }finally{ui.restore();}
});

test('static gain explicit read and apply verify only shared grab lambda',async()=>{
  const ui=parameterSurface(),wires=[],occupations=[];let panel,value=2.5;
  try{
    panel=await mount({root:ui.root,canSend:()=>true,onBusy:busy=>occupations.push(busy),send:async wire=>{
      wires.push(wire);queueMicrotask(()=>{
        if(wire.startsWith('grab set')){value=Number(wire.split(' ')[3]);panel.receive('OK grab set');}
        else panel.receive(`OK grab config key=lambda value=${value}`);
      });return true;
    }});
    panel.connection(true);await panel.read();
    const input=ui.root.querySelector('#static-align-lambda');input.value='4';input.oninput();
    await panel.apply();
    assert.deepEqual(wires,['grab get lambda','grab set lambda 4','grab get lambda']);
    assert.deepEqual(occupations,[true,false,true,false]);
    assert.ok(ui.all().some(item=>item.textContent.includes('读回一致')));
  }finally{ui.restore();}
});

test('static gain rejects invalid drafts without transmitting',async()=>{
  const ui=parameterSurface(),wires=[];
  try{
    const panel=await mount({root:ui.root,canSend:()=>true,send:async wire=>{wires.push(wire);return true;}});
    panel.connection(true);
    const input=ui.root.querySelector('#static-align-lambda');
    for(const value of ['', '0', '-1', '.0009', '10.1', 'NaN']){input.value=value;input.oninput();await panel.apply();}
    assert.deepEqual(wires,[]);
  }finally{ui.restore();}
});

test('static gain reports mismatched readback and device busy without claiming success',async()=>{
  const ui=parameterSurface(),wires=[];let panel,reject=false;
  try{
    panel=await mount({root:ui.root,canSend:()=>true,send:async wire=>{
      wires.push(wire);queueMicrotask(()=>panel.receive(reject?'ERR grab task_running':wire.startsWith('grab set')?'OK grab set':'OK grab config key=lambda value=2.5'));return true;
    }});
    panel.connection(true);const input=ui.root.querySelector('#static-align-lambda');input.value='4';input.oninput();
    await panel.apply();assert.ok(ui.all().some(item=>item.textContent.includes('不一致')));
    reject=true;wires.length=0;await panel.apply();
    assert.deepEqual(wires,['grab set lambda 4']);
    assert.ok(ui.all().some(item=>item.textContent.includes('任务忙')));
  }finally{ui.restore();}
});

test('disconnect cancels pending gain write and prevents a trailing read or replay',async()=>{
  const ui=parameterSurface(),wires=[];let panel;
  try{
    panel=await mount({root:ui.root,canSend:()=>true,send:async wire=>{
      wires.push(wire);queueMicrotask(()=>panel.connection(false));return true;
    }});
    panel.connection(true);const input=ui.root.querySelector('#static-align-lambda');input.value='4';input.oninput();
    await panel.apply();panel.connection(true);
    assert.deepEqual(wires,['grab set lambda 4']);assert.equal(panel.busy,false);
    assert.ok(ui.all().some(item=>item.textContent.includes('历史')||item.textContent.includes('取消')));
  }finally{ui.restore();}
});
