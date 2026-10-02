import test from 'node:test';
import assert from 'node:assert/strict';
import {createParameterExchange,parseGrabConfig,parseArmConfig} from '../parameter-console.mjs';
import {mountParameterPage} from '../parameter-page.mjs';

test('parameter replies parse only complete device readbacks',()=>{
  assert.deepEqual(parseGrabConfig('OK grab config key=travel_mm value=0'),{key:'travel_mm',value:0});
  assert.deepEqual(parseGrabConfig('OK grab config key=ref_u value=unset'),{key:'ref_u',value:null});
  assert.equal(parseGrabConfig('OK grab set'),null);
  assert.equal(parseGrabConfig('OK grab config key=travel_mm value=nan'),null);
  assert.deepEqual(parseArmConfig('OK config z rpm=500 acc=120 limit_pulses=unlimited'),{axis:'z',rpm:500,acc:120,limit:0});
  assert.deepEqual(parseArmConfig('OK config base rpm=30 acc=20 limit_pulses=3200'),{axis:'base',rpm:30,acc:20,limit:3200});
  assert.equal(parseArmConfig('OK config all rpm=30 acc=20 limit_pulses=3200'),null);
});

test('grab setting is applied only after a matching readback',async()=>{
  const wires=[];let exchange;
  exchange=createParameterExchange({send:async wire=>{
    wires.push(wire);
    if(wire==='grab get travel_mm')queueMicrotask(()=>exchange.receive('OK grab config key=travel_mm value=0'));
    return true;
  }});
  assert.equal(exchange.busy,false);
  assert.deepEqual(await exchange.setGrab('travel_mm',0),{key:'travel_mm',value:0,confirmed:true});
  assert.deepEqual(wires,['grab set travel_mm 0','grab get travel_mm']);
  assert.equal(exchange.busy,false);
});

test('mismatched readback and unrelated telemetry do not claim success',async()=>{
  let exchange;
  exchange=createParameterExchange({send:async wire=>{
    if(wire==='grab get body_speed')queueMicrotask(()=>{
      exchange.receive('OK grab state=idle mode=align reason=none');
      exchange.receive('OK grab config key=travel_mm value=0');
      exchange.receive('OK grab config key=body_speed value=10');
    });
    return true;
  }});
  assert.deepEqual(await exchange.setGrab('body_speed',20),{key:'body_speed',value:10,confirmed:false});
  await assert.rejects(exchange.setGrab('not_a_key',1));
});

test('small nonzero readback cannot be mistaken for disabled travel limit',async()=>{
  let exchange;
  exchange=createParameterExchange({send:async wire=>{
    if(wire==='grab get travel_mm')queueMicrotask(()=>exchange.receive('OK grab config key=travel_mm value=0.000000001'));
    return true;
  }});
  assert.equal((await exchange.setGrab('travel_mm',0)).confirmed,false);
});

test('six-significant-digit firmware readback accepts normal rounding',async()=>{
  let exchange;
  exchange=createParameterExchange({send:async wire=>{
    if(wire==='grab get body_speed')queueMicrotask(()=>exchange.receive('OK grab config key=body_speed value=1.23457'));
    return true;
  }});
  assert.equal((await exchange.setGrab('body_speed',1.234567)).confirmed,true);
});

test('arm configuration reads back all fields after set',async()=>{
  const wires=[];let exchange;
  exchange=createParameterExchange({send:async wire=>{
    wires.push(wire);
    if(wire==='config z')queueMicrotask(()=>exchange.receive('OK config z rpm=500 acc=120 limit_pulses=unlimited'));
    return true;
  }});
  assert.deepEqual(await exchange.setArm('z',500,120,0),{axis:'z',rpm:500,acc:120,limit:0,confirmed:true});
  assert.deepEqual(wires,['config z 500 120 0','config z']);
});

test('disconnect cancels a pending read and ignores late replies',async()=>{
  const exchange=createParameterExchange({send:async()=>true,timeoutMs:200});
  const reading=exchange.readGrab('travel_mm');
  exchange.cancel('串口已断开');
  await assert.rejects(reading,/串口已断开/);
  assert.equal(exchange.receive('OK grab config key=travel_mm value=0'),false);
  assert.equal(exchange.busy,false);
});

test('opening the parameter page sends nothing; route speed stays local until route start',()=>{
  class Element{
    constructor(tag){this.tag=tag;this.children=[];this.textContent='';this.value='';this.disabled=false;}
    append(...children){this.children.push(...children);}
    get lastChild(){return this.children.at(-1);}
    setAttribute(){}
  }
  const original=globalThis.document;
  globalThis.document={createElement:tag=>new Element(tag)};
  try{
    const root=new Element('div'),wires=[],saved=[];
    const page=mountParameterPage({root,send:wire=>{wires.push(wire);return Promise.resolve(true);},
      canSend:()=>false,onBusy:()=>{},onRouteSpeed:speed=>saved.push(speed)});
    assert.deepEqual(wires,[]);
    const route=root.children.at(-1);
    assert.equal(route.children[2].value,'5000');
    route.children[2].value='250';
    route.children[3].onclick();
    assert.deepEqual(saved,[250]);
    assert.deepEqual(wires,[]);
    page.cancel('串口已断开');
    assert.deepEqual(wires,[]);
  }finally{globalThis.document=original;}
});
