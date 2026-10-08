import test from 'node:test';
import assert from 'node:assert/strict';
import {buildCommand} from '../protocol.mjs';

test('stack alignment requests its compensation branch and descent never includes release',()=>{
  assert.equal(buildCommand('grab-stack-align',{color:'3'}),'grab stack align 3');
  assert.equal(buildCommand('grab-stack-lower',{z:-92}),'grab stack lower -92');
  assert.equal(buildCommand('grab-stack-release'),'grab stack release');
  assert.equal(buildCommand('grab-stack-lift'),'grab stack lift');
});

test('real stacking panel distinguishes green alignment, compensation and ready to load',async()=>{
  const f=await panelFixture();
  f.elements.find(e=>e.textContent==='静态对准并补偿').onclick();
  assert.deepEqual(f.wires,['grab stack align 3']);
  f.panel.receive({state:'align',reason:'stack_compensating'});
  assert.ok(f.elements.some(e=>e.textContent.includes('正在执行码垛专用补偿')));
  // Short MCU progress replies may omit operation but always retain reason.
  f.panel.receive({state:'hold',reason:'stack_aligned_ready'});
  assert.ok(f.elements.some(e=>e.textContent.includes('可手动装料并夹紧')));
});

function fakeClock(){
  let now=0,next=0;const pending=new Map();
  return {schedule(fn,ms){const id=++next;pending.set(id,{fn,at:now+ms});return id;},cancel:id=>pending.delete(id),
    advance(ms){now+=ms;for(const[id,item]of [...pending])if(item.at<=now){pending.delete(id);item.fn();}}};
}
async function controller(){
  const module=await import('../stack-panel.mjs').catch(error=>assert.fail(`height wheel is missing: ${error.message}`));
  const clock=fakeClock(),sent=[],previews=[];
  const wheel=module.createHeightWheel({initial:-100,min:-150,max:0,step:1,schedule:clock.schedule,cancel:clock.cancel,
    onCommit:z=>sent.push(z),onPreview:z=>previews.push(z)});
  return {wheel,clock,sent,previews};
}
test('wheel batches all scrolling into one absolute target after a full idle second',async()=>{
  const f=await controller();f.wheel.scroll(1);f.clock.advance(700);f.wheel.scroll(1);
  f.clock.advance(999);assert.deepEqual(f.sent,[]);assert.equal(f.previews.at(-1),-102);
  f.clock.advance(1);assert.deepEqual(f.sent,[-102]);
});
test('stop cancels the deferred command and a later scroll gets a new full delay',async()=>{
  const f=await controller();f.wheel.scroll(1);f.wheel.cancel();f.clock.advance(2000);
  assert.deepEqual(f.sent,[]);f.wheel.scroll(-1);f.clock.advance(1000);assert.deepEqual(f.sent,[-100]);
});
test('height stays within the mechanical range and typed edits cancel stale wheel submissions',async()=>{
  const f=await controller();f.wheel.scroll(999);f.clock.advance(1000);assert.deepEqual(f.sent,[-150]);
  f.wheel.scroll(-1);f.wheel.set(-90);f.clock.advance(2000);assert.deepEqual(f.sent,[-150]);
  f.wheel.scroll(-999);f.clock.advance(1000);assert.deepEqual(f.sent,[-150,0]);
});

async function panelFixture(){
  const {mountStacking}=await import('../stack-panel.mjs');
  const elements=[],wires=[],clock=fakeClock();
  const doc={createElement(tag){
    const attributes={},listeners=new Map(),classes=new Set();
    const e={tag,ownerDocument:doc,children:[],dataset:{},value:'',textContent:'',isConnected:true,hidden:false,
      append(...items){this.children.push(...items);},setAttribute(k,v){attributes[k]=v;},
      classList:{toggle(k,on){if(on)classes.add(k);else classes.delete(k);},remove:k=>classes.delete(k)},
      addEventListener(k,fn){listeners.set(k,fn);},emit(k,event){listeners.get(k)?.(event);}};
    elements.push(e);return e;
  }};
  const root=doc.createElement('div'),panel=mountStacking({root,canSend:()=>true,
    send:async wire=>{wires.push(wire);return true;},schedule:clock.schedule,cancel:clock.cancel});
  const dial=elements.find(e=>e.className==='height-wheel');
  return {panel,clock,elements,wires,root,scroll:()=>dial.emit('wheel',{deltaY:100,preventDefault(){}})};
}
test('real stacking panel sends height alone and cancels wheel work on disconnect or fault',async()=>{
  const f=await panelFixture();f.scroll();f.clock.advance(1000);
  assert.deepEqual(f.wires,['grab stack lower -101']);
  f.scroll();f.panel.connection(false);f.clock.advance(1500);assert.equal(f.wires.length,1);
  f.panel.connection(true);f.scroll();f.panel.receive({state:'error',reason:'feedback_stale'});
  f.clock.advance(1500);assert.equal(f.wires.length,1);
});
test('panel exit never sends an old height and release/lift remain separate mouse actions',async()=>{
  const f=await panelFixture();f.scroll();f.root.hidden=true;f.clock.advance(1000);assert.deepEqual(f.wires,[]);
  f.root.hidden=false;
  f.elements.find(e=>e.textContent==='单独松爪').onclick();
  assert.deepEqual(f.wires,['grab stack release']);
  f.elements.find(e=>e.textContent==='提起到最高').onclick();
  assert.deepEqual(f.wires,['grab stack release','grab stack lift']);
});
