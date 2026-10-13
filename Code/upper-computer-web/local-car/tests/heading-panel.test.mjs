import test from 'node:test';
import assert from 'node:assert/strict';
import {mountHeadingTest,buildHeadingTestCommand,buildHeadingProbeCommand} from '../heading-panel.mjs';
import {parsePageRoute} from '../page-navigation.mjs';
function fixture(){
 const elements=[],wires=[];const doc={createElement(tag){const e={tag,ownerDocument:doc,children:[],value:'',textContent:'',append(...a){this.children.push(...a);},addEventListener(k,fn){this[k]=fn;}};elements.push(e);return e;}};
 const panel=mountHeadingTest({root:doc.createElement('div'),send:async w=>{wires.push(w);return true;},canSend:()=>true});
 return {panel,wires,get:id=>elements.find(e=>e.id===id)};
}
test('standalone entry and command select station without starting a match',()=>{
 assert.equal(parsePageRoute('#vision/heading').tab,'heading');
 assert.equal(buildHeadingTestCommand('proc',5),'grab heading proc 5');
 assert.equal(buildHeadingTestCommand('temp',2),'grab heading temp 2');
 for(const [s,v] of [['raw',5],['proc',''],['proc',0],['proc',11]])assert.throws(()=>buildHeadingTestCommand(s,v));
});
test('preparation is shared; mounting, drafts and reconnect send no commands',async()=>{
 const f=fixture();f.panel.connection(true);assert.deepEqual(f.wires,[]);
 await f.get('heading-prepare').onclick();assert.deepEqual(f.wires,['grab pose work']);
 f.panel.receiveGrab({state:'complete',result:'work_ready'});
 assert.match(f.get('heading-pose').textContent,/到位/);
 f.get('heading-station').value='temp';f.get('heading-station').change();assert.equal(f.wires.length,1);
 await f.get('heading-start').onclick();assert.equal(f.wires.at(-1),'grab heading temp 5');
 f.panel.receive({mode:'heading',state:'done',station:3,heading_result:'heading_uncalibrated',heading_image:'0.50',heading_z:'0.00',heading_before:'90.00',map_yaw:'90.00'});
 assert.match(f.get('heading-result').textContent,/未标定/);assert.match(f.get('heading-result').textContent,/0.50/);assert.match(f.get('heading-result').textContent,/观察Z 0.00 mm/);
 f.panel.connection(false);f.panel.connection(true);assert.equal(f.wires.length,2);
 assert.match(f.get('heading-result').textContent,/历史/);
 await f.get('heading-stop').onclick();assert.equal(f.wires.at(-1),'grab stop');
});
test('active test blocks repeat start; cancellation waits for device confirmation',()=>{
 const f=fixture();f.panel.connection(true);f.panel.receive({mode:'heading',state:'station_heading',heading_result:'heading_observing'});
 assert.equal(f.get('heading-start').disabled,true);assert.equal(f.get('heading-stop').disabled,false);
 f.panel.receive({mode:'heading',state:'cancel_wait',reason:'cancel_stop_unconfirmed'});assert.equal(f.get('heading-start').disabled,true);
 f.panel.receive({mode:'heading',state:'cancelled'});assert.equal(f.get('heading-start').disabled,false);
});

test('signed trial is explicit and shows before/after data without writing calibration',async()=>{
 assert.equal(buildHeadingProbeCommand('proc',1,5),'grab heading angle proc 1 5');
 assert.equal(buildHeadingProbeCommand('temp',-1,5),'grab heading angle temp -1 5');
 for(const [d,v] of [[0,5],[3,5],[1,2]])assert.throws(()=>buildHeadingProbeCommand('proc',d,v));
 const f=fixture();f.panel.connection(true);assert.deepEqual(f.wires,[]);
 assert.equal(f.get('heading-polarity').value,'-1');
 assert.equal(buildHeadingProbeCommand('proc'),'grab heading angle proc -1 5');
 await f.get('heading-probe-start').onclick();assert.equal(f.wires.at(-1),'grab heading angle proc -1 5');
 f.panel.receive({mode:'heading',state:'heading_probe_moving',probe:'1',reason:'heading_probe_moving'});
 assert.equal(f.get('heading-probe-start').disabled,true);assert.equal(f.get('heading-stop').disabled,false);
 f.panel.receive({mode:'heading',state:'done',probe:'1',reason:'heading_probe_done',probe_before:'9.51',probe_after:'0.10',probe_delta:'9.51',probe_yaw_delta:'9.501'});
 assert.match(f.get('heading-result').textContent,/9.51.*0.10/);assert.match(f.get('heading-result').textContent,/9.501/);
 assert.equal(f.wires.length,1);
});
