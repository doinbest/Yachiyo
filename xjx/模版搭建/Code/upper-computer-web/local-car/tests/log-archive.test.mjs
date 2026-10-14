import test from 'node:test';
import assert from 'node:assert/strict';
import {mountLogArchive,downloadCurrentConsole} from '../log-archive.mjs';

test('archive browsing and full TXT export use disk API, with no serial commands',async()=>{
  const old=globalThis.document,requests=[],clicks=[];
  class Element{
    constructor(tag){this.tag=tag;this.children=[];this.textContent='';this.value='';}
    append(...items){this.children.push(...items);}
    replaceChildren(...items){this.children=items;}
    setAttribute(){}
    click(){clicks.push(this.href);}
  }
  globalThis.document={createElement:tag=>new Element(tag)};
  const name='20261009-190539-ffa902';
  const request=async url=>{requests.push(url);return url==='/api/logs'?{logs:[{name,rx_bytes:75000}],current:name}:{sessions:[{index:0,records:600,window_ms:59900,expected_records:604,missing_count:4,corrupt_rows:2,closed:true,trace_complete:false,parameter_keys:13,source_type:'pixels',valid_frames:599,invalid_frames:1,unmatched_frames:0,receive_hz:60,device_dropped:2,motor_records:50}]};};
  try{
    const root=new Element('div');mountLogArchive({root,request});
    const all=()=>{const walk=e=>[e,...e.children.flatMap(walk)];return walk(root);};
    assert.deepEqual(requests,[]);
    await all().find(e=>e.textContent==='读取磁盘记录').onclick();
    assert.equal(requests.length,2);
    assert.ok(all().some(e=>e.textContent.includes('缺 4 条')));
    assert.ok(all().some(e=>e.textContent.includes('漏检回复')&&e.textContent.includes('60.0 Hz')&&e.textContent.includes('速度诊断 50 条')));
    all().find(e=>e.textContent==='导出整段追踪 TXT').onclick();
    assert.ok(clicks.at(-1).includes('kind=tracking&segment=0'));
    all().find(e=>e.textContent==='导出完整接收 TXT').onclick();
    assert.ok(clicks.at(-1).includes('kind=rx'));
    await downloadCurrentConsole(request);
    assert.ok(clicks.at(-1).includes('kind=console'));
  }finally{globalThis.document=old;}
});
