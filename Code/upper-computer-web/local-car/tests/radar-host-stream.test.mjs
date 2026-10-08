import assert from 'node:assert/strict';
import test from 'node:test';
import {RadarHostClient} from '../radar-host-client.mjs';
import {RadarHostModel} from '../radar-host-model.mjs';

const identity={v:1,profile:'f1_host',session:41,config:1,scan:2,map:3,plan:4};
const records={map:['s,1620,1411,1409','m,00000400,00000000,00050540','p,1,0,0',...Array.from({length:25},(_,i)=>`c,${i},${i===10?1:0}`)],
  path:['w,230,230,545,1,1,1,0','w,775,230,0,1,2,255,255']};
const wire=frame=>'@RADAR '+JSON.stringify({...identity,...frame});
const pages=(kind,id=identity)=>records[kind].map((data,page)=>({...id,k:`host_${kind}`,page,pages:records[kind].length,offset:page,total:records[kind].length,data}));
const end=(id=identity)=>({...id,k:'host_result_end',mp:28,pp:2});
const status=(id=identity)=>wire({...id,k:'host_status',ss:4,a:0,l:1,ls:2,reason:'complete'});
const pause=ms=>new Promise(resolve=>setTimeout(resolve,ms));
async function settled(f,plan=4){for(let i=0;i<100&&!f.commands.includes(`radar host ack ${plan}`);i++)await pause(2);}

function fixture({drop=[],omitEnd=false,spacing=0,corrupt={}}={}){
  const commands=[],errors=[],model=new RadarHostModel();let client,sending=false;
  const send=async command=>{
    // Mirrors the actual shared console: a second simultaneous send is refused.
    assert.equal(sending,false,'overlapping writes');sending=true;commands.push(command);
    await pause(1);sending=false;
    if(command==='radar host result 4')void (async()=>{
      for(const frame of [...pages('map'),...pages('path'),...(omitEnd?[]:[end()])]){
        if(spacing)await pause(spacing);
        const key=`${frame.k}:${frame.page}`;
        if(!drop.includes(key))client.receive(wire(corrupt[key]?{...frame,data:corrupt[key]}:frame));
      }
    })();
    const match=/^radar host fetch (map|path):(\d+) (\d+)$/.exec(command);
    if(match)queueMicrotask(()=>client.receive(wire(pages(match[1])[Number(match[3])])));
    return {};
  };
  client=new RadarHostClient({send,model,onError:error=>errors.push(error.message),timeoutMs:20});
  client.connection(true);return {client,model,commands,errors};
}

test('one request streams a frozen map/path pair without parallel writes or automatic cloud reads',async()=>{
  const f=fixture();f.client.receive(status());await settled(f);
  assert.deepEqual(f.commands,['radar host result 4','radar host ack 4']);
  assert.equal(f.model.snapshots.map.counts[10],1);assert.equal(f.model.snapshots.path.points.length,2);
  assert.deepEqual(f.errors,[]);
});

test('end manifest repairs only missing first, middle and last pages before ACK',async()=>{
  const f=fixture({drop:['host_map:0','host_map:7','host_path:1']});f.client.receive(status());await settled(f);
  assert.deepEqual(f.commands,['radar host result 4','radar host fetch map:3 0','radar host fetch map:3 7','radar host fetch path:4 1','radar host ack 4']);
  assert.equal(f.model.snapshots.path.points.length,2);assert.deepEqual(f.errors,[]);
});

test('an entirely lost map is repaired using the manifest, not by restarting the stream',async()=>{
  const f=fixture({drop:pages('map').map(p=>`host_map:${p.page}`)});f.client.receive(status());await settled(f);
  assert.equal(f.commands.filter(c=>c==='radar host result 4').length,1);
  assert.equal(f.commands.filter(c=>c.startsWith('radar host fetch map:')).length,28);
  assert.equal(f.commands.at(-1),'radar host ack 4');assert.deepEqual(f.errors,[]);
});

test('complete pair needs no end marker and progressing streams do not hit a total-duration timeout',async()=>{
  // Windows timers can be rounded to ~16ms: use a gap well below the idle
  // timeout, while the whole 30-page stream lasts much longer than it.
  const f=fixture({omitEnd:true,spacing:20});f.client.timeoutMs=80;f.client.receive(status());await settled(f);
  assert.deepEqual(f.commands,['radar host result 4','radar host ack 4']);assert.deepEqual(f.errors,[]);
});

test('lost end marker and lost last page fall back to repairing that page',async()=>{
  const f=fixture({omitEnd:true,drop:['host_path:1']});f.client.receive(status());await settled(f);
  assert.deepEqual(f.commands,['radar host result 4','radar host fetch path:4 1','radar host ack 4']);
  assert.deepEqual(f.errors,[]);
});

test('repeated ready statuses cannot start a second result request while a stream is active',async()=>{
  const f=fixture({spacing:2});f.client.receive(status());f.client.receive(status());f.client.receive(status());await settled(f);
  assert.deepEqual(f.commands,['radar host result 4','radar host ack 4']);
});

test('a valid JSON path page with missing record bytes is repaired without losing good pages',async()=>{
  // COM14 circle 46 had "w,230,...;0,425,1,2,1,1;w,1200,...":
  // JSON still parsed, so the old assembler only discovered it at completion.
  const f=fixture({corrupt:{'host_path:0':'w,230,230,545,1,1,1,0;0,425,1,2,1,1'}});
  f.client.receive(status());await settled(f);
  assert.deepEqual(f.commands,['radar host result 4','radar host fetch path:4 0','radar host ack 4']);
  assert.deepEqual(f.model.snapshots.path.points[0],[230,230,545,1,1,1,0]);
  assert.equal(f.model.snapshots.path.points.length,2);assert.equal(f.client.lastError,null);
  assert.deepEqual(f.errors,[]);
});
