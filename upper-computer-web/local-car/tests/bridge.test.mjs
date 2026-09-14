import test from 'node:test';
import assert from 'node:assert/strict';
import {BridgeLink,readOnlyCommand} from '../bridge.mjs';

function fixture(){
  const requests=[],received=[],events=[],states=[];let status={token:'session',connected:true,last_event_id:15,stop_latched:false,config:{profile:'',units_per_rev:null,directions_confirmed:false}},id=0;
  const bridge=new BridgeLink({fetch:async(url,options)=>{
    const body=options.body&&JSON.parse(options.body);requests.push({url,options,body});
    const data=url==='/api/status'?status:url==='/api/send'?{queued:true,request_id:++id}:url==='/api/config'?{config:body}:{};
    return {ok:true,json:async()=>data};
  },receive:x=>received.push(x),event:x=>events.push(x),state:x=>states.push(x)});
  return {bridge,requests,received,events,states,setStatus:s=>{status={...status,...s};}};
}

test('refresh attachment reads status only, skips old cursor and never sends setup or motion',async()=>{
  const f=fixture();await f.bridge.attach();assert.equal(f.bridge.connected,true);assert.equal(f.bridge.cursor,15);
  assert.deepEqual(f.requests.map(r=>r.url),['/api/status']);assert.deepEqual(f.received,[]);
  await f.bridge.attach();assert.deepEqual(f.states,['connected']);assert.ok(f.requests.every(r=>r.options.method==='GET'));
});

test('send waits for actual TX event, keeps auth token and shares RX once',async()=>{
  const f=fixture();await f.bridge.attach();let resolved=false;
  const sent=f.bridge.send('chassis task').then(()=>{resolved=true;});await new Promise(setImmediate);assert.equal(resolved,false);
  const request=f.requests.at(-1);assert.equal(request.options.headers['X-Console-Token'],'session');assert.deepEqual(request.body,{command:'chassis task'});
  f.bridge.deliver({id:16,kind:'tx',request_id:1,text:'chassis task'});await sent;
  f.bridge.deliver({id:17,kind:'rx',text:'OK chassis task state=0'});assert.deepEqual(f.received,['OK chassis task state=0']);
  assert.equal(f.events.filter(e=>e.kind==='tx').length,1);
});

test('TX may precede HTTP response; cancelled queued commands reject without resending',async()=>{
  const f=fixture();await f.bridge.attach();f.bridge.deliver({id:16,kind:'tx',request_id:1,text:'info'});await f.bridge.send('info');
  const sent=f.bridge.send('chassis origin 2250 150 90');const rejection=assert.rejects(sent,/停止/);await new Promise(setImmediate);
  f.bridge.deliver({id:17,kind:'stop',latched:true,status:'requested'});await rejection;
  await assert.rejects(f.bridge.send('chassis route next'),/停止/);assert.equal(f.requests.filter(r=>r.url==='/api/send').length,2);
  await f.bridge.resume();assert.equal(f.bridge.stopLatched,false);assert.equal(f.requests.at(-1).url,'/api/resume');
});

test('disconnect cancels pending requests; reattachment does not replay previous commands',async()=>{
  const f=fixture();await f.bridge.attach();const sent=f.bridge.send('info');const rejection=assert.rejects(sent,/断开/);await new Promise(setImmediate);
  f.bridge.deliver({id:16,kind:'connection',connected:false});await rejection;
  f.setStatus({last_event_id:90});await f.bridge.attach();assert.equal(f.bridge.cursor,90);
  assert.equal(f.requests.filter(r=>r.url==='/api/send').length,1);
});

test('explicit configuration save persists only to host; stop is independent of ordinary queue',async()=>{
  const f=fixture();await f.bridge.attach();const config={profile:'none',units_per_rev:65536,directions_confirmed:true};
  assert.deepEqual(await f.bridge.saveConfig(config),config);assert.deepEqual(f.bridge.config,config);
  await f.bridge.stop();assert.equal(f.requests.at(-1).url,'/api/stop');assert.equal(f.bridge.stopLatched,true);
  assert.equal(f.requests.filter(r=>r.url==='/api/send').length,0);
  for(const wire of ['info','chassis task','chassis stop-status','imu status'])assert.equal(readOnlyCommand(wire),true);
  for(const wire of ['chassis origin 0 0 0','chassis route next','qr stream on'])assert.equal(readOnlyCommand(wire),false);
});

test('preparation owns setup sends and explicitly releases without issuing any motion',async()=>{
  const f=fixture();await f.bridge.attach();const owner=await f.bridge.acquirePreparation();
  assert.equal(f.requests.at(-1).url,'/api/preparation');assert.deepEqual(f.requests.at(-1).body,{active:true,owner});
  const sent=f.bridge.send('chassis units 65536',true);await new Promise(setImmediate);
  assert.equal(f.requests.at(-1).body.owner,owner);f.bridge.deliver({id:16,kind:'tx',request_id:1,text:'chassis units 65536'});await sent;
  await f.bridge.releasePreparation();assert.deepEqual(f.requests.at(-1).body,{active:false,owner});assert.equal(f.bridge.preparationOwner,null);
});

test('old HTTP replies cannot overwrite newer stop or connection events',async()=>{
  const f=fixture();await f.bridge.attach();let finish;
  f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({})});});
  const resume=f.bridge.resume();
  f.bridge.deliver({id:16,kind:'stop',latched:true,status:'confirmed',token:222});
  finish();await resume;assert.equal(f.bridge.stopLatched,true);
  const connect=f.bridge.connect(null,115200);
  f.bridge.deliver({id:17,kind:'connection',connected:false});
  finish();await connect;assert.equal(f.bridge.connected,false);assert.equal(f.states.at(-1),'disconnected');
  const disconnect=f.bridge.disconnect();
  f.bridge.deliver({id:18,kind:'connection',connected:true});
  finish();await disconnect;assert.equal(f.bridge.connected,true);
});

for(const restart of [false,true])test(`poll skips duplicates and ${restart?'server restart':'history gap'} cancels pending requests before reattachment`,async()=>{
  const f=fixture();await f.bridge.attach();const sent=f.bridge.send('info');const rejection=assert.rejects(sent,/过期/);await new Promise(setImmediate);
  let n=0;const original=f.bridge.fetcher;
  f.bridge.fetcher=async(url,options)=>{
    if(!url.startsWith('/api/events'))return original(url,options);
    n++;if(n===1)return {ok:true,json:async()=>({events:[{id:14,kind:'rx',text:'old'},{id:16,kind:'rx',text:'fresh'},{id:16,kind:'rx',text:'fresh'}]})};
    if(n===2)return {ok:true,json:async()=>({gap:!restart,last_event_id:restart?0:16,events:[]})};
    f.bridge.suspend();return {ok:true,json:async()=>({events:[]})};
  };
  f.bridge.running=true;const generation=++f.bridge.generation;await f.bridge.poll(generation);await rejection;
  assert.deepEqual(f.received,['fresh']);assert.deepEqual(f.states,['connected','disconnected','connected']);
  assert.equal(f.requests.filter(r=>r.url==='/api/send').length,1);
});
