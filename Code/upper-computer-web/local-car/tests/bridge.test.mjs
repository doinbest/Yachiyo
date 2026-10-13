import test from 'node:test';
test('single ID probes remain available through the browser stop latch',()=>{
  for(const wire of ['bus test 1','bus test 4 10','bus test 8 100','bus test status','bus test stop','bus read 5 current','bus read 5 voltage','bus read 5 error','bus read 5 home','bus read 5 driver'])
    assert.equal(readOnlyCommand(wire),true,wire);
  for(const wire of ['bus test 0','bus test 9','bus test 4 0','bus test 4 101','bus test 4 1 extra','bus read 5 write','bus read 9 current','bus read 5 home 1'])
    assert.equal(readOnlyCommand(wire),false,wire);
});
import assert from 'node:assert/strict';
import {BridgeLink,readOnlyCommand,scopedStopCommand} from '../bridge.mjs';
test('mission pause and terminal stop both use priority cancellation',()=>{
 for(const wire of ['grab mission pause','grab mission stop'])assert.equal(scopedStopCommand(wire),true);
 for(const wire of ['grab mission continue','grab mission stop extra'])assert.equal(scopedStopCommand(wire),false);
});
test('F407 radar host reads and scoped stops keep the global stop latch',()=>{
  for(const wire of ['radar host status fast','radar host bundle 5 all','radar host bundle 5 0,2,4'])assert.equal(readOnlyCommand(wire),true,wire);
  for(const wire of ['radar host scan fast','radar host live start fast','radar host bundle 5 -1','radar host bundle 5 0,','radar host status fast extra'])assert.equal(readOnlyCommand(wire),false,wire);
  for(const wire of ['radar host status','radar host result 4294967295','radar host get active','radar host get draft','radar host live status','radar host fetch map:4294967295 65535'])assert.equal(readOnlyCommand(wire),true,wire);
  for(const wire of ['radar host stop','radar host live stop'])assert.equal(scopedStopCommand(wire),true,wire);
  for(const wire of ['radar host start','radar host scan','radar host live start','radar host draft reset','radar host set threshold 1','radar host apply 1'])assert.equal(readOnlyCommand(wire),false,wire);
});

test('radar result acknowledgement is allowed while stopped and cannot start a scan',()=>{
  assert.equal(readOnlyCommand('radar host ack 4294967295'),true);
  for(const wire of ['radar host ack','radar host ack 1 extra','radar host ack -1','radar host result -1','radar host result 1 extra','radar host live start'])assert.equal(readOnlyCommand(wire),false);
});
test('live radar status is readable and live stop bypasses a stop latch without enabling motion',async()=>{
  assert.equal(readOnlyCommand('radar live status'),true);
  assert.equal(scopedStopCommand('radar live stop'),true);
  for(const command of ['radar live start lower','radar live start upper','radar live status extra'])assert.equal(readOnlyCommand(command),false);
  const f=fixture();f.setStatus({stop_latched:true});await f.bridge.attach();
  for(const command of ['radar live status','radar live stop']){
    const sent=f.bridge.send(command);await new Promise(setImmediate);
    const count=f.requests.filter(r=>r.url==='/api/send').length;
    f.bridge.deliver({id:15+count,kind:'tx',request_id:count,text:command});await sent;
  }
  assert.equal(f.bridge.stopLatched,true);
  await assert.rejects(f.bridge.send('radar live start lower'),/停止/);
});
test('raw radar downloads and debug status remain readable while scans stay explicit',async()=>{
  for(const wire of ['radar debug status','radar fetch raw 1 0','radar fetch evidence 17 23'])assert.equal(readOnlyCommand(wire),true);
  for(const wire of ['radar debug scan lower','radar debug scan upper','radar debug scan auto','radar debug status extra'])assert.equal(readOnlyCommand(wire),false);
  const f=fixture();f.setStatus({stop_latched:true});await f.bridge.attach();
  const sent=f.bridge.send('radar fetch raw 1 0');await new Promise(setImmediate);
  f.bridge.deliver({id:16,kind:'tx',request_id:1,text:'radar fetch raw 1 0'});await sent;
  assert.equal(f.bridge.stopLatched,true);
  await assert.rejects(f.bridge.send('radar debug scan lower'),/停止/);
});
test('communication diagnostics remain read-only while stopped',()=>{
  assert.equal(readOnlyCommand('radar start status'),true);
  for(const wire of ['radar start upper','radar start lower','radar scan auto'])assert.equal(readOnlyCommand(wire),false);
  assert.equal(readOnlyCommand('bus status'),true);
  assert.equal(readOnlyCommand('bus recover'),false);
  assert.equal(readOnlyCommand('grab status'),true);
  assert.equal(readOnlyCommand('grab get retarget_ok'),true);
  assert.equal(readOnlyCommand('grab get z_grab extra'),false);
  assert.equal(readOnlyCommand('grab start fixed'),false);
  assert.equal(readOnlyCommand('grab stop'),false);
  assert.equal(readOnlyCommand('console status'),true);
  assert.equal(readOnlyCommand('chassis snapshot'),true);
  assert.equal(readOnlyCommand('console status extra'),false);
});

test('bus recovery is explicit under stop latch and never releases the latch',async()=>{
  const f=fixture();f.setStatus({stop_latched:true});await f.bridge.attach();
  const pending=f.bridge.send('bus recover');await new Promise(setImmediate);
  assert.equal(f.requests.at(-1).body.command,'bus recover');
  f.bridge.deliver({id:16,kind:'tx',request_id:1,text:'bus recover'});await pending;
  assert.equal(f.bridge.stopLatched,true);
  await assert.rejects(f.bridge.send('pos z -80'),/停止/);
});

function fixture(){
  const requests=[],received=[],events=[],states=[];let status={token:'session',connected:true,last_event_id:15,stop_latched:false,config:{profile:'',units_per_rev:null,directions_confirmed:false}},id=0;
  const bridge=new BridgeLink({fetch:async(url,options)=>{
    const body=options.body&&JSON.parse(options.body);requests.push({url,options,body});
    const data=url==='/api/status'?status:url==='/api/send'?{queued:true,request_id:++id}:url==='/api/config'?{config:body}:{};
    return {ok:true,json:async()=>data};
  },receive:x=>received.push(x),event:x=>events.push(x),state:x=>states.push(x)});
  return {bridge,requests,received,events,states,setStatus:s=>{status={...status,...s};}};
}

test('requests carry stop epoch and scoped cancellation leaves new clicks and reads usable',async()=>{
  const f=fixture();f.setStatus({command_epoch:7,stop:{token:111,status:'confirmed'}});await f.bridge.attach();
  const started=f.bridge.send('grab track begin 3 50');const cancelled=assert.rejects(started,/停止|取消/);
  await new Promise(setImmediate);assert.equal(f.requests.at(-1).body.command_epoch,7);
  const read=f.bridge.send('grab track status');await new Promise(setImmediate);
  f.bridge.deliver({kind:'command_epoch',command_epoch:8});await cancelled;
  f.bridge.deliver({kind:'tx',request_id:2,text:'grab track status'});await read;
  const next=f.bridge.send('grab track begin 3 50');await new Promise(setImmediate);
  assert.equal(f.requests.at(-1).body.command_epoch,8);
  f.bridge.deliver({kind:'tx',request_id:3,text:'grab track begin 3 50'});await next;
  await f.bridge.resume();assert.equal(f.requests.at(-1).body.stop_token,111);
});

test('a delayed old stop response cannot replace the newer stop token',async()=>{
  const f=fixture();f.setStatus({command_epoch:3,stop:{token:111}});await f.bridge.attach();let finish;
  f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({token:222,command_epoch:4,status:'confirmed'})});});
  const stopped=f.bridge.stop();
  f.bridge.deliver({kind:'stop',command_epoch:5,token:333,latched:true,status:'confirmed'});
  finish();await stopped;
  f.bridge.fetcher=async(url,options)=>{f.requests.push({url,body:JSON.parse(options.body)});return {ok:true,json:async()=>({})};};
  await f.bridge.resume();assert.equal(f.requests.at(-1).body.stop_token,333);
});

test('a scoped stop epoch does not discard the current global stop token',async()=>{
  const f=fixture();f.setStatus({command_epoch:3,stop:{token:111}});await f.bridge.attach();let finish;
  f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({token:222,command_epoch:4,status:'confirmed'})});});
  const stopped=f.bridge.stop();
  f.bridge.deliver({kind:'command_epoch',command_epoch:5});
  finish();await stopped;
  f.bridge.fetcher=async(url,options)=>{f.requests.push({url,body:JSON.parse(options.body)});return {ok:true,json:async()=>({})};};
  await f.bridge.resume();assert.equal(f.requests.at(-1).body.stop_token,222);
  assert.equal(f.bridge.commandEpoch,5);
});

for(const action of ['send','stop','resume'])test(`late ${action} response cannot poison a restarted service session`,async()=>{
  const f=fixture();f.setStatus({command_epoch:7,stop:{token:111}});await f.bridge.attach();
  const original=f.bridge.fetcher;let finish;
  f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({queued:true,request_id:90,command_epoch:7,token:111,status:'confirmed'})});});
  const old=action==='send'?f.bridge.send('pos z -90'):f.bridge[action]();
  const rejected=old.then(()=>false,()=>true);
  f.setStatus({token:'new-service',command_epoch:0,stop_latched:false,stop:{token:0,status:'idle'}});
  f.bridge.fetcher=original;await f.bridge.attach();
  finish();await new Promise(setImmediate);f.bridge.deliver({kind:'tx',request_id:90,text:'old response'});
  assert.equal(await rejected,true);
  assert.equal(f.bridge.commandEpoch,0);assert.equal(f.bridge.stopToken,0);assert.equal(f.bridge.stopLatched,false);
  const next=f.bridge.send('pos z -90');await new Promise(setImmediate);
  assert.equal(f.requests.at(-1).body.command_epoch,0);
  f.bridge.deliver({kind:'tx',request_id:1,text:'pos z -90'});await next;
});

test('a delayed resume cannot clear a newer stop adopted from status on the same service',async()=>{
  const f=fixture();f.setStatus({command_epoch:4,stop_latched:true,stop:{token:111,status:'confirmed'}});await f.bridge.attach();
  const original=f.bridge.fetcher;let finish;
  f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({command_epoch:4,stop_latched:false})});});
  const resumed=f.bridge.resume();
  f.setStatus({command_epoch:5,stop_latched:true,stop:{token:333,status:'confirmed'}});
  f.bridge.fetcher=original;await f.bridge.attach();
  finish();await resumed;
  assert.equal(f.bridge.stopLatched,true);assert.equal(f.bridge.stopToken,333);assert.equal(f.bridge.commandEpoch,5);
});

for(const phase of ['headers','body'])test(`HTTP ${phase} stall has a finite deadline and aborts the request`,async()=>{
  let signal,calls=0;
  const never=new Promise(()=>{});
  const bridge=new BridgeLink({requestTimeoutMs:20,fetch:async(_url,options)=>{
    calls++;signal=options.signal;
    return phase==='headers'?never:{ok:true,json:()=>never};
  }});
  let deadline;
  try {
    const result=await Promise.race([
      bridge.api('send',{command:'info'}).then(()=> 'resolved',e=>e.message),
      new Promise(resolve=>{deadline=setTimeout(()=>resolve('hung'),500);})
    ]);
    assert.match(result,/超时/);assert.equal(signal.aborted,true);assert.equal(calls,1);
  } finally {clearTimeout(deadline);}
});

test('parent abort settles an API request even when the transport ignores abort',async()=>{
  const parent=new AbortController();let signal;
  const bridge=new BridgeLink({requestTimeoutMs:1000,fetch:async(_url,options)=>{
    signal=options.signal;return new Promise(()=>{});
  }});
  const request=bridge.api('events',undefined,parent.signal);
  parent.abort();let deadline;
  try {
    const result=await Promise.race([request.then(()=> 'resolved',e=>e.name),
      new Promise(resolve=>{deadline=setTimeout(()=>resolve('hung'),100);})]);
    assert.equal(result,'AbortError');assert.equal(signal.aborted,true);
  } finally {clearTimeout(deadline);}
});

test('offline C planning keeps its build/run budget but still has a finite deadline',async t=>{
  t.mock.timers.enable({apis:['setTimeout']});let settled=false;
  const bridge=new BridgeLink({fetch:async()=>new Promise(()=>{})});
  const request=bridge.api('radar/plan',{}).catch(e=>{settled=true;return e.message;});
  t.mock.timers.tick(12001);await Promise.resolve();await Promise.resolve();
  assert.equal(settled,false);
  t.mock.timers.tick(38000);assert.match(await request,/超时/);
});

test('grasp stop remains sendable under stop latch while starts remain blocked',async()=>{
  const f=fixture();f.setStatus({stop_latched:true});await f.bridge.attach();
  await assert.rejects(f.bridge.send('grab start pick'),/停止/);
  const pending=f.bridge.send('grab stop');await new Promise(setImmediate);
  assert.equal(f.requests.at(-1).body.command,'grab stop');
  f.bridge.deliver({id:16,kind:'tx',request_id:1,text:'grab stop'});await pending;
});

test('stable downloads and result ACK stay readable under the stop latch',async()=>{
  const f=fixture();f.setStatus({stop_latched:true});await f.bridge.attach();
  const commands=['radar fetch stable 3 1','radar host fetch stable:3 1','radar ack 8','radar host ack 8'];
  for(const [index,command]of commands.entries()){
    assert.equal(readOnlyCommand(command),true);const pending=f.bridge.send(command);await new Promise(setImmediate);
    f.bridge.deliver({id:16+index,kind:'tx',request_id:index+1,text:command});await pending;
  }
  assert.equal(f.bridge.stopLatched,true);await assert.rejects(f.bridge.send('radar live map start lower'),/停止/);
  assert.deepEqual(f.requests.filter(r=>r.url==='/api/send').map(r=>r.body.command),commands);
});

test('refresh attachment reads status only, skips old cursor and never sends setup or motion',async()=>{
  const f=fixture();await f.bridge.attach();assert.equal(f.bridge.connected,true);assert.equal(f.bridge.cursor,15);
  assert.deepEqual(f.requests.map(r=>r.url),['/api/status']);assert.deepEqual(f.received,[]);
  await f.bridge.attach();assert.deepEqual(f.states,['connected']);assert.ok(f.requests.every(r=>r.options.method==='GET'));
});

test('send waits for actual TX event, keeps auth token and shares RX once',async()=>{
  const f=fixture();await f.bridge.attach();let resolved=false;
  const sent=f.bridge.send('chassis task').then(()=>{resolved=true;});await new Promise(setImmediate);assert.equal(resolved,false);
  const request=f.requests.at(-1);assert.equal(request.options.headers['X-Console-Token'],'session');assert.deepEqual(request.body,{command:'chassis task',command_epoch:0});
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
  const connect=f.bridge.connect('COM7',115200);
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

test('connect requires explicit selection and sends the chosen port instead of a fixed COM number',async()=>{
  const f=fixture();
  await assert.rejects(f.bridge.connect('',115200),/选择/);
  assert.equal(f.requests.length,0);
  await f.bridge.connect('COM7',115200);
  assert.deepEqual(f.requests.at(-1).body,{port:'COM7',baudrate:115200});
  assert.equal(f.bridge.port,'COM7');
  f.setStatus({port:'COM12'});await f.bridge.attach();
  assert.equal(f.bridge.port,'COM12');
  f.bridge.deliver({id:100,kind:'connection',connected:true,port:'COM8'});
  assert.equal(f.bridge.port,'COM8');
});

test('connection event before HTTP completion emits connected once',async()=>{
  const f=fixture();f.setStatus({connected:false});await f.bridge.attach();let finish;
  f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({})});});
  const pending=f.bridge.connect('COM7');f.bridge.deliver({kind:'connection',connected:true,port:'COM7'});finish();await pending;
  assert.equal(f.states.filter(x=>x==='connected').length,1);
});
test('disconnect immediately blocks commands on this and other attached pages',async()=>{
  const f=fixture();await f.bridge.attach();let finish;
  f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({})});});
  const pending=f.bridge.disconnect();assert.equal(f.states.at(-1),'disconnecting');
  await assert.rejects(f.bridge.send('info'),/断开/);finish();await pending;assert.equal(f.states.at(-1),'disconnected');
  const other=fixture();other.setStatus({disconnecting:true});await other.bridge.attach();
  assert.equal(other.states.at(-1),'disconnecting');await assert.rejects(other.bridge.send('info'),/断开/);
});
test('send HTTP response from an older connection cannot register in the new session',async()=>{
  const f=fixture();await f.bridge.attach();let finish;
  f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({queued:true,request_id:10})});});
  const pending=f.bridge.send('info');const rejected=assert.rejects(pending,/连接已变化/);
  f.bridge.deliver({kind:'connection',connected:false});f.bridge.deliver({kind:'connection',connected:true});
  finish();await rejected;assert.equal(f.bridge.pending.size,0);
});

test('a failed connect event restores disconnected even when already disconnected',async()=>{
  const f=fixture();f.setStatus({connected:false});await f.bridge.attach();let finish;
  f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:false,json:async()=>({error:'open failed'})});});
  const p=f.bridge.connect('COM7');const rejected=assert.rejects(p,/open failed/);
  f.bridge.deliver({kind:'connection',connected:false});assert.equal(f.states.at(-1),'disconnected');
  finish();await rejected;
});

test('a delayed duplicate connected event does not invalidate a same-session send',async()=>{
  const f=fixture();f.setStatus({connected:false});await f.bridge.attach();await f.bridge.connect('COM7');
  let finish;f.bridge.fetcher=()=>new Promise(resolve=>{finish=()=>resolve({ok:true,json:async()=>({queued:true,request_id:1})});});
  const pending=f.bridge.send('info');
  f.bridge.deliver({kind:'connection',connected:true,port:'COM7'});
  f.bridge.deliver({kind:'tx',request_id:1,text:'info'});
  finish();assert.equal((await pending).text,'info');
});
