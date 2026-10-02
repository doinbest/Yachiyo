import {buildCommand} from './protocol.mjs?v=grab-preset-20260930';

const decimal='[+-]?(?:\\d+\\.?\\d*|\\.\\d+)(?:[eE][+-]?\\d+)?';
const grabReply=new RegExp(`^OK grab config key=([a-z][a-z0-9_]*) value=(unset|${decimal})$`);

export function parseGrabConfig(line){
  const match=typeof line==='string'&&line.trim().match(grabReply);
  if(!match)return null;
  const value=match[2]==='unset'?null:Number(match[2]);
  return value===null||Number.isFinite(value)?{key:match[1],value}:null;
}

export function parseArmConfig(line){
  const match=typeof line==='string'&&line.trim().match(/^OK config (base|z|x) rpm=(\d+) acc=(\d+) limit_pulses=(unlimited|\d+)$/);
  if(!match)return null;
  const rpm=Number(match[2]),acc=Number(match[3]),limit=match[4]==='unlimited'?0:Number(match[4]);
  return rpm>=1&&rpm<=3000&&acc>=1&&acc<=255&&limit>=0&&limit<=3200?
    {axis:match[1],rpm,acc,limit}:null;
}

// Firmware prints grab values with %.6g; preserve an exact zero check for 0=disabled settings.
const sameNumber=(actual,expected)=>actual!==null&&
  (expected===0?actual===0:Math.abs(actual-expected)<=Math.abs(expected)*0.000006);

/** One explicit request at a time; TX alone never counts as a device confirmation. */
export function createParameterExchange({send,timeoutMs=6000}){
  let active=false,pending=null,revision=0;
  function clearPending(error){
    if(!pending)return;
    const item=pending;pending=null;clearTimeout(item.timer);
    if(error)item.reject(error);
  }
  function cancel(reason='参数读取已取消'){
    revision++;
    clearPending(new Error(reason));
  }
  function receive(line){
    if(!pending)return false;
    const result=pending.kind==='grab'?parseGrabConfig(line):parseArmConfig(line);
    if(!result||result[pending.kind==='grab'?'key':'axis']!==pending.id)return false;
    const item=pending;pending=null;clearTimeout(item.timer);item.resolve(result);
    return true;
  }
  async function run(work){
    if(active)throw new Error('上一项参数仍在等待设备读回。');
    active=true;const current=revision;
    try{return await work(current);}finally{active=false;}
  }
  function check(current){if(current!==revision)throw new Error('连接或操作状态已变化，请重新读取。');}
  function read(wire,kind,id,current){
    check(current);
    return new Promise((resolve,reject)=>{
      const item={kind,id,resolve,reject,timer:null};
      item.timer=setTimeout(()=>{
        if(pending===item){pending=null;reject(new Error('设备读回超时，请查看串口日志。'));}
      },timeoutMs);
      pending=item;
      Promise.resolve().then(()=>send(wire)).then(written=>{
        if(written!==true&&pending===item)clearPending(new Error('串口写出未确认。'));
      },error=>{if(pending===item)clearPending(error);});
    });
  }
  const grabRead=(key,current)=>read(buildCommand(`grab-get-${key}`),'grab',key,current);
  const armRead=(axis,current)=>read(buildCommand('config-get',{axis}),'arm',axis,current);
  return {
    get busy(){return active;},receive,cancel,
    readGrab:key=>run(current=>grabRead(key,current)),
    readArm:axis=>run(current=>armRead(axis,current)),
    setGrab:(key,value)=>run(async current=>{
      const wire=buildCommand(`grab-set-${key}`,{value});
      if(await send(wire)!==true)throw new Error('串口写出未确认。');
      check(current);
      const result=await grabRead(key,current);
      return {...result,confirmed:sameNumber(result.value,Number(value))};
    }),
    setArm:(axis,rpm,acc,limit)=>run(async current=>{
      const wire=buildCommand('config-set',{axis,rpm,acc,limit});
      if(await send(wire)!==true)throw new Error('串口写出未确认。');
      check(current);
      const result=await armRead(axis,current);
      return {...result,confirmed:result.rpm===Number(rpm)&&result.acc===Number(acc)&&result.limit===Number(limit)};
    })
  };
}
