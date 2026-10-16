import {reasonText} from './protocol.mjs';
const profileText={linear:'线性',cosine:'余弦'};

export function parseChassisTestResult(line){
  const match=String(line).trim().replace(/^(?:arm>\s*)+/,'').match(/^(OK|ERR) chassis (test_result|test) (.*)$/);
  if(!match)return null;
  const result={kind:match[2]};
  const fields=result.kind==='test'?match[3].split(';',1)[0].trim():match[3];
  for(const token of fields.split(/\s+/)){
    const field=token.match(/^([a-z_][a-z0-9_]*)=([^\s=]+)$/);
    if(!field||Object.hasOwn(result,field[1]))return null;
    result[field[1]]=field[2];
  }
  if(match[1]==='ERR')return result.kind==='test'&&/^[a-z_]+$/.test(result.reason??'')&&result.reason!=='accepted'?{...result,rejected:true}:null;
  if(!/^\d+$/.test(result.id??'')||!Number.isSafeInteger(Number(result.id))||!Object.hasOwn(profileText,result.profile))return null;
  result.id=Number(result.id);
  for(const key of ['accel_mm_s2','decel_mm_s2'])if(result[key]!==undefined){
    if(!/^\d+(?:\.\d+)?$/.test(result[key]))return null;
    result[key]=Number(result[key]);
    if(!Number.isFinite(result[key])||result[key]<0||result[key]>5000)return null;
  }
  for(const key of ['cosine_k','heading_kp','heading_kd'])if(result[key]!==undefined){
    if(!/^\d+(?:\.\d+)?$/.test(result[key]))return null;
    result[key]=Number(result[key]);
    if(!Number.isFinite(result[key])||result[key]<0||result[key]>(key==='cosine_k'?2000:1)||(key==='cosine_k'&&!Number.isInteger(result[key])))return null;
  }
  for(const key of ['curve_period_ms','control_period_ms'])if(result[key]!==undefined){
    if(!/^\d+$/.test(result[key]))return null;result[key]=Number(result[key]);
    if(result[key]<1||result[key]>100)return null;
  }
  if(result.kind==='test')return result.reason==='accepted'?result:null;
  for(const key of ['state','elapsed_ms','peak_mm_s','error_mm','stop_ms','stop_confirmed']){
    if(!/^\d+(?:\.\d+)?$/.test(result[key]??''))return null;
    result[key]=Number(result[key]);if(!Number.isFinite(result[key]))return null;
  }
  if(![0,1].includes(result.stop_confirmed))return null;
  return result;
}

export function mountChassisTestResult({root}){
  const make=(tag,text)=>{const e=root.ownerDocument.createElement(tag);if(text!==undefined)e.textContent=text;return e;};
  const card=make('section');card.className='chassis-test-result';card.setAttribute('aria-live','polite');
  const status=make('p','尚无单段测试结果 · 开始测试后自动查询，也可手动查看底盘状态');status.className='form-note';
  const metrics=make('dl');metrics.className='chassis-test-metrics';metrics.hidden=true;
  const values=['加减速','实际耗时','实测峰速','终点误差','停车确认'].map(label=>{const cell=make('div'),value=make('dd','—');cell.append(make('dt',label),value);metrics.append(cell);return value;});
  const tuning=make('p','');tuning.className='form-note';tuning.hidden=true;
  card.append(status,metrics,tuning);root.append(card);
  let connected=false,id=null,waitingAck=false,snapshot=null;
  const render=(historical=false)=>{
    metrics.hidden=tuning.hidden=!snapshot;
    if(!snapshot)return;
    const s=snapshot;
    tuning.textContent=s.heading_kp!==undefined&&s.heading_kd!==undefined?`设备采用航向 P=${s.heading_kp}，D=${s.heading_kd}`:'设备未回报航向P、D';
    if(s.curve_period_ms!==undefined&&s.control_period_ms!==undefined)tuning.textContent+=`；控制 ${s.control_period_ms} ms，曲线 ${s.curve_period_ms} ms`;
    status.textContent=`${historical?'历史读回':'设备读回'} · 测试 #${s.id} · ${s.state===3&&s.stop_confirmed?'已完成':s.state===4?'测试异常':s.state===2?'正在停车':'运行中'}`;
    [s.profile==='linear'&&s.accel_mm_s2>0?(s.decel_mm_s2>0?`线性 · 起步${s.accel_mm_s2} / 减速${s.decel_mm_s2} mm/s²`:`线性 · ${s.accel_mm_s2} mm/s²`):s.profile==='cosine'&&s.cosine_k>0?`余弦 · K${s.cosine_k} / ${s.cosine_k*2} ms`:profileText[s.profile],`${s.elapsed_ms} ms`,`${s.peak_mm_s.toFixed(1)} mm/s`,s.state===3&&s.stop_confirmed?`${s.error_mm.toFixed(1)} mm`:'待完成',s.stop_confirmed?`${s.stop_ms} ms`:'待确认'].forEach((text,i)=>{values[i].textContent=text;});
  };
  return {
    preparing(){id=null;waitingAck=false;if(snapshot)render(true);else status.textContent='本次尚无运动结果 · 尚未发送测试指令';},
    interrupted(){id=null;waitingAck=false;if(snapshot){render(true);status.textContent='停止前最后读回 · 下方缓存不代表当前状态，停车确认见上方';}else status.textContent='本次已取消 · 尚无运动结果';},
    requested(wire){
      if(/^chassis test /.test(wire)){id=null;waitingAck=true;snapshot=null;metrics.hidden=tuning.hidden=true;status.textContent='请求单段测试 · 等待设备受理';}
    },
    failed(wire){if(/^chassis test /.test(wire)){waitingAck=false;status.textContent='测试指令未确认写出 · 请查看通信状态';}},
    receive(line){
      if(!connected)return;
      const s=parseChassisTestResult(line);if(!s)return;
      if(s.kind==='test'){
        if(!waitingAck)return;
        if(s.rejected){waitingAck=false;id=null;status.textContent=`单段测试未受理 · ${reasonText(s.reason)}`;return;}
        id=s.id;waitingAck=false;status.textContent=`测试 #${id} · ${profileText[s.profile]} · 已受理，查看底盘状态读回结果`;return;
      }
      if(waitingAck||id===null||s.id!==id)return;
      id=s.id;snapshot=s;render();
    },
    connection(value){connected=value;id=null;waitingAck=false;render(true);},
    reset(){id=null;waitingAck=false;snapshot=null;metrics.hidden=tuning.hidden=true;status.textContent='设备已重启 · 等待新的单段测试';}
  };
}
