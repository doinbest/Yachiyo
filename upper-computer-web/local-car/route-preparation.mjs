const accepted='OK chassis request accepted (not motion/ACK confirmation)';
const fields=line=>Object.fromEntries([...line.matchAll(/\b([a-z_]+)=([^\s]+)/g)].map(m=>[m[1],m[2]]));
const idleRoute=s=>['idle','done','error','cancelled'].includes(s);
export const validCarConfig=c=>!!c&&['receive','none'].includes(c.profile)&&c.units_per_rev===65536&&c.directions_confirmed===true;

/** Explicit browser preparation only. Never sends motion or driver configuration. */
export class RoutePreparation {
  constructor({send,now=()=>performance.now(),beginTelemetry=()=>{},telemetry=()=>null,
    nonce=()=>{let n=0;while(!n)n=crypto.getRandomValues(new Uint32Array(1))[0];return n;},getConfig=()=>null,finished=()=>{},changed=()=>{}}={}) {
    Object.assign(this,{send,now,beginTelemetry,telemetry,nonce,getConfig,finished,changed});
    this.phase='idle';this.message='连接后准备跑图 · 全程保持静止，不会启动电机。';
    this.connected=false;this.serial=0;this.pending=null;
  }
  get busy(){return !['idle','failed','cancelled','ready'].includes(this.phase);}
  show(phase,message){this.phase=phase;this.message=message;this.changed();if(!this.busy)this.finished();}
  connection(value){
    this.connected=value;
    this.cancel(value?'已连接 · 请点击准备跑图。':'连接已断开 · 准备不会自动继续。');
  }
  cancel(message='已取消准备；已启动的静止验证会自行结束，不再设置原点。'){
    this.serial++;this.pending=null;this.show('cancelled',message);
  }
  fail(message){this.serial++;this.pending=null;this.show('failed',message);}
  request(phase,wire,message){
    const serial=++this.serial;
    this.pending={wire,echo:false,lines:[],deadline:this.now()+12000};
    this.show(phase,message);
    Promise.resolve(this.send(wire)).then(ok=>{
      if(!ok&&this.serial===serial)this.fail('发送未完成，请检查连接后重新准备。');
    }).catch(()=>{if(this.serial===serial)this.fail('发送失败，请检查连接后重新准备。');});
  }
  start(){
    if(!this.connected||this.busy)return false;
    const config=this.getConfig();
    if(!validCarConfig(config)){this.fail('请先展开“本车配置”，核对应答模式、实测位置单位和四轮方向，并保存。');return false;}
    this.config={...config};
    this.verifiedNow=false;this.session=null;this.streamAccepted=false;
    this.request('quiet','chassis stream off','1/5 暂停遥测，检查底盘配置与任务…');
    return true;
  }
  readImu(){this.request('imu','imu status','3/5 检查本次启动的 IMU 验证资格…');}
  confirmOrigin(){
    if(this.phase!=='confirm'||!this.connected)return false;
    this.request('origin','chassis origin 2250 150 90','4/5 设置已确认的起点 (2250,150,90°)…');
    return true;
  }
  receive(raw){
    const p=this.pending;
    if(!this.busy||!p)return;
    if(this.now()>p.deadline){this.tick();return;}
    const line=String(raw).trim().replace(/^(?:arm>\s*)+/,'');
    if(line===p.wire){p.echo=true;return;}
    // The current firmware echoes commands; no echo means no reliable association.
    if(!p.echo)return;
    if(line.startsWith('ERR ')){this.fail(`设备拒绝：${line}`);return;}
    p.lines.push(line);
    const f=fields(line);
    switch(this.phase){
      case 'quiet':
        if(line===accepted)this.request('task','chassis task','1/5 读取应答模式、位置单位及总线状态…');
        break;
      case 'task':
        if(!line.startsWith('OK chassis task '))break;
        if(f.locked!=='0'||!['0','3','4'].includes(f.state)||!['0','5'].includes(f.stage)){
          this.fail('底盘未空闲或总线锁定，请先查看任务状态并处理故障。');break;
        }
        this.request('route','chassis route status','1/5 检查是否有路线仍在运行或等待继续…');
        break;
      case 'route':
        if(!line.startsWith('OK chassis route '))break;
        if(!idleRoute(f.state)){this.fail('已有路线占用底盘，请先取消并确认停车后重新准备。');break;}
        this.request('profile','chassis profile '+this.config.profile,'1/5 应用已保存的驱动应答模式（仅 MCU RAM）…');break;
      case 'profile':
        if(line===accepted)this.request('units','chassis units 65536','1/5 应用已实测的位置单位（仅 MCU RAM）…');
        break;
      case 'units':
        if(line===accepted)this.request('configured','chassis task','1/5 复查配置已生效且底盘空闲…');
        break;
      case 'configured':
        if(!line.startsWith('OK chassis task '))break;
        if(f.locked!=='0'||!['0','3','4'].includes(f.state)||!['0','5'].includes(f.stage)||
          f.ack!==(this.config.profile==='receive'?'2':'1')||f.units!=='65536'){
          this.fail('保存的配置未生效，或底盘已被占用。请检查任务中的应答模式与位置单位。');break;
        }
        this.request('select','chassis feedback 0','2/5 选择四轮反馈…');break;
      case 'select':
        if(line===accepted)this.request('poll','chassis feedback on','2/5 开启四轮反馈轮询…');
        break;
      case 'poll':
        if(line===accepted){this.pending=null;this.until=this.now()+2000;this.show('feedback_wait','2/5 等待四轮反馈更新…');}
        break;
      case 'feedback': {
        if(!line.startsWith('OK wheel=4 '))break;
        const wheels=p.lines.filter(s=>s.startsWith('OK wheel=')).map(fields);
        if(wheels.length!==4||wheels.some((w,i)=>w.wheel!==String(i+1)||!/^1,1,[01]$/.test(w.valid??'')||
          !/^-?\d+$/.test(w.raw_rpm??'')||Math.abs(Number(w.raw_rpm))>1)){
          this.fail('四轮速度/位置反馈未全部有效，或轮子仍在转动。请查看“读取四轮反馈缓存”。');break;
        }
        this.readImu();break;
      }
      case 'imu': {
        // Wait for the last status line, so the next command cannot steal a prior reply.
        if(!line.startsWith('OK imu relative_deg='))break;
        const sensor=fields(p.lines.find(s=>s.startsWith('OK imu ready='))??'');
        const cal=fields(p.lines.find(s=>s.startsWith('OK imu cal_state='))??'');
        if(sensor.ready!=='1'||sensor.valid!=='1'||sensor.fresh!=='1'||cal.busy!=='0'){
          this.fail('IMU 未就绪、数据过期或正在标定，请读取 IMU 状态。');break;
        }
        if(cal.verified==='1'&&cal.control_ready==='1'&&cal.normal_mode==='1'&&
          (!this.verifiedNow||cal.run_id!==this.previousRun)){
          this.pending=null;this.until=this.now()+120000;
          this.show('confirm','3/5 检查通过。请确认车体中心在右下起点 (2250,150)，车头朝上且整车静止。');break;
        }
        if(this.verifiedNow){this.fail('10 秒验证未通过，请查看 IMU 结果；必要时执行原生标定。');break;}
        this.previousRun=cal.run_id;
        this.request('verify','imu verify 10','3/5 请求 10 秒静止验证，请勿移动小车…');break;
      }
      case 'verify':
        if(!line.startsWith('OK imu verify started '))break;
        if(f.verify_ms!=='10000'){this.fail('固件未执行 10 秒验证，请更新本轮固件。');break;}
        this.verifiedNow=true;this.pending=null;this.until=this.now()+11500;
        this.show('verify_wait','3/5 正在进行 10 秒静止验证，完成后自动读取结果…');break;
      case 'origin':
        if(line!==accepted)break;
        this.session=this.nonce();this.beginTelemetry(this.session);
        this.request('telemetry','chassis stream on '+this.session,'5/5 开启遥测，等待起点反馈与静止状态…');break;
      case 'telemetry':
        if(line===accepted){this.streamAccepted=true;}
        break;
    }
  }
  validStartSnapshot(){
    const t=this.telemetry(),r=t?.state,p=t?.feedback;
    if(!t||t.session!==this.session||!r||!p||r.localization?.origin_valid!==true||
      !['idle','done','error'].includes(r.task?.state)||!idleRoute(r.route?.state)||r.tx?.error!==0||
      !Array.isArray(r.target?.rpm)||r.target.rpm.some(v=>v!==0)||
      !Array.isArray(r.feedback?.rpm)||r.feedback.rpm.length!==4)return false;
    if(r.feedback.rpm.some((v,i)=>!Number.isFinite(v)||Math.abs(v)>1||r.feedback.speed_valid?.[i]!==true||
      !Number.isInteger(r.feedback.speed_ms?.[i])||((r.t_ms-r.feedback.speed_ms[i])>>>0)>600))return false;
    const angle=Math.atan2(Math.sin(p.yaw_rad-Math.PI/2),Math.cos(p.yaw_rad-Math.PI/2));
    return Math.hypot(p.x_mm-2250,p.y_mm-150)<=10&&Math.abs(angle)<=2*Math.PI/180;
  }
  tick(){
    if(this.phase==='ready'){
      if(!this.validStartSnapshot())this.cancel('起点反馈已变化或遥测失效；开始前请重新检查，运动中以实车任务状态为准。');
      return;
    }
    if(!this.busy)return;
    if(this.phase==='telemetry'&&this.streamAccepted&&this.validStartSnapshot()){
      this.pending=null;this.show('ready','准备完成 · 起点反馈有效，可以点击“开始第 1 段”。');return;
    }
    if(this.pending&&this.now()>this.pending.deadline){
      this.fail(this.phase==='telemetry'?'未收到有效起点反馈，请查看地图遥测、轮子反馈与 IMU 状态。':'等待设备回复超时，请查看收发记录后重新准备。');return;
    }
    if(this.phase==='feedback_wait'&&this.now()>=this.until)
      this.request('feedback','chassis feedback','2/5 检查四轮反馈与静止速度…');
    else if(this.phase==='verify_wait'&&this.now()>=this.until)this.readImu();
    else if(this.phase==='confirm'&&this.now()>this.until)this.cancel('起点确认等待超时，请重新准备。');
  }
}

export function mountPreparation({root,send,beginTelemetry,telemetry,getConfig=()=>null,saveConfig,configAvailable=()=>false,isStopped=()=>false,acquire=async()=>{},release=async()=>{},changed=()=>{}}){
  const doc=root.ownerDocument,make=(tag,text)=>{const e=doc.createElement(tag);e.textContent=text;return e;};
  const section=make('section','');section.className='route-preparation';section.id='route-preparation';
  section.append(make('h3','准备跑图'),make('p','保持整车静止：先应用已保存的本车配置，再检查四轮反馈、按需验证 IMU 10 秒，最后确认起点。'));
  const details=make('details','');details.className='car-config';details.append(make('summary','本车配置 · 首次核对后保存'));
  const profileLabel=make('label','已核对的驱动应答模式'),profile=make('select','');profile.id='car-profile';
  for(const [value,title] of [['','请选择实际模式'],['receive','接收应答 · receive'],['none','无应答 · none']]){const option=make('option',title);option.value=value;profile.append(option);}profileLabel.append(profile);
  const unitsLabel=make('label',''),units=make('input','');units.type='checkbox';units.id='car-units-confirmed';unitsLabel.append(units,make('span','已实测一圈反馈变化为 65536 协议位置单位'));
  const directionsLabel=make('label',''),directions=make('input','');directions.type='checkbox';directions.id='car-directions-confirmed';directionsLabel.append(directions,make('span','已逐轮确认四轮正方向与车体前向一致'));
  const save=make('button','保存本车配置'),configText=make('p','');save.id='car-config-save';save.type='button';configText.id='car-config-status';configText.setAttribute('role','status');
  details.append(profileLabel,unitsLabel,directionsLabel,make('p','只保存到本机服务；准备时设置 MCU RAM，不写驱动器或 Flash。更换驱动设置、轮向或测量结果后须重新核对。'),save,configText);section.append(details);
  const row=make('div','');row.className='actions';
  const start=make('button','准备跑图'),confirm=make('button','确认已在起点，继续准备'),cancel=make('button','取消准备');
  start.className='primary';confirm.className='primary';row.append(start,confirm,cancel);
  const text=make('p','');text.setAttribute('role','status');section.append(row,text);root.prepend(section);
  const state=new RoutePreparation({send,beginTelemetry,telemetry,getConfig:()=>configAvailable()?getConfig():null,finished:()=>{void release().catch(error=>{text.textContent='结束准备锁定失败：'+error.message;});},changed:()=>{render();changed();}});
  let saving=false,loaded;
  function render(){text.textContent=state.message;start.disabled=!state.connected||state.busy||isStopped()||!configAvailable();
    confirm.hidden=state.phase!=='confirm';confirm.disabled=isStopped();cancel.hidden=!state.busy;
    const c=getConfig();if(c!==loaded){loaded=c;profile.value=c?.profile||'';units.checked=c?.units_per_rev===65536;directions.checked=c?.directions_confirmed===true;}
    profile.disabled=units.disabled=directions.disabled=state.busy||saving;
    save.disabled=!configAvailable()||state.busy||saving||isStopped();
    configText.textContent=!configAvailable()?'本地服务不可用，无法加载或保存配置。':validCarConfig(c)?'已保存：'+c.profile+' · 65536 单位/圈 · 四轮方向已确认':'尚未保存完整的本车配置。';}
  save.onclick=async()=>{const c={profile:profile.value,units_per_rev:units.checked?65536:null,directions_confirmed:directions.checked};if(!validCarConfig(c)){configText.textContent='请选择应答模式，并确认实测位置单位与四轮方向后再保存。';return;}saving=true;render();try{await saveConfig(c);state.cancel('本车配置已保存；点击准备跑图时应用。');}catch(error){state.fail('保存失败：'+error.message);}finally{saving=false;render();}};
  state.refreshConfiguration=render;
  start.onclick=async()=>{if(!state.connected||state.busy||isStopped())return;if(!validCarConfig(state.getConfig())){state.start();return;}state.show('acquiring','正在锁定本次准备；其他窗口只能查询或停止…');const serial=state.serial;try{await acquire();if(state.serial!==serial||state.phase!=='acquiring'||isStopped()){await release();return;}state.phase='idle';state.streamAccepted=false;state.start();}catch(error){state.fail('无法开始准备：'+error.message);}};confirm.onclick=()=>state.confirmOrigin();cancel.onclick=()=>state.cancel();
  setInterval(()=>state.tick(),100);render();return state;
}
