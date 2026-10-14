// Disk captures outlive the browser's 500-row terminal and the MCU's 128-row ring.
async function readJson(url){
  const response=await fetch(url);
  const result=await response.json();
  if(!response.ok)throw new Error(result.error||'读取磁盘记录失败');
  return result;
}
function download(name,kind,segment){
  const query=new URLSearchParams({name,kind});
  if(segment!==undefined)query.set('segment',segment);
  const anchor=document.createElement('a');
  anchor.href=`/api/logs/export?${query}`;anchor.download='';anchor.click();
}
export async function downloadCurrentConsole(request=readJson){
  const result=await request('/api/logs');
  if(!result.current)throw new Error('本地服务尚无记录');
  download(result.current,'console');
}
export function mountLogArchive({root,request=readJson}){
  const el=(tag,text)=>{const e=document.createElement(tag);if(text!==undefined)e.textContent=text;return e;};
  const section=el('details');section.className='advanced';
  section.append(el('summary','整段记录与完整 TXT 导出'));
  section.append(el('p','开启实时记录后持续保存，不限128条；磁盘记录可在刷新、重连和重启服务后读取。先停止并确认停稳，再关闭记录。'));
  const actions=el('div');actions.className='actions';
  const refresh=el('button','读取磁盘记录'),files=el('select'),sessions=el('select');
  files.setAttribute('aria-label','磁盘串口记录');sessions.setAttribute('aria-label','整段追踪记录');
  const tracking=el('button','导出整段追踪 TXT'),rx=el('button','导出完整接收 TXT'),consoleLog=el('button','导出完整收发 TXT');
  const status=el('p','尚未读取磁盘记录。完整性检查基于电脑收到的数据。');status.className='form-note';
  let rows=[],loading=false;
  function buttons(){refresh.disabled=loading;tracking.disabled=loading||!rows.length;rx.disabled=consoleLog.disabled=loading||!files.value;}
  function show(){
    const row=rows.find(r=>String(r.index)===sessions.value);
    status.textContent=row?`第 ${row.index+1} 段 · ${row.closed?'记录已关闭':'记录仍开启或未收到结束标记'} · 有效 ${row.records} 条 / 候选 ${row.expected_records??'未知'} 条 · 缺 ${row.missing_count??'未知'} 条 · 坏校验/格式 ${row.corrupt_rows} 条 · 有效数据跨度 ${(row.window_ms/1000).toFixed(2)} 秒 · 参数 ${row.parameter_keys} 项。${row.trace_complete?'全部候选记录校验完整。':'记录不完整；可导出分析，缺失数据不会补造。'}`:'此文件暂无追踪订阅段；可导出完整接收或收发记录。';
    buttons();
    if(row?.source_type==='pixels')status.textContent+=` 逐帧像素 ${row.valid_frames} 个有效目标 / ${row.invalid_frames} 个漏检回复，非当前目标 ${row.unmatched_frames} 帧；接收均频 ${(row.receive_hz??0).toFixed(1)} Hz，设备放弃入队 ${row.device_dropped??'未知'} 帧；速度诊断 ${row.motor_records} 条。`;
  }
  async function loadSessions(){
    rows=[];sessions.replaceChildren();show();
    if(!files.value)return;
    const result=await request(`/api/logs/tracking?name=${encodeURIComponent(files.value)}`);
    rows=result.sessions;
    for(const row of rows){const option=el('option',`第 ${row.index+1} 段 · ${row.records} 条 · ${row.trace_complete?'完整':'有缺失或未结束'}`);option.value=String(row.index);sessions.append(option);}
    sessions.value=rows.length?String(rows.at(-1).index):'';show();
  }
  async function run(operation){if(loading)return;loading=true;buttons();try{await operation();}catch(error){status.textContent=error.message;}finally{loading=false;buttons();}}
  refresh.onclick=()=>run(async()=>{
    const selected=files.value,result=await request('/api/logs');files.replaceChildren();
    for(const file of result.logs){const option=el('option',`${file.name} · ${(file.rx_bytes/1024).toFixed(1)} KB`);option.value=file.name;files.append(option);}
    files.value=result.logs.some(r=>r.name===selected)?selected:result.logs.some(r=>r.name===result.current)?result.current:result.logs[0]?.name??'';
    await loadSessions();
  });
  files.onchange=()=>run(loadSessions);sessions.onchange=show;
  tracking.onclick=()=>download(files.value,'tracking',Number(sessions.value));
  rx.onclick=()=>download(files.value,'rx');consoleLog.onclick=()=>download(files.value,'console');
  actions.append(refresh,files,sessions,tracking,rx,consoleLog);section.append(actions,status);root.append(section);buttons();
  return {refresh:refresh.onclick};
}
