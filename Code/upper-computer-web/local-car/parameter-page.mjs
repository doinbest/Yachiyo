import {buildCommand} from './protocol.mjs';
import {createParameterExchange,sameNumber} from './parameter-console.mjs';
import {parameterTabs,parameterFields,materialScenarios,displayValue,fieldWrites,deviceRequests} from './parameter-schema.mjs';
import {GRAB_TEST_PRESET} from './grab-params.mjs';
const el=(tag,text,cls)=>{const out=document.createElement(tag);if(text!==undefined)out.textContent=text;if(cls)out.className=cls;return out;};
const notes={material:'最高处 Z=0，下降为正、抬升从夹取处算；车载退出再分度；第二层下降=第一层下降−下层实高。',turntable:'ID 8 · 三仓 · 1∶1 直连；人工对齐首仓建立参考，二、三仓待标定。',vision:'普通停止保留参数；观察姿态改变后按设备状态恢复参考。',motion:'手动轴与自动取放速度独立；Z 手动脉冲限制固定不限。',radar:'雷达参数作用于下次扫描；路线速度仅存浏览器。'};

/** Persistent drafts; mounting, tab changes and navigation never transmit. */
export function mountParameterPage({root,send,canSend,onBusy=()=>{},routeSpeed=5000,onRouteSpeed=()=>{},radarExchange,radarModel}){
  const exchange=createParameterExchange({send}),rows=new Map(),values={grab:{},turntable:{},arm:{},radar:{}},panels=new Map(),buttons=new Map(),actions=[];
  const historicalValues=new Set();let tab='material',busy=false,revision=0,successes=0,failures=0;
  root.className='parameter-workspace';
  const toolbar=el('div',undefined,'parameter-toolbar'),back=el('a','','parameter-back');back.hidden=true;
  const readAll=el('button','读取当前全部参数','primary'),apply=el('button','应用本页修改');readAll.type=apply.type='button';
  const progress=el('span','尚未读取设备参数','parameter-progress');toolbar.append(back,readAll,apply,progress);root.append(toolbar);
  const nav=el('div',undefined,'parameter-tabs');nav.setAttribute('role','tablist');root.append(nav);
  const resultBar=el('div',undefined,'parameter-result-bar');resultBar.setAttribute('role','status');
  const resultLine=el('div','尚未读取 · 修改后点击应用'),detailLine=el('div','原始通信记录仍可在其他页面查看和导出');resultBar.append(resultLine,detailLine);
  const rowFor=(kind,key)=>rows.get(kind+':'+key),device=field=>field.kind==='local'?{route_speed:routeSpeed}:values[field.kind];
  function repaint(row,preserve=false,updateState=true){
    const value=displayValue(row.field,device(row.field)),known=Object.hasOwn(device(row.field),row.field.key),formatted=value===null?'未设置':String(Number(value.toPrecision(8)));
    const history=historicalValues.has(row.field.kind+':'+row.field.key)||row.field.down&&historicalValues.has(row.field.kind+':'+row.field.down);
    row.current.textContent=known?(history?'历史设备值':'设备值')+'：'+formatted+(value!==null&&row.field.unit?' '+row.field.unit:''):'设备值：未读取';
    row.current.title=row.current.textContent;
    if(known&&!preserve){row.input.value=value===null?'':String(Number(value.toPrecision(8)));row.dirty=false;row.state.textContent=value===null?'未设置':'已读取';}
    if(known&&preserve&&updateState)row.state.textContent=value!==null&&row.input.value!==''&&sameNumber(value,Number(row.input.value))?'读回一致':'待应用';
    if(updateState){row.state.dataset.state=row.state.textContent==='读回一致'?'confirmed':row.dirty?'draft':'read';row.state.title=row.state.textContent;}
  }
  function createField(field,shortLabel){
    const row=el('div',undefined,'parameter-field'),label=el('label',shortLabel??field.label),input=el('input'),control=el('div',undefined,'parameter-input-control'),current=el('small','设备值：未读取','parameter-device-value'),state=el('span','未读取','parameter-field-state');label.title=field.label;
    Object.assign(input,{type:'number',min:field.min,max:field.max,step:field.step??'any',id:'parameter-'+field.kind+'-'+field.key,readOnly:!!field.readOnly});label.htmlFor=input.id;input.dataset.parameterKey=field.key;
    input.setAttribute('aria-label',field.label+(field.unit?'（'+field.unit+'）':''));
    const unit=el('span',field.unit||'—','parameter-unit');unit.setAttribute('aria-hidden','true');control.append(input,unit);
    if(field.readOnly){input.setAttribute('aria-readonly','true');input.placeholder=field.key==='z_limit'?'固定不限脉冲':'参考 0°';}
    const entry={field,input,current,state,dirty:false,root:row};rows.set(field.kind+':'+field.key,entry);
    input.oninput=()=>{entry.dirty=true;state.textContent='待应用';state.title=state.textContent;state.dataset.state='draft';update();};
    row.id='parameter-row-'+field.kind+'-'+field.key;row.append(label,control,current,state);
    if(field.kind==='local'){input.value=String(routeSpeed);current.textContent='浏览器本地：'+routeSpeed+' mm/s';state.textContent='本地设置';}
    return row;
  }
  function cardHeading(card,title,note){
    const heading=el('div',undefined,'parameter-card-heading');heading.append(el('h3',title));
    if(note){const helper=el('span',note,'parameter-card-note');helper.title=note;heading.append(helper);}card.append(heading);
  }
  function mountMaterialScenarios(grid,fields){
    const card=el('section',undefined,'parameter-card parameter-height-card');cardHeading(card,'Z 场景距离',notes.material);
    const table=el('div',undefined,'parameter-height-table');
    for(const [name,down,lift]of materialScenarios){
      const row=el('div',undefined,'parameter-height-row');row.append(el('strong',name));
      row.append(createField(fields.find(field=>field.key===down),'下降距离'));
      const empty=el('div',undefined,'parameter-empty-cell');empty.append(el('span','抬升距离'),el('span','—'));
      row.append(lift?createField(fields.find(field=>field.key===lift),'抬升距离'):empty);
      table.append(row);
    }
    card.append(table);grid.append(card);
  }
  for(const [key,label]of parameterTabs){
    const button=el('button',label);button.type='button';button.dataset.parameterTab=key;button.setAttribute('role','tab');button.onclick=()=>{selectTab(key);if(globalThis.location)location.hash='params/'+key;};buttons.set(key,button);nav.append(button);
    const panel=el('section',undefined,'parameter-tab-panel');panel.dataset.parameterPanel=key;
    const grid=el('div',undefined,'parameter-card-grid'+(key==='material'?' parameter-material-grid':''));panel.append(grid);panels.set(key,panel);root.append(panel);
    const fields=parameterFields.filter(f=>f.tab===key);
    if(key==='material')mountMaterialScenarios(grid,fields);
    if(key==='vision'){
      const preset=el('div',undefined,'parameter-preset-row'),button=el('button','填入现有调试预设');button.type='button';
      preset.append(button,el('span','仅填草稿，应用后读回确认。','parameter-card-note'));grid.append(preset);
      button.onclick=()=>{for(const row of rows.values())if(row.field.kind==='grab'&&Object.hasOwn(GRAB_TEST_PRESET,row.field.key)){const value=displayValue(row.field,GRAB_TEST_PRESET);if(value!==null){row.input.value=String(value);row.input.oninput();}}resultLine.textContent='现有调试预设已填入草稿；尚未写入设备';};
    }
    let firstGroup=true;
    for(const group of new Set(fields.map(f=>f.group))){
      if(key==='material'&&group==='Z 场景距离')continue;
      const card=el('section',undefined,'parameter-card');cardHeading(card,group,key!=='material'&&firstGroup?notes[key]:undefined);firstGroup=false;
      const fieldGrid=el('div',undefined,'parameter-group-fields');
      for(const field of fields.filter(f=>f.group===group))fieldGrid.append(createField(field));
      card.append(fieldGrid);
      for(const field of fields.filter(f=>f.group===group&&f.note))card.append(el('p',field.note,'form-note'));
      if(group==='雷达下次静止扫描')card.append(el('p','安装参考 2170 / 230 / 180 对应起点 2250 / 150 / 90；仍需实测。分页读取一次取得整组。','form-note'));
      grid.append(card);
    }
    if(key==='turntable')mountTurntableControls(panel);
  }
  root.append(resultBar);
  const inventory=new Map();
  function mountTurntableControls(panel){
    const section=el('section',undefined,'parameter-card turntable-operations');section.append(el('h3','参考、分度与库存'));
    const grid=el('div',undefined,'turntable-operation-grid');section.append(grid);
    const group=title=>{const card=el('div',undefined,'turntable-operation-group'),row=el('div',undefined,'turntable-operation-row');card.append(el('h4',title),row);grid.append(card);return row;};
    const originRow=group('软件参考'),indexRow=group('仓位分度'),jogRow=group('相对点动'),inventoryRow=group('人工库存校正');
    const readout=el('p','转盘状态尚未读取','turntable-readout');readout.id='turntable-readout';
    const stock=el('p','三仓库存尚未读取','turntable-readout');stock.id='turntable-inventory-readout';
    const act=(title,wire,target)=>{
      const button=el('button',title);button.type='button';button.onclick=async()=>{
        try{const text=typeof wire==='function'?wire():wire;if(!canSend(text))return;detailLine.textContent=text;resultLine.textContent='已请求 · 等待设备回复';await send(text);}catch(error){resultLine.textContent=error.message;}
      };actions.push({button,wire});target.append(button);return button;
    };
    const slot=el('select');slot.id='turntable-operation-slot';for(const [value,name]of [['1','第一仓'],['2','第二仓'],['3','第三仓']]){const option=el('option',name);option.value=value;slot.append(option);}slot.value='1';const slotLabel=el('label','仓位');slotLabel.htmlFor=slot.id;indexRow.append(slotLabel,slot);
    act('建立转盘参考','turntable origin',originRow);originRow.append(el('p','人工对齐第一仓后建立参考。','form-note'));act('分度到选中仓位',()=> 'turntable index '+slot.value,indexRow);
    const jog=el('input');Object.assign(jog,{type:'number',min:-360,max:360,step:.1,placeholder:'角度',id:'turntable-operation-jog'});const jogLabel=el('label','角度');jogLabel.htmlFor=jog.id;jogRow.append(jogLabel,jog,el('span','°','parameter-unit'));
    act('点动',()=>buildCommand('turntable-jog',{degree:jog.value}),jogRow);act('停止转盘','turntable stop',jogRow);
    const content=el('select');content.id='turntable-operation-inventory';for(const [value,name]of [['empty','空仓'],['unknown','待核对'],['1','红色'],['2','黄色'],['3','蓝色'],['4','绿色'],['5','黑色'],['6','浅蓝色']]){const option=el('option',name);option.value=value;content.append(option);}content.value='unknown';const contentLabel=el('label','实际库存');contentLabel.htmlFor=content.id;inventoryRow.append(contentLabel,content);
    act('校正选中仓库存',()=> 'turntable inventory '+slot.value+' '+content.value,inventoryRow);act('确认三仓为空','turntable inventory empty',inventoryRow);inventoryRow.append(el('p','校正上方选中的仓位；三仓为空请单独确认。','form-note'));
    const statusActions=el('div',undefined,'turntable-status-actions');act('读取状态','turntable status',statusActions);act('读取库存','turntable inventory',statusActions);section.append(statusActions);
    section.append(readout,stock,el('p','库存表示软件动作结果。请核对实际物料；停止保留记录，中断相关仓位可能标为待核对。','form-note'));panel.append(section);
  }
  function selectTab(key='material',parameter){
    if(!panels.has(key))key='material';tab=key;
    for(const [name,panel]of panels)panel.hidden=name!==tab;
    for(const [name,button]of buttons){button.setAttribute('aria-selected',String(name===tab));button.classList?.toggle('active',name===tab);}
    if(parameter)rowFor(parameterFields.find(f=>f.tab===tab&&f.key===parameter)?.kind,parameter)?.root.scrollIntoView?.({block:'center'});
    update();
  }
  function setReturn(source){back.hidden=!source;if(source){back.href=source.hash;back.textContent='← 返回'+source.label;}}
  const check=token=>{if(token!==revision)throw new Error('参数操作已取消；已读取值和输入已保留。');};
  function accept(kind,key,value,preserve=false){
    values[kind][key]=value;historicalValues.delete(kind+':'+key);
    for(const row of rows.values())if(row.field.kind===kind){
      if(row.field.key===key)repaint(row,preserve);
      else if(row.field.down===key)repaint(row,true,false); // A related read updates the device display, never this field's draft or result.
    }
  }
  function showError(kind,key,error){const affected=[...rows.values()].filter(row=>row.field.kind===kind&&(row.field.key===key||kind==='arm'&&row.field.axis===key||kind==='radar'&&key==='all'));for(const row of affected){row.state.textContent=/unknown|unsupported|invalid_key|not_found|unknown_key|未支持/.test(error.message)?'未支持':error.message;row.state.title=error.message;row.state.dataset.state='failed';}}
  async function run(work,localOnly=false){
    if(busy||(!localOnly&&!canSend('grab get z_grab')))return;
    busy=true;successes=failures=0;const token=revision;onBusy(true);update();
    try{await work(token);resultLine.textContent='完成 · 成功 '+successes+' 项'+(failures?'，失败 '+failures+' 项':'');}
    catch(error){resultLine.textContent=error.message;}
    finally{busy=false;progress.textContent=successes+' 项成功'+(failures?' · '+failures+' 项失败':'');onBusy(false);update();}
  }
  async function readRequest(request,preserve=false){
    if(request.kind==='arm'){const result=await exchange.readArm(request.key);for(const part of ['rpm','acc','limit'])accept('arm',request.key+'_'+part,result[part],preserve);return 3;}
    if(request.kind==='radar'){
      if(!radarExchange)throw new Error('雷达参数尚未支持');
      const previousProgress=radarExchange.onProgress;radarExchange.onProgress=({page,pages})=>{progress.textContent='读取雷达参数 · '+page+'/'+pages+' 页';detailLine.textContent=progress.textContent;previousProgress?.({kind:'params',page,pages});};
      let pages;try{pages=await radarExchange.get();}finally{radarExchange.onProgress=previousProgress;}
      const params=Object.assign({},...pages.filter(p=>p.key).map(p=>({[p.key]:p.value})));
      let count=0;for(const row of rows.values())if(row.field.kind==='radar'){if(!Object.hasOwn(params,row.field.key)){showError('radar',row.field.key,new Error('设备未报告'));failures++;continue;}accept('radar',row.field.key,params[row.field.key],preserve);count++;}return count;
    }
    const result=await (request.kind==='grab'?exchange.readGrab(request.key):exchange.readTurntable(request.key));accept(request.kind,request.key,result.value,preserve);return 1;
  }
  readAll.onclick=()=>run(async token=>{
    const requests=deviceRequests();
    for(let i=0;i<requests.length;i++){
      check(token);const request=requests[i];progress.textContent='读取 '+(i+1)+'/'+requests.length+' · '+request.kind+' '+request.key;resultLine.textContent='正在读取当前全部参数';detailLine.textContent=progress.textContent;
      try{successes+=await readRequest(request);}catch(error){check(token);failures+=request.kind==='arm'?3:request.kind==='radar'?parameterFields.filter(p=>p.kind==='radar').length:1;showError(request.kind,request.key,error);detailLine.textContent=error.message;}
    }
  });
  apply.onclick=()=>{
    const changed=[...rows.values()].filter(row=>row.field.tab===tab&&row.dirty&&!row.field.readOnly),localOnly=changed.every(row=>row.field.kind==='local');
    return run(async token=>{
      const drafts=Object.fromEntries([...rows.values()].filter(row=>row.field.kind==='grab').map(row=>[row.field.key,row.input.value])),writes=new Map();
      for(const row of changed){
        if(row.field.kind==='arm')writes.set('arm:'+row.field.axis,{kind:'arm',key:row.field.axis});
        else for(const [key,value]of fieldWrites(row.field,row.field.kind==='grab'?drafts:{[row.field.key]:row.input.value}))writes.set(row.field.kind+':'+key,{kind:row.field.kind,key,value});
      }
      if(!writes.size){detailLine.textContent='当前标签没有待应用的修改';return;}
      let index=0;for(const item of writes.values()){
        check(token);progress.textContent='应用 '+(++index)+'/'+writes.size+' · '+item.kind+' '+item.key;resultLine.textContent='等待受理 → 主动读回 → 比较';detailLine.textContent=progress.textContent;
        try{
          let result;
          if(item.kind==='local'){onRouteSpeed(item.value);routeSpeed=item.value;result={confirmed:true};const row=rowFor('local',item.key);row.current.textContent='浏览器本地：'+routeSpeed+' mm/s';row.dirty=false;row.state.textContent='本地已保存';}
          else if(item.kind==='arm'){const parts=['rpm','acc','limit'].map(part=>rowFor('arm',item.key+'_'+part).input.value);result=await exchange.setArm(item.key,...parts);for(const part of ['rpm','acc','limit'])accept('arm',item.key+'_'+part,result[part],true);}
          else{result=await (item.kind==='grab'?exchange.setGrab(item.key,item.value):item.kind==='turntable'?exchange.setTurntable(item.key,item.value):radarExchange.set(item.key,item.value));accept(item.kind,item.key,result.value,true);}
          if(!result.confirmed)throw new Error('读回与输入不一致');successes++;
          for(const row of rows.values())if(row.field.kind===item.kind&&(row.field.key===item.key||item.kind==='arm'&&row.field.axis===item.key)){
            const shown=displayValue(row.field,device(row.field));
            row.dirty=item.kind!=='local'&&(row.input.value===''||!sameNumber(shown,Number(row.input.value)));
            if(item.kind!=='local')row.state.textContent=row.dirty?'读回与界面距离不一致':'读回一致';
          }
        }catch(error){check(token);failures++;showError(item.kind,item.key,error);detailLine.textContent=error.message;}
      }
    },localOnly);
  };
  function update(){
    const changed=[...rows.values()].filter(row=>row.field.tab===tab&&row.dirty);
    readAll.disabled=busy||!canSend('grab get z_grab');apply.disabled=busy||!changed.length||(!changed.every(row=>row.field.kind==='local')&&!canSend('grab get z_grab'));
    for(const {button,wire}of actions){try{button.disabled=busy||!canSend(typeof wire==='function'?wire():wire);}catch{button.disabled=true;}}
  }
  function cancel(reason='操作已取消',markHistorical=false){
    revision++;exchange.cancel(reason);radarExchange?.cancel(reason);
    if(markHistorical){
      for(const row of rows.values())if(row.field.kind!=='local'&&Object.hasOwn(device(row.field),row.field.key))historicalValues.add(row.field.kind+':'+row.field.key);
      for(const row of rows.values())if(row.field.kind!=='local'){repaint(row,true);if(Object.hasOwn(device(row.field),row.field.key))row.state.textContent=row.dirty?'待应用 · 历史设备值':'历史值 · 请重新读取';}
      for(const id of ['turntable-readout','turntable-inventory-readout']){const item=panels.get('turntable').querySelector?.('#'+id);if(item&&!item.textContent.startsWith('历史'))item.textContent='历史 · '+item.textContent+' · 请重新读取';}
    }
    if(busy)resultLine.textContent=reason;update();
  }
  function receive(line){
    const consumed=exchange.receive(line);
    if(/^OK turntable (?:status state=|inventory )/.test(line)){
      const match=line.match(/slot=(\d+) state=(\w+) color=(\d+)/);if(match)inventory.set(match[1],{state:match[2],color:match[3]});
      const readout=panels.get('turntable').querySelector?.(match?'#turntable-inventory-readout':'#turntable-readout');
      const status=Object.fromEntries(line.split(/\s+/).filter(token=>token.includes('=')).map(token=>token.split('=')));
      const text=match?[1,2,3].map(slot=>{const item=inventory.get(String(slot));return slot+'仓：'+(item?({empty:'空仓',unknown:'待核对',occupied:'占用 · '+({1:'红色',2:'黄色',3:'蓝色',4:'绿色',5:'黑色',6:'浅蓝色'})[item.color]})[item.state]:'未读取');}).join('　'):
        (status.ref==='1'?'参考已建立':'参考未建立')+' · '+({idle:'空闲',reading:'正在读取反馈',moving:'正在分度',stopping:'正在停车',arrived:'实际到位'})[status.state]+' · 当前 '+status.angle_deg+'° / 目标 '+status.target_deg+'°';
      if(readout)readout.textContent=text;resultLine.textContent=text;detailLine.textContent=match?'库存已读回；以实际物料核对为准':'设备原因：'+status.reason;
    }
    if(/^OK turntable accepted\b/.test(line)){resultLine.textContent='转盘请求已受理 · 等待反馈确认';detailLine.textContent='点击读取状态查看参考建立或实际到位结果';}
    if(/^ERR turntable\b/.test(line)){resultLine.textContent='设备拒绝：'+line.slice(4);detailLine.textContent='请按设备原因处理后再操作';}
    return consumed;
  }
  selectTab();return {receive,cancel,update,selectTab,setReturn,readAll:()=>readAll.onclick(),apply:()=>apply.onclick(),get tab(){return tab;},get busy(){return busy;}};
}
