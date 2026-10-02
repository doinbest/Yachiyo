export class WheelFeedback {
  constructor(now=()=>performance.now()){this.now=now;this.clear();}
  clear(){this.samples=Array(4).fill(null);}
  receive(line){
    const m=String(line).match(/^(?:arm>\s*)?OK wheel=([1-4]) raw_rpm=(-?\d+) raw_units=(-?\d+) flags=(0x[\da-f]+) valid=([01]),([01]),([01]) rx_ms=(\d+),(\d+),(\d+)$/i);
    if(!m)return false;
    const index=Number(m[1])-1,old=this.samples[index];
    if(old?.signature===m[0])return true;
    this.samples[index]={signature:m[0],rpm:Number(m[2]),position:m[3],speedValid:m[5]==='1',positionValid:m[6]==='1',seen:this.now()};
    return true;
  }
  rows(){return this.samples.map(s=>s?{...s,age:Math.floor((this.now()-s.seen)/1000)}:null);}
}

export function mountWheelFeedback(root){
  const model=new WheelFeedback(),doc=root.ownerDocument;
  const table=doc.createElement('table');table.className='wheel-table';
  const head=doc.createElement('thead');head.innerHTML='<tr><th>车轮</th><th>速度 RPM</th><th>位置单位</th></tr>';
  const body=doc.createElement('tbody'),note=doc.createElement('p');note.className='form-note';
  table.append(head,body);const wrap=doc.createElement('div');wrap.className='wheel-table-wrap';wrap.append(table);root.append(wrap,note);
  let connected=false;
  function render(){
    body.replaceChildren(...model.rows().map((s,i)=>{
      const row=doc.createElement('tr');
      for(const text of [['左前','左后','右后','右前'][i],s?.speedValid?String(s.rpm):'—',s?.positionValid?s.position:'—']){const td=doc.createElement('td');td.textContent=text;row.append(td);}
      const detail=doc.createElement('small');detail.textContent=s?`${s.age} 秒无新样本`:'暂无反馈';row.firstChild.append(doc.createElement('br'),detail);return row;
    }));
    note.textContent=connected?'这是驱动缓存快照，非连续实时显示。位置 65536 单位 = 电机轴一圈；“—”表示无有效数据。准备跑图会开启四轮采集；连续数据请看地图遥测。':'未连接 · 连接后可查看四轮快照。';
  }
  setInterval(render,1000);render();
  return {receive(line){if(model.receive(line))render();},connection(value){connected=value;model.clear();render();}};
}
