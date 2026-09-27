import {FIELD} from './map-model.mjs';
import {comparison,sampleProfile,mapSegments,exposure} from './speed-profile.mjs';

const $=id=>document.getElementById(id),ns='http://www.w3.org/2000/svg';
const colors=['var(--blue)','var(--orange)'],segments=mapSegments();
const make=(tag,attrs={},text)=>{const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;return e;};
let profiles,selected,threshold,time=0,total=0,running=false,last=0,charts=[];
const settings=$('settings');
segments.forEach(s=>{const o=document.createElement('option');o.value=s.index;o.textContent=`${s.index+1}. ${s.name} · ${(s.length/1000).toFixed(2)} m`;$('segment').append(o);});
$('segment').value='4';
const field=$('field');
field.append(make('rect',{width:2400,height:2400,fill:'var(--road)',stroke:'var(--line)','stroke-width':8}));
for(let i=0;i<=2400;i+=300)field.append(make('path',{d:`M${i} 0 V2400 M0 ${i} H2400`,stroke:'var(--line)','stroke-width':2}));
for(const b of FIELD.blocks)field.append(make('rect',{x:b.x,y:b.y,width:FIELD.blockSize,height:FIELD.blockSize,fill:'var(--block)'}));
for(const b of [{x:0,y:910,w:150,h:580},{x:910,y:0,w:580,h:150}])field.append(make('rect',{x:b.x,y:b.y,width:b.w,height:b.h,fill:'var(--block)'}));
for(const y of [0,2100])field.append(make('rect',{x:2100,y,width:300,height:300,fill:'var(--blue)',opacity:.18}));
field.append(make('circle',{cx:1200,cy:2470,r:150,fill:'var(--block)',stroke:'var(--line)','stroke-width':5}));
field.append(make('path',{d:`M${segments[0].start.join(' ')} `+segments.map(s=>`L${s.end.join(' ')}`).join(' '),fill:'none',stroke:'var(--muted)','stroke-width':8,'stroke-dasharray':'18 14',opacity:.5}));
const route=make('path',{fill:'none',stroke:'var(--ink)','stroke-width':14}),markers=make('g');field.append(route,markers);
const cars=colors.map((color,i)=>{
  const g=make('g');g.append(make('rect',{x:-150,y:-150,width:300,height:300,rx:20,fill:'none',stroke:color,'stroke-width':i?7:13,...i?{'stroke-dasharray':'20 12'}:{}}));
  g.append(i?make('path',{d:'M0 40 L40 0 L0 -40 L-40 0 Z',fill:color}):make('circle',{r:65,fill:color}));
  field.append(g);return g;
});
for(const [x,y,text]of [[1200,2600,'原料区'],[1200,70,'粗加工'],[80,1650,'暂存区'],[2240,90,'启停区'],[2450,1200,'扫码']])$('labels').append(make('text',{x,y:2400-y,'text-anchor':'middle','font-size':58},text));
const rows=[['进度 / mm',s=>s.s.toFixed(0)],['速度 / m/s',s=>(s.v/1000).toFixed(3)],['加速度 / m/s²',s=>(s.a/1000).toFixed(3)],['峰值速度 / m/s',(_,p)=>(p.peak/1000).toFixed(3)],['最大 |加速度| / m/s²',(_,p)=>(p.maxAcceleration/1000).toFixed(3)],['整段用时 / s',(_,p)=>p.duration.toFixed(2)],['超出假设上限 / s',(_,p)=>exposure(p,threshold).toFixed(2)]];
const cells=rows.map(([label])=>{const tr=document.createElement('tr'),th=document.createElement('th');th.scope='row';th.textContent=label;tr.append(th);const td=()=>{const e=document.createElement('td');tr.append(e);return e;};const pair=[td(),td()];$('readouts').append(tr);return pair;});
function pause(){running=false;$('play').textContent='播放';}
function apply(){
  pause();if(!settings.reportValidity())return;
  selected=segments[Number($('segment').value)];threshold=Number($('traction').value)*1000;
  profiles=comparison(selected.length,Number($('speed').value)*1000,Number($('acceleration').value)*1000,$('comparison').value);
  total=Math.max(...profiles.map(p=>p.duration));time=0;$('scrub').max=total;
  route.setAttribute('d',`M${selected.start.join(' ')} L${selected.end.join(' ')}`);
  markers.replaceChildren(...[selected.start,selected.end].map(([x,y])=>make('circle',{cx:x,cy:y,r:24,fill:'var(--ink)'})));
  $('message').textContent=$('comparison').value==='time'
    ?'同样用时：半余弦两头更轻柔，但中间峰值加速度是线性的1.57倍。'
    :'相同最大加速度：半余弦过渡更柔和，也需要更多时间。';
  drawCharts();render();
}
function drawCharts(){
  if(!profiles)return;charts=[];
  for(const [id,key,title]of [['speed-chart','v','速度 / m/s'],['acceleration-chart','a','加速度 / m/s²']]){
    const host=$(id),width=Math.max(260,host.clientWidth),height=225,left=64,right=18,top=32,bottom=42;
    const max=key==='v'?Math.max(...profiles.map(p=>p.peak))/1000:Math.max(threshold,...profiles.map(p=>p.maxAcceleration))/1000;
    const low=key==='v'?0:-max*1.15,high=max*1.15;
    const x=t=>left+t/total*(width-left-right),y=v=>top+(high-v)/(high-low)*(height-top-bottom);
    const s=make('svg',{viewBox:`0 0 ${width} ${height}`,role:'img','aria-label':`${title}随时间变化，蓝色线性，橙色半余弦`});
    s.append(make('text',{x:left,y:18,'font-size':13},title),make('rect',{x:left,y:top,width:width-left-right,height:height-top-bottom,fill:'none',stroke:'var(--line)'}));
    for(let i=0;i<=4;i++){
      const value=key==='v'?high*i/4:low+(high-low)*i/4;
      s.append(make('path',{d:`M${left} ${y(value)} H${width-right}`,stroke:'var(--line)','stroke-width':.6}),make('text',{x:left-8,y:y(value)+4,'text-anchor':'end','font-size':12},value.toFixed(2)));
    }
    const ticks=width<400?3:4;
    for(let i=0;i<=ticks;i++){const t=total*i/ticks;s.append(make('text',{x:x(t),y:height-bottom+19,'text-anchor':i===0?'start':i===ticks?'end':'middle','font-size':12},t.toFixed(1)));}
    s.append(make('text',{x:width-right,y:height-2,'text-anchor':'end','font-size':12},'时间 / s'));
    if(key==='a')for(const sign of [-1,1]){
      const yy=y(sign*threshold/1000),edge=y(sign*high);
      s.append(make('rect',{x:left,y:Math.min(yy,edge),width:width-left-right,height:Math.abs(yy-edge),fill:'var(--risk)',opacity:.08}),make('path',{d:`M${left} ${yy} H${width-right}`,stroke:'var(--risk)','stroke-dasharray':'5 4'}));
    }
    profiles.forEach((p,j)=>{
      const times=new Set(Array.from({length:401},(_,i)=>total*i/400));
      for(const t of [0,p.ramp,p.ramp+p.cruise,p.duration]){times.add(Math.max(0,t-1e-8));times.add(t);}
      const points=[...times].sort((a,b)=>a-b).map(t=>[x(t),y(sampleProfile(p,t)[key]/1000)]);
      s.append(make('path',{d:points.map((p,i)=>`${i?'L':'M'}${p.join(' ')}`).join(' '),fill:'none',stroke:colors[j],'stroke-width':2.5,...j?{'stroke-dasharray':'7 3'}:{}}));
    });
    const guide=make('path',{stroke:'var(--muted)','stroke-width':1,'stroke-dasharray':'3 3'});
    const dots=colors.map(c=>make('circle',{r:4,fill:c}));s.append(guide,...dots);
    const overlay=make('rect',{x:left,y:top,width:width-left-right,height:height-top-bottom,fill:'transparent',style:'cursor:crosshair'});
    const seek=e=>{pause();const b=s.getBoundingClientRect();time=Math.max(0,Math.min(total,((e.clientX-b.left)*width/b.width-left)/(width-left-right)*total));render();};
    overlay.addEventListener('pointerdown',e=>{overlay.setPointerCapture(e.pointerId);seek(e);});
    overlay.addEventListener('pointermove',e=>{if(e.buttons)seek(e);});s.append(overlay);
    host.replaceChildren(s);charts.push({key,x,y,guide,dots,top,bottom:height-bottom});
  }
}
function render(){
  if(!profiles)return;
  const samples=profiles.map(p=>sampleProfile(p,time));
  samples.forEach((s,i)=>{const f=s.s/selected.length;cars[i].setAttribute('transform',`translate(${selected.start[0]+(selected.end[0]-selected.start[0])*f} ${selected.start[1]+(selected.end[1]-selected.start[1])*f})`);});
  $('clock').textContent=`${time.toFixed(2)} / ${total.toFixed(2)} s`;$('scrub').value=time;
  rows.forEach(([,format],r)=>profiles.forEach((p,i)=>{cells[r][i].textContent=format(samples[i],p);cells[r][i].classList.toggle('risk',(r===2&&Math.abs(samples[i].a)>threshold)||(r===4&&p.maxAcceleration>threshold)||(r===6&&exposure(p,threshold)>0));}));
  for(const chart of charts){const x=chart.x(time);chart.guide.setAttribute('d',`M${x} ${chart.top} V${chart.bottom}`);samples.forEach((s,i)=>{chart.dots[i].setAttribute('cx',x);chart.dots[i].setAttribute('cy',chart.y(s[chart.key]/1000));});}
}
settings.addEventListener('submit',e=>{e.preventDefault();apply();});
settings.addEventListener('change',apply);
$('play').addEventListener('click',()=>{if(running){pause();return;}if(time>=total)time=0;running=true;last=performance.now();$('play').textContent='暂停';render();});
$('reset').addEventListener('click',()=>{pause();time=0;render();});
$('scrub').addEventListener('input',()=>{pause();time=Number($('scrub').value);render();});
for(const [id,mode,a]of [['preset-time','time',.6],['preset-limit','acceleration',.6],['preset-slow','acceleration',.1]])$(id).addEventListener('click',()=>{$('segment').value='4';$('speed').value='1';$('acceleration').value=a;$('traction').value='.8';$('comparison').value=mode;apply();});
document.addEventListener('visibilitychange',()=>{if(document.hidden)pause();});
new ResizeObserver(()=>{drawCharts();render();}).observe($('speed-chart'));
function frame(now){if(running){const dt=now-last;if(dt>250){pause();$('message').textContent='页面停顿，演示已暂停；可继续播放。';}else{time=Math.min(total,time+dt/1000*Number($('rate').value));if(time>=total)pause();render();}}last=now;requestAnimationFrame(frame);}
apply();requestAnimationFrame(frame);
