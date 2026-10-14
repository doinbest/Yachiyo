/** C stage numbers are authoritative. Geometry alone only supplies waypoint numbers. */
export function routeSegments(profile){
  if(!profile?.points?.length)return [];
  let previous=profile.points[0].slice(0,2);
  const segments=[];
  if(profile.phases?.length){
    for(const phase of profile.phases){
      let segment=segments.at(-1);
      if(segment?.number!==phase.segment){segment={number:phase.segment,points:[previous],waypoint:false};segments.push(segment);}
      const point=phase.target.slice(0,2);
      if(phase.kind==='curve')for(const q of phase.path??[]){
        if(q[0]!==previous[0]||q[1]!==previous[1])segment.points.push(q.slice(0,2));
        previous=q;
      }
      if(point[0]!==previous[0]||point[1]!==previous[1])segment.points.push(point);
      segment.target=point;previous=point;
    }
  }else for(let i=1;i<profile.points.length;i++){
    const point=profile.points[i].slice(0,2);
    segments.push({number:i,points:[previous,point],target:point,waypoint:true});previous=point;
  }
  return segments;
}
export function routeLabels(segments){
  const labels=[];
  for(const segment of segments){
    const [x,y]=segment.target;
    let label=labels.find(l=>Math.hypot(l.x-x,l.y-y)<1);
    if(!label){label={x,y,numbers:[],waypoint:segment.waypoint};labels.push(label);}
    if(!label.numbers.includes(segment.number))label.numbers.push(segment.number);
  }
  return labels;
}

export function mountRouteAnnotations({root,onSelect=()=>{}}){
  const ns='http://www.w3.org/2000/svg';
  const svg=(tag,attrs,text)=>{const e=document.createElementNS(ns,tag);for(const [k,v]of Object.entries(attrs))e.setAttribute(k,v);if(text!==undefined)e.textContent=text;return e;};
  let lastKey='';
  return {render(profile,{selected=null,current=null,online=false,numbers=true,arrows=true}={}){
    const matrix=document.getElementById('field-map')?.getScreenCTM?.(),scale=Math.hypot(matrix?.a??1,matrix?.b??0)||1,fontSize=Math.max(48,12/scale);
    const key=JSON.stringify([profile,selected,current,online,numbers,arrows,fontSize]);if(key===lastKey)return;lastKey=key;
    root.replaceChildren();
    const segments=routeSegments(profile),focus=selected??current;
    for(const segment of segments){
      const active=focus===segment.number&&(selected!==null||!segment.waypoint);
      const opacity=focus!==null?(active?1:.18):1;
      const color=active?'#ff962e':online?'#008e64':'#bb4b79';
      const group=svg('g',{'data-route-segment':segment.number,opacity});
      const d=segment.points.map((p,i)=>`${i?'L':'M'}${p.join(',')}`).join(' ');
      const line=svg('path',{d,fill:'none',stroke:color,'stroke-width':active?18:7,'pointer-events':'stroke',tabindex:0,role:'button','aria-label':`${segment.waypoint?'路点':'目标'} ${segment.number} 路段`});
      const choose=()=>onSelect(segment.number);
      line.addEventListener('click',choose);line.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();choose();}});group.append(line);
      if(arrows)for(let i=1;i<segment.points.length;i++){
        const a=segment.points[i-1],b=segment.points[i],length=Math.hypot(b[0]-a[0],b[1]-a[1]);if(length<35)continue;
        const angle=Math.atan2(b[1]-a[1],b[0]-a[0])*180/Math.PI;
        // Offset the arrow along the actual segment; reverse trips occupy different positions.
        const t=.35+(segment.number%3)*.15,x=a[0]+(b[0]-a[0])*t,y=a[1]+(b[1]-a[1])*t;
        group.append(svg('path',{d:'M -32 -22 L 0 0 L -32 22',transform:`translate(${x} ${y}) rotate(${angle})`,fill:'none',stroke:color,'stroke-width':9,'pointer-events':'none'}));
      }
      root.append(group);
    }
    if(numbers)for(const label of routeLabels(segments)){
      const x=label.x>2000?label.x-100:label.x+100,y=label.y>2100?label.y-85:label.y+85;
      const active=label.numbers.includes(focus),text=(label.waypoint?'W':'')+label.numbers.join('/');
      const group=svg('g',{role:'button',tabindex:0,'pointer-events':'bounding-box','aria-label':`${label.waypoint?'路点':'目标'} ${label.numbers.join('、')}`,'data-route-numbers':label.numbers.join('/'),class:'route-number-label'});
      const width=Math.max(fontSize*1.8,text.length*fontSize*.62+20);
      group.append(svg('path',{d:`M ${label.x} ${label.y} L ${x} ${y}`,stroke:'#365c70','stroke-width':3}),svg('circle',{cx:label.x,cy:label.y,r:14,fill:active?'#ff962e':'#fff',stroke:'#365c70','stroke-width':4}),svg('rect',{x:x-width/2,y:y-fontSize*.42,width,height:fontSize*1.45,rx:12,fill:active?'#ffe2bb':'#fffffff0',stroke:active?'#bd4b00':'#365c70','stroke-width':3}),svg('text',{transform:`translate(${x} ${y}) scale(1 -1)`,'text-anchor':'middle','font-size':fontSize,'font-weight':700,fill:active?'#bd4b00':'#183b50'},text));
      const choose=()=>{const index=label.numbers.indexOf(selected);onSelect(label.numbers[(index+1)%label.numbers.length]);};
      group.addEventListener('click',choose);group.addEventListener('keydown',e=>{if(e.key==='Enter'||e.key===' '){e.preventDefault();choose();}});root.append(group);
    }
  }};
}
