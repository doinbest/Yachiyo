const ns='http://www.w3.org/2000/svg';
function svg(tag,attrs={},text){
  const node=document.createElementNS(ns,tag);
  for(const [key,value] of Object.entries(attrs))node.setAttribute(key,value);
  if(text!==undefined)node.textContent=text;
  return node;
}

/** Body +X is forward and +Y is left. This is an installation/phase illustration. */
export function appendArmMarker(root,pose,size,armPose='rear'){
  const length=size.length_mm,width=size.width_mm,baseY=width*.22;
  const reach=length*.4,work=armPose==='work',tipX=work?0:-reach,tipY=work?baseY+width*.25:baseY;
  const layer=svg('g',{'data-vehicle-arm':armPose,transform:`translate(${pose.x_mm} ${pose.y_mm}) rotate(${pose.yaw_rad*180/Math.PI})`,'pointer-events':'none'});
  layer.append(svg('title',{},`机械臂安装示意 · 车体中左 · ${work?'作业朝向（顺时针90°）':'朝后'}`),
    svg('path',{d:`M 0 ${baseY} L ${tipX} ${tipY}`,stroke:'#9b4600','stroke-width':14,'stroke-linecap':'round'}),
    svg('path',{d:work?`M ${tipX-22} ${tipY-28} L ${tipX} ${tipY} L ${tipX+22} ${tipY-28}`:`M ${tipX+28} ${tipY-22} L ${tipX} ${tipY} L ${tipX+28} ${tipY+22}`,fill:'none',stroke:'#9b4600','stroke-width':12,'stroke-linejoin':'round'}),
    svg('circle',{cx:0,cy:baseY,r:23,fill:'#ffb347',stroke:'#753600','stroke-width':7}));
  root.append(layer);
}
