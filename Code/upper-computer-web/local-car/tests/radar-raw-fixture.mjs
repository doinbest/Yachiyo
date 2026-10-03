export const rawFrames=(overrides={})=>{
  const base={v:1,k:'raw',session:9,origin:2,scan:3,map:7,plan:0,id:11,pages:7,...overrides};
  return [
    {state:'ready',complete:true,truncated:true,total:9,crc32:'CBF43926'},
    {pose:[2170,230,180],unit:1,start:2,auto:false,points:2},
    {filter:[100,40000,0,3600,3,0,255],point_total:2},
    {window:100,circle:[101,109],cloud_truncated:false},
    {received:109,losses:0,bad:1,restarts:1,cloud_valid:true},
    {offset:0,total:9,hex:'313233'},
    {offset:3,total:9,hex:'343536373839'}
  ].map((fields,page)=>({...base,page,...fields}));
};
export const rawLine=frame=>'@RADAR '+JSON.stringify(frame);
