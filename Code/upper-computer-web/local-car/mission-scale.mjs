/** Validate a new mission draft without changing device state. */
export function missionSpeedSettings(speed,scale=1,profile='linear'){
  const base=Number(speed),factor=Number(scale);
  if(!String(speed).trim()||!Number.isFinite(base)||base<10||base>5000)throw Error('基准总限速须为10～5000 mm/s');
  if(!String(scale).trim()||!Number.isFinite(factor)||factor<0.1||factor>5)throw Error('比赛倍率K须为0.1～5');
  if(profile==='cosine'&&factor!==1)throw Error('余弦比赛仅支持倍率K=1');
  const effective=base*factor;
  if(effective<10||effective>5000)throw Error('实际总限速须为10～5000 mm/s（基准总限速×比赛倍率K）');
  return {base,scale:factor,effective};
}
