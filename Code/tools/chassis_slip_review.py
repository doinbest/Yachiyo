"""Offline sensitivity study. No serial/debugger/firmware writes; assumed, unmeasured contact data.

Equal vertical support stiffness, all four wheels in contact, planar floor,
45-degree ideal rollers. Force envelopes are feasibility bounds, not a slip probability.
"""
from pathlib import Path
import argparse,csv,json,math,sys

ROOT=Path(__file__).resolve().parents[2]
parser=argparse.ArgumentParser()
parser.add_argument('--output',type=Path,default=ROOT/'.embeddedskills/analysis/high-speed-left-cg')
args=parser.parse_args();out=args.output;out.mkdir(parents=True,exist_ok=True)
# Optional isolated numerical runtime used for this report; does not change global Python.
if (out/'deps').exists():sys.path.insert(0,str(out/'deps'))
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.font_manager import FontProperties
font=Path('C:/Windows/Fonts/msyh.ttc')
if font.exists():plt.rcParams['font.family']=FontProperties(fname=str(font)).get_name()
plt.rcParams.update({'axes.unicode_minus':False,'font.size':10,'figure.dpi':140})
G=9.81;L=.200;B=.240;K=(L+B)/2;R=.040
SIGMA_X=np.array([1,-1,-1,1]);SIGMA_Y=np.array([1,1,-1,-1]);S=np.array([-1,1,-1,1])
Z=np.array([1,-1,-1,1]);A=np.array([np.ones(4),S,K*np.array([-1,-1,1,1])])

def normals(e,h,ax=0,ay=0):
    """Ni/(m*g). Four equal-compliance supports; not a unique rigid four-point solution."""
    return .25*(1+2*e*SIGMA_Y/B-2*ax*h*SIGMA_X/(G*L)-2*ay*h*SIGMA_Y/(G*B))

def force_coeff(e):
    # fi=ma/4*c_i produces Fx=ma, Fy=0, M_geo=-e*ma (zero CG yaw acceleration).
    return np.array([1+e/K,1+e/K,1-e/K,1-e/K])

def limit(e,h,mu,mode='cg_balanced'):
    coeff=np.ones(4) if mode=='equal_effort' else force_coeff(e)
    side=1+2*e*SIGMA_Y/B
    if mode!='feasible':
        return float(np.min(mu*G/math.sqrt(2)*side/(coeff+math.sqrt(2)*mu*h/L)))
    def feasible(a):
        n=normals(e,h,a)
        if min(n)<0:return False
        base=a/4*force_coeff(e)  # fi/m
        bounds=mu*G*n/math.sqrt(2)
        lo,hi=-float('inf'),float('inf')
        for f,b,z in zip(base,bounds,Z):
            p,q=sorted(((-b-f)/z,(b-f)/z));lo=max(lo,p);hi=min(hi,q)
        return lo<=hi
    low,high=0.,G*L/(2*h) if h else 100.
    for _ in range(60):
        mid=(low+high)/2
        if feasible(mid):low=mid
        else:high=mid
    return low

def ramp(t,v,T):
    u=np.clip(t/T,0,1)
    speed=v*(1-np.cos(np.pi*u))/2
    accel=np.where((t>=0)&(t<=T),v*np.pi/(2*T)*np.sin(np.pi*u),0)
    return speed,accel

def savefig(name):
    plt.savefig(out/(name+'.png'),bbox_inches='tight')
    plt.savefig(out/(name+'.svg'),bbox_inches='tight');plt.close()

def c_round(x):return np.sign(x)*np.floor(np.abs(x)+.5)
def wheel_rpm(vx,vy,omega):
    # Production source wheelbase 210mm / track240mm, distinct from 200mm measured L.
    k_source=.225
    return c_round((vx+S*vy+np.array([-1,-1,1,1])*k_source*omega)*60/(2*np.pi*R))

# Numerical invariants / sanity checks, independent of any real robot result.
for e in (0,.02,.04,.06):
    n=normals(e,.1,1.2)
    assert abs(sum(n)-1)<1e-12 and abs(np.dot(n,SIGMA_Y*B/2)-e)<1e-12
    assert abs(np.dot(n,SIGMA_X*L/2)+1.2*.1/G)<1e-12
    assert np.allclose(A@(force_coeff(e)/4),[1,0,-e])
    assert np.linalg.norm(A@Z)<1e-12
    assert limit(e,.1,.5,'feasible')+1e-8>=limit(e,.1,.5)
assert abs(limit(0,0,.5)-.5*G/math.sqrt(2))<1e-12
assert np.array_equal(wheel_rpm(1,0,0),[239]*4)

e,h,mu=.04,.10,.5
parameters={'body_mm':[300,300],'measured_wheelbase_mm':200,'source_wheelbase_mm':210,
 'assumed_track_mm':240,'source_diameter_mm':80,'central_case_left_offset_mm':40,
 'central_case_height_mm':100,'central_case_mu':.5,'mass':'unknown; cancels in force ratios',
 'force_model':'four equal-compliance vertical supports; all wheels in contact; ideal45 roller',
 'warning':'All friction, CG and contact stiffness values are assumptions, not measurements.'}
rows=[]
for offset in (0,20,40,60):
 for height in (60,100,140):
  for friction in (.3,.5,.7):
   a=limit(offset/1000,height/1000,friction)
   row={'left_offset_mm':offset,'height_mm':height,'mu':friction,
        'equal_effort_limit_mm_s2':limit(offset/1000,height/1000,friction,'equal_effort')*1000,
        'cg_balanced_limit_mm_s2':a*1000,
        'feasible_force_limit_mm_s2':limit(offset/1000,height/1000,friction,'feasible')*1000}
   for v in (300,500,1000):row[f'cosine_T_min_{v}_ms']=math.pi*v/(2*a)
   rows.append(row)
with (out/'traction-grid.csv').open('w',newline='',encoding='utf-8-sig') as f:
 writer=csv.DictWriter(f,fieldnames=rows[0]);writer.writeheader();writer.writerows(rows)

fig,axes=plt.subplots(2,2,figsize=(11,7.6));t=np.linspace(0,3.5,1200)
for T,label,color in ((.4,'余弦 T=0.4 s（当前）','#c04849'),(1.,'余弦 T=1.0 s（候选）','#3f7d72')):
 v,a=ramp(t,1,T);axes[0,0].plot(t,v*1000,label=label,color=color)
 axes[0,1].plot(t,a*1000,label=label,color=color)
axes[0,0].plot(t,np.minimum(.3*t,1)*1000,label='线性 300 mm/s²',color='#4d6f9d')
axes[0,1].axhline(300,color='#4d6f9d',label='线性加速度上限')
axes[0,1].axhline(limit(e,h,mu)*1000,ls='--',color='#8a7353',label='假设最小范数配力边界')
axes[0,0].set(title='最高速度相同，加速度差异很大',xlabel='时间 / s',ylabel='速度 / mm/s')
axes[0,1].set(title='1000 mm/s起步：当前峰值3927 mm/s²',xlabel='时间 / s',ylabel='加速度 / mm/s²')
for ax in axes[0]:ax.grid(alpha=.2);ax.legend(fontsize=8)
for ax,acc,label in [(axes[1,0],0,'静止载荷'),(axes[1,1],math.pi/(.8),'向前加速的峰值载荷')]:
 ax.add_patch(Rectangle((-.15,-.15),.3,.3,fill=False,lw=1.7))
 shares=normals(e,h,acc)
 for i,(sx,sy,share) in enumerate(zip(SIGMA_X,SIGMA_Y,shares)):
  # Plot body left/right horizontally and front/back vertically.
  x,y=-sy*B/2,sx*L/2
  ax.scatter([x],[y],s=180+share*1200,c=['#cc5555' if share<.12 else '#587fa2'])
  ax.text(x,y-.035,f'ID{i+1}: {share*100:.1f}%',ha='center',fontsize=9)
 ax.scatter([-e],[0],c='#d79233',marker='x',s=90);ax.text(-e,.023,'假设重心',ha='center',fontsize=8)
 ax.set(title=label+'（左偏40 mm，高100 mm）',xlim=(-.22,.22),ylim=(-.20,.20))
 ax.set_aspect('equal');ax.set_xticks([-.12,0,.12],['车体左','中心','车体右']);ax.set_yticks([-.1,.1],['后','前'])
fig.suptitle('条件模型：30×30 cm车体，轮心200×240 mm；参数未经实测',fontsize=13)
fig.tight_layout();savefig('load-and-acceleration')

offsets=np.linspace(0,60,81);mus=np.linspace(.2,.8,81)
surface=np.array([[limit(x/1000,h,m)*1000 for x in offsets] for m in mus])
fig,ax=plt.subplots(figsize=(9,5.5));image=ax.pcolormesh(offsets,mus,surface,cmap='viridis',shading='auto')
contour=ax.contour(offsets,mus,surface,levels=[math.pi*v/.8 for v in (300,500,1000)],colors='white',linewidths=1)
ax.clabel(contour,fmt=lambda v:f'{v:.0f} mm/s²',fontsize=9)
ax.scatter([40],[.5],c='#ef6a56',edgecolors='white',s=80)
ax.annotate(f'假设点：{limit(e,h,mu)*1000:.0f} mm/s²\n1000 mm/s、T=0.4 s需3927',xy=(40,.5),xytext=(17,.27),color='white',arrowprops={'arrowstyle':'->','color':'white'})
fig.colorbar(image,ax=ax,label='最小范数配力的无滑移边界 / mm/s²')
ax.set(xlabel='重心向左偏移 / mm',ylabel='假设摩擦系数 μ',
       title='高度100 mm：轻载轮先限制附着\n仅此假设配力，不是所有可行配力的最大值')
fig.tight_layout();savefig('traction-envelope')

fig,axes=plt.subplots(1,3,figsize=(13,3.9));t=np.linspace(0,3,1201);kp=.02*180/math.pi
error=np.radians(3)*np.exp(-kp*t)
axes[0].plot(t,np.degrees(error),color='#587fa2')
axes[0].set(title='理想航向P：初始偏3°',xlabel='时间 / s',ylabel='剩余航向误差 / °')
for eta,label in ((1,'横向分量完全实现'),(.5,'横向分量只实现50%'),(0,'横向分量无法实现')):
 dy=(1-eta)*np.sin(error)*np.cos(error)
 y=np.cumsum(dy)*(t[1]-t[0])*1000
 axes[1].plot(t,y,label=label)
axes[1].set(title='1000 mm/s：角度回正不等于回到原直线',xlabel='时间 / s',ylabel='条件横移 / mm')
axes[1].legend(fontsize=8)
err=np.linspace(-2,2,801)
for speed in (.3,.5,1):
 actual=[]
 for angle in err:
  rpm=wheel_rpm(speed,0,float(np.clip(.02*angle,-.15,.15)))
  omega=(-rpm[0]-rpm[1]+rpm[2]+rpm[3])*(2*np.pi*R/60)/(4*.225)
  actual.append(math.degrees(omega))
 axes[2].plot(err,actual,label=f'{speed*1000:.0f} mm/s')
axes[2].plot(err,.02*err*180/math.pi,'k--',lw=1,label='未量化P输出')
axes[2].set(title='整数RPM造成航向输出台阶',xlabel='目标减实际航向 / °',ylabel='运动学指令角速度 / °/s')
axes[2].legend(fontsize=8)
for ax in axes:ax.grid(alpha=.2)
fig.suptitle('航向/执行敏感性：理想模型，不是本车实测轨迹',fontsize=12)
fig.tight_layout();savefig('heading-sensitivity')

central={mode:limit(e,h,mu,mode)*1000 for mode in ('equal_effort','cg_balanced','feasible')}
summary={'parameters':parameters,'central_case_limits_mm_s2':central,
 'central_static_load_percent':(normals(e,h)*100).tolist(),
 'central_1000_cosine_peak_load_percent':(normals(e,h,math.pi/.8)*100).tolist(),
 'requested_cosine_accel_mm_s2':{str(v):math.pi*v/.8 for v in (300,500,1000)},
 'central_T_min_ms':{str(v):math.pi*v/(2*(central['cg_balanced']/1000)) for v in (300,500,1000)},
 'kinematic_rpm':{str(v):wheel_rpm(v/1000,0,0).astype(int).tolist() for v in (300,500,1000)},
 'wheel_speed_error_at_80ms_lag_peak_RPM':3926.990817*.08/(math.pi*80/60),
 'validation':'normal force sum, force/moment balance, zero-mass dependence, null-space feasibility, symmetric speed sanity PASS'}
(out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(summary,ensure_ascii=False,indent=2))
