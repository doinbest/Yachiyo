"""Offline analytical/event model, not firmware-in-loop or measured vehicle timing.
Route phases are exported from the current C builder. Station mechanics follow
GrabTask/contest_mission, with uncertain vision/X/feedback varied explicitly.
No serial/probe/device APIs. Run: python output/mission-time-20261014/simulate.py
"""
import csv, json, math
from pathlib import Path

HERE=Path(__file__).resolve().parent
DT=.01
def trapezoid(d,v,a):
    d=abs(d)
    return 2*math.sqrt(d/a) if d<v*v/a else d/v+v/a

def linear(d,vmax,stop=.2,v0=0):
    # 1D ideal plant approximation of distance_command, 30ms feedback age.
    # Includes source's 0.25s response reserve and 10mm arrival tolerance.
    t=0.;v=v0;left=d;peak=v
    while left>10 and t<200:
        brake=math.sqrt((300*.28)**2+600*max(0,left-5))-300*.28
        want=min(vmax,brake,2.5*left,50 if left<=20 else vmax)
        v+=max(-300*DT,min(300*DT,want-v))
        left-=v*DT;t+=DT;peak=max(peak,v)
    return t+v/300+stop,peak

def turn(degrees,stop):
    e=abs(math.remainder(degrees,360));t=0
    while e>2 and t<30:
        ramp=.5-.5*math.cos(math.pi*min(t/.5,1))
        omega=min(100,.02*180/math.pi*e)*ramp
        e-=omega*DT;t+=DT
    return t+stop

def path_points(row):
    kind=int(row['kind']);sx=row['x0'];sy=row['y0']
    points=[(sx,sy,180 if kind==1 else 90)]
    points.append((2050,350,180) if kind==1 else (1300,2050,90) if kind==2 else (1200,1950,90))
    for i in range(1,21):
        u=i/20;b=30*math.sin(math.pi*u)
        points.append((2050-b,350+100*u,180-90*u) if kind==1 else
                      (1300-100*u,2050-b,90-90*u) if kind==2 else
                      (1200+b,1950+100*u,90-90*u))
    points.append((row['x1'],row['y1'],row['yaw1']))
    return points

def curve(row,vmax,stop):
    p=path_points(row);s=0.;v=0.;t=0.;segments=[]
    for a,b in zip(p,p[1:]):
        length=math.hypot(b[0]-a[0],b[1]-a[1]);gradient=abs(b[2]-a[2])/length
        cap=min(60,65/gradient) if gradient else 60
        segments.append((s,s+length,cap));s+=length
    end=segments[-2][1]
    s=0
    # Ideal path following; no yaw error, slip or vector curvature penalty.
    while s<end+5:
        want=vmax
        for a,b,cap in segments[:-1]:
            if b>=s:want=min(want,math.sqrt(cap*cap+300*max(0,a-s)))
        v+=max(-300*DT,min(300*DT,want-v));s+=v*DT;t+=DT
        if t>200:raise RuntimeError('path')
    tail,_=linear(segments[-1][1]-s,vmax,stop,v)
    return t+tail

ROWS=[]
for row in csv.DictReader((HERE/'route.csv').open()):
    ROWS.append({k:(v if k=='phase' else float(v)) for k,v in row.items()})

def route(vmax,stop):
    out=[]
    for r in ROWS:
        if r['kind']:seconds=curve(r,vmax,stop)
        elif not r['speed']:seconds=turn(r['yaw1']-r['yaw0'],stop)
        else:seconds=linear(r['length'],min(vmax,r['speed']),stop)[0]
        out.append(dict(r,seconds=seconds))
    return out

class Model:
    def __init__(self,x,visual,track,feedback,stop,qr):
        self.x=x;self.visual=visual;self.track=track;self.feedback=feedback;self.stop=stop;self.qr=qr
        self.rows=[];self.tray=0
    def add(self,stage,action,t):self.rows.append(dict(stage=stage,action=action,seconds=t))
    def z(self,d):return 0 if abs(d)<.01 else trapezoid(d,111.11111111,2222.2222)+self.feedback
    def xm(self,d):return 0 if abs(d)<1 else trapezoid(d,58.8,142.37288)+self.feedback
    def base(self):return trapezoid(130,360,6*20000/236)+self.feedback
    def index(self,angle,forced=False):
        delta=angle-self.tray if forced else math.remainder(angle-self.tray,360)
        self.tray=angle%360
        return .12 if abs(delta)<.01 else trapezoid(delta,180,6*20000/248)+self.feedback
    def local(self,stage,a,b,previous_x=0,depart=False):
        # Z is already at0 after the child task; X must retract before translation.
        self.add(stage,'X retract before local movement',self.xm(previous_x))
        travel=linear(abs(b-a),50,self.stop)[0] if a!=b else self.stop
        self.add(stage,'local move (departure Base turn overlaps)',max(travel,self.base()) if depart else travel)
    def raw(self,batch):
        st=f'batch{batch+1}_raw'
        for i in range(3):
            tt=self.index(i*90)
            self.add(st,'track/descent, close, lift Z then retract X; parallel ID8',
                     max(tt,max(self.track,self.z(50))+.3+self.z(50)+self.xm(self.x)))
            self.add(st,'Base to car, descend40, release, lift40',self.base()+2*self.z(40)+.3)
            if i<2:self.add(st,'return observation Base and X',self.base()+self.xm(self.x))
        self.add(st,'handoff/start evidence allowance',.3)
    def place(self,batch,rings,temporary=False):
        st=f'batch{batch+1}_'+('temp' if temporary else 'rough_place')
        last=0;x=0
        for i,ring in enumerate(rings):
            pos=(ring-2)*150
            self.local(st,last,pos,x);last=pos
            prealign=not(temporary and batch==1)
            if prealign:self.add(st,'empty-claw visual align including stop/capture',max(self.visual,self.xm(self.x)))
            self.add(st,'retract X / ID8 in parallel',max(self.xm(self.x) if prealign else 0,self.index(i*90)))
            clear=30 if i<2 else 0
            self.add(st,'Base car, open dwell, descend40, close, lift+X30, Base work',
                     2*self.base()+.3+self.z(40)+.3+max(self.z(40),self.xm(clear)))
            if prealign:self.add(st,'restore saved X',self.xm(self.x-clear))
            else:self.add(st,'held visual align',max(self.visual,self.xm(self.x-clear)))
            depth=100 if temporary and batch else 130
            self.add(st,'lower to release, release dwell, raise Z to0',2*self.z(depth)+.3)
            x=self.x
        if temporary:self.local(st,last,0,x,True)
        return last,x
    def retrieve(self,batch,rings,last,x):
        st=f'batch{batch+1}_rough_retrieve'
        for i,ring in enumerate(rings):
            pos=(ring-2)*150
            self.local(st,last,pos,x);last=pos
            if i==2:self.add(st,'slot3 clearance before acquisition',self.index(270,True))
            self.add(st,'visual align and final descent130, close',max(self.visual,self.xm(self.x))+self.z(130)+.3)
            if i==2:
                self.add(st,'raise Z to0',self.z(130))
                self.add(st,'X retract then Base; parallel ID8 return',max(self.xm(self.x)+self.base(),self.index(180,True)))
            else:
                self.add(st,'Z to0 then X; parallel ID8, then Base',max(self.z(130)+self.xm(self.x),self.index(i*90))+self.base())
            self.add(st,'deposit40 and raise40',2*self.z(40)+.3)
            if i<2:
                self.add(st,'return observation Base and X',self.base()+self.xm(self.x));x=self.x
            else:x=0
        self.local(st,last,0,x)
    def run(self,speed):
        for row in route(speed,self.stop):self.add('route',row['phase'],row['seconds'])
        self.add('start_scan_handoff','initial evidence, QR acquisition/display and residual pose joins',self.qr+2)
        for b,rings in enumerate(([1,2,3],[2,3,1])):
            self.raw(b)
            last,x=self.place(b,rings)
            self.retrieve(b,rings,last,x)
            self.place(b,[1,2,3] if b==0 else [2,1,3],True)
        totals={}
        for r in self.rows:totals[r['stage']]=totals.get(r['stage'],0)+r['seconds']
        return dict(seconds=sum(totals.values()),stages=totals)

if __name__=='__main__':
    scenarios={'smooth':dict(x=30,visual=1.5,track=2,feedback=.23,stop=.12,qr=.7),
               'central':dict(x=60,visual=2.5,track=4,feedback=.30,stop=.20,qr=1.5),
               'slower':dict(x=100,visual=4,track=7,feedback=.45,stop=.35,qr=3)}
    results={}
    for name,args in scenarios.items():
        model=Model(**args);results[name]=dict(assumptions=args,**model.run(1000))
        with (HERE/(name+'-events.csv')).open('w',newline='',encoding='utf-8-sig') as f:
            w=csv.DictWriter(f,fieldnames=['stage','action','seconds']);w.writeheader();w.writerows(model.rows)
    results['speed_comparison']={str(v):Model(**scenarios['central']).run(v)['seconds'] for v in (200,300,500,1000)}
    results['route_at_1000']=route(1000,.2)
    results['route_length_mm']=sum(r['length'] for r in ROWS)
    (HERE/'result.json').write_text(json.dumps(results,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:v for k,v in results.items() if k!='route_at_1000'},indent=2))
