"""Read-only route export and approximate speed-reference analysis; no hardware I/O.

Straight model: ideal velocity tracking, current source P/braking/creep/rate limits.
Feedback age is represented as a braking reserve, not delayed position simulation.
Times include configurable stop evidence, but exclude QR, mechanics and vision.
Curve and turn times are separate estimates; this is not firmware-in-loop.
"""
import csv, io, json, math, subprocess, re
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DT = .001

def ideal(d, vmax, up, down):
    peak = min(vmax, math.sqrt(2*d/(1/up+1/down)))
    cruise = max(0, d-peak*peak/(2*up)-peak*peak/(2*down))
    return dict(peak=peak, seconds=peak/up+peak/down+cruise/peak)

def reference(d, vmax, up, down, age=.1, stop=.25, initial=0):
    x, v, t, peak = 0., initial, 0., initial
    delay = .25+age
    while d-x > 10 and t < 120:
        left = d-x
        available = max(0,left-5)
        brake = math.sqrt((down*delay)**2+2*down*available)-down*delay
        want = min(vmax,brake,2.5*left,50 if left<=20 else vmax)
        rate = down if want<v else up
        v += max(-rate*DT,min(rate*DT,want-v))
        x += v*DT; t += DT; peak=max(peak,v)
    return dict(peak=peak, seconds=t+v/down+stop)

def turn(degrees, stop=.25):
    e=abs(math.remainder(degrees,360)); t=0.
    while e>2 and t<30:
        ramp=.5-.5*math.cos(math.pi*min(t/.5,1))
        omega=min(100,.02*180/math.pi*e)*ramp
        e-=omega*DT;t+=DT
    return t+stop

def parameters(r):
    if r['speed']<=50:return 300,300,50
    if r['length']<300:return 500,300,300
    if r['leg'] in (4,9):return 800,300,1000
    if r['leg']==3:return 800,300,700
    if r['leg'] in (1,2):return 1000,300,1000
    return 800,300,800

def logged_tests():
    accepted={}; results=[]
    path=ROOT/'.embeddedskills/logs/serial/20261015-204133-c2da65.jsonl'
    for line in path.read_text(encoding='utf-8').splitlines():
        event=json.loads(line)
        if event.get('kind')!='rx':continue
        text=event.get('text','')
        fields=dict(re.findall(r'(\w+)=([^ ;]+)',text))
        if text.startswith('OK chassis test reason=accepted'):accepted[fields['id']]=fields
        if text.startswith('OK chassis test_result') and fields.get('state')=='3' and 'decel_mm_s2' in fields:
            a=accepted.get(fields['id'])
            if a:results.append(dict(request=a,result=fields,time=event['time']))
    return results

def main():
    text=subprocess.check_output([str(HERE/'export_route.exe')],text=True)
    rows=[]
    for record in csv.DictReader(io.StringIO(text)):
        r={k:(v if k=='phase' else float(v)) for k,v in record.items()}
        if not r['speed']:
            r['turn_estimate_s']=turn(r['yaw1']-r['yaw0']);rows.append(r);continue
        if r['kind']:
            # All pre-tail segment limits are60. The first segment's a.s=0
            # means path_command also caps the750mm straight approach at60.
            r['curve_reference_limit']=60
            r['curve_time_estimate_s']=r['length']/60+.6
            rows.append(r);continue
        up,down,vmax=parameters(r)
        r.update(accel=up,decel=down,target=vmax,ideal=ideal(r['length'],vmax,up,down))
        r['reference_scenarios']=[dict(age=age,stop=stop,**reference(r['length'],vmax,up,down,age,stop))
                                  for age,stop in ((.04,.13),(.1,.25),(.18,.47))]
        r['baseline_reference']=reference(r['length'],r['speed'],300,300)
        rows.append(r)
    tests=logged_tests()
    for t in tests:
        a=t['request']
        t['reference_model']=reference(float(a['distance_mm']),float(a['speed_mm_s']),float(a['accel_mm_s2']),float(a['decel_mm_s2']))
    output=dict(assumptions={'profile':'linear','start':[2250,150,180],'qr_y':1200,
                            'type':'approximate speed-reference model, not full vehicle simulation',
                            'proposed_deceleration':300,'turn_limit_deg_s':100},rows=rows,measured_tests=tests)
    (HERE/'analysis.json').write_text(json.dumps(output,ensure_ascii=False,indent=2),encoding='utf-8')
    for r in rows:
        if not r['speed']: print('TURN',int(r['leg']),round(r['turn_estimate_s'],2));continue
        if r['kind']: print('CURVE',int(r['leg']),r['length'],round(r['curve_time_estimate_s'],2));continue
        cases=r['reference_scenarios']
        print(int(r['leg']),r['phase'],r['length'],r['accel'],r['decel'],r['target'],
              'ideal',round(r['ideal']['peak']),round(r['ideal']['seconds'],2),
              'ref peak',[round(c['peak']) for c in cases],
              'ref time',[round(c['seconds'],2) for c in cases])
    print('TEST_MODEL',[(t['request']['id'],t['request']['distance_mm'],t['request']['accel_mm_s2'],
                       t['result']['elapsed_ms'],round(t['reference_model']['seconds'],2),
                       t['result']['peak_mm_s'],round(t['reference_model']['peak'])) for t in tests])
    translation=sum(r.get('reference_scenarios',[{'seconds':r.get('curve_time_estimate_s',0)}])[1 if 'reference_scenarios' in r else 0]['seconds'] for r in rows if r['speed'])
    turns=sum(r.get('turn_estimate_s',0) for r in rows)
    baseline=sum(r.get('baseline_reference',{'seconds':r.get('curve_time_estimate_s',0)})['seconds'] for r in rows if r['speed'])
    print('TOTAL',{'translation':translation,'turns':turns,'route':translation+turns,'baseline_translation':baseline,'saved':baseline-translation})

if __name__=='__main__':main()
