"""접촉 법선/힘의 크기만 집계한다. 재배열된 collision 이름으로 힘 방향을 인증하지 않는다."""
import argparse
from collections import defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics


def vectors(contact):
    normals=contact.get('normal',[])
    wrenches=contact.get('wrench',[])
    if not normals or len(normals)!=len(wrenches):
        return None
    result=[]
    for n,w in zip(normals,wrenches):
        f=w.get('body1Wrench',{}).get('force')
        if f is None:
            return None
        n=[float(n.get(k,0.)) for k in ('x','y','z')]
        f=[float(f.get(k,0.)) for k in ('x','y','z')]
        if not all(math.isfinite(x) for x in n+f):
            raise ValueError('nonfinite contact vector')
        norm=math.sqrt(sum(x*x for x in n))
        if not .999 < norm < 1.001:
            raise ValueError('contact normal is not unit length')
        normal_magnitude=abs(sum(a*b for a,b in zip(n,f)))/norm
        result.append(dict(normal_force_magnitude_n=normal_magnitude,
            absolute_normal_z=abs(n[2])/norm,raw_force_z_n=f[2]))
    return result


def analyze(folder):
    trace=json.loads((folder/'trace.json').read_text(encoding='utf-8'))
    r=next(r for r in trace if r.get('elapsed_since_green_s') is not None)
    green=r['t']-r['elapsed_since_green_s']
    pairs=defaultdict(lambda:dict(points=0,events=0,normal=[],vertical=[],raw_z=[],unavailable=0))
    samples=0
    raw=folder/'wheel_contacts.jsonl'
    for line in raw.open(encoding='utf-8'):
        row=json.loads(line);stamp=row['header']['stamp']
        at=float(stamp.get('sec',0))+float(stamp.get('nsec',0))*1e-9
        if not green+6 <= at <= green+11.9:
            continue
        samples+=1
        for c in row.get('contact',[]):
            key=c['collision1']['name']+' | '+c['collision2']['name']
            p=pairs[key];p['events']+=1;data=vectors(c)
            if data is None:
                p['unavailable']+=1;continue
            p['points']+=len(data)
            p['normal'].append(sum(x['normal_force_magnitude_n'] for x in data))
            p['vertical'] += [x['absolute_normal_z'] for x in data]
            p['raw_z'].append(sum(x['raw_force_z_n'] for x in data))
    def stats(xs):
        return dict(min=min(xs),median=statistics.median(xs),max=max(xs)) if xs else None
    return dict(run=str(folder),sha256=hashlib.sha256(raw.read_bytes()).hexdigest(),
        since_green_s=[6,11.9],messages=samples,
        pairs={key:dict(events=p['events'],points=p['points'],unavailable=p['unavailable'],
            normal_force_magnitude_n=stats(p['normal']),absolute_normal_z=stats(p['vertical']),
            raw_message_body1_z_n=stats(p['raw_z'])) for key,p in pairs.items()},
        scope='Sign-independent normal magnitude per observed contact pair; not signed wheel support, time-average load, or missing-contact-as-zero.')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run',type=Path);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();result=analyze(args.run)
    with args.output.open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2,allow_nan=False)
    print(json.dumps(result,indent=2))
