"""저장 궤적의 같은 입출구 단면 비교. 평가 전용이며 제어기에 좌표를 주지 않는다."""
import hashlib,json,math
from pathlib import Path
import numpy as np

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
CASES={
    'r11_main':'artifacts/validation/2026-11-07/junction_handoff_v1/run1',
    'r25_faster_main':'artifacts/validation/2026-11-08/solo_gap_envelope_connected_v3/run1',
    'r50_shortcut':'artifacts/validation/2026-11-09/p22_route_connected_v1/run1',
}
Y=5.65


def crossing(poses,after,xlo,xhi,direction):
    for a,b in zip(poses,poses[1:]):
        if b[0]<=after or not direction*(b[2]-a[2])>0:continue
        k=(Y-a[2])/(b[2]-a[2])
        if not 0<=k<1:continue
        x=a[1]+k*(b[1]-a[1]);t=a[0]+k*(b[0]-a[0])
        if xlo<x<xhi and t>after:
            return dict(time_s=t,bracket_s=[a[0],b[0]],xy_m=[x,Y])
    return None


def main():
    rows=[];inputs={}
    for name,rel in CASES.items():
        d=ROOT/rel;p=d/'ego_poses.jsonl'
        digest=hashlib.sha256(p.read_bytes()).hexdigest();inputs[p.relative_to(ROOT).as_posix()]=digest
        poses=np.array([json.loads(l) for l in p.open()])
        enter=crossing(poses,-math.inf,5.635,6.645,-1)
        leave=crossing(poses,enter['time_s'],3.81,4.81,1) if enter else None
        q=dict(case=name,entry=enter,exit=leave,finished=bool(enter and leave))
        if enter and leave:
            t0,t1=enter['time_s'],leave['time_s'];middle=poses[(poses[:,0]>t0)&(poses[:,0]<t1),1:3]
            path=np.vstack((enter['xy_m'],middle,leave['xy_m']))
            q.update(duration_s=t1-t0,distance_m=float(np.linalg.norm(np.diff(path,axis=0),axis=1).sum()),
                     duration_bracket_s=[leave['bracket_s'][0]-enter['bracket_s'][1],leave['bracket_s'][1]-enter['bracket_s'][0]])
        q['speed_profile']=json.loads((d/'report.json').read_bytes()).get('speed_profile')
        rows.append(q)
        assert hashlib.sha256(p.read_bytes()).hexdigest()==digest
    out=dict(scope='saved independent single runs, equal physical entry/exit cross-sections; not paired repeatability or full lap benefit',
             gates=dict(y_m=Y,entry_x_m=[5.635,6.645],entry_direction='south',exit_x_m=[3.81,4.81],exit_direction='north'),
             inputs=inputs,rows=rows)
    (A/'comparison.json').write_text(json.dumps(out,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps(out,ensure_ascii=False,indent=2))


if __name__=='__main__':main()
