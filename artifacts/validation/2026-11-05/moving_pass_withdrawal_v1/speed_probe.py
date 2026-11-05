"""현재 대응 반환의 센서 운동 보정 속도. 정답 위치·각본은 계산에 사용하지 않는다."""
from collections import deque
from pathlib import Path
import sys
import numpy as np
from prepare import HERE,ROOT,save,read
from assess import rows
from arena_autonomy.moving_pass import MovingTargetTrack
from arena_autonomy.scene_motion import transport_points
from arena_autonomy.follow_trace import replay_events

def probe(run):
    events=rows(run/'follow_decisions.jsonl')
    green=read(run/'report.json')['case_window']['green_sim_s']
    observed=[];window=deque(maxlen=5);last=None
    original=MovingTargetTrack.update
    def wrapped(self,scene,history):
        nonlocal last
        result=original(self,scene,history)
        source=scene.source_first_s
        if source==last:return result
        last=source
        if self.current is None or not result.get('observed'):
            window.clear();return result
        window.append((source,scene.t_ref_s,np.median(self.current,axis=0)))
        if len(window)==5:
            q=np.array([transport_points(p[None],history,ref,scene.t_ref_s)[0] for _,ref,p in window])
            dt=source-window[0][0];v=(q[-1,0]-q[0,0])/dt;upper=v+.06/dt
            bound=float(self.current[:,0].min())-.25-.03-.75*(scene.t_ref_s-source)
            observed.append(dict(t=round(scene.t_ref_s-green,3),estimate=float(v),upper=float(upper),
                front_bound=bound,points=len(self.current),rear=self.detail.get('rear_clear')))
        return result
    MovingTargetTrack.update=wrapped
    try:r=replay_events(events)
    finally:MovingTargetTrack.update=original
    return dict(calls=r['calls'],mismatches=len(r['mismatches']),rows=observed)

if __name__=='__main__':
    name=sys.argv[1]
    paths={'constant':HERE.parent/'moving_pass_v1/closed_loop_side_pass_2/run1',
           'accelerating':HERE/'baseline_r12/run1'}
    out={k:probe(p) for k,p in paths.items()}
    save(HERE/(name+'.json'),out)
    for k,v in out.items():print(k,v['calls'],v['mismatches'],[(r['t'],round(r['upper'],3),round(r['front_bound'],3)) for r in v['rows'] if r['t']<6.5])
