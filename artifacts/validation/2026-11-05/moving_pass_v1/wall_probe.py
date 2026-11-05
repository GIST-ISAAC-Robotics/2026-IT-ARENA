"""저장된 첫 시험에서 현재 외벽 적합이 거절된 지점의 실제 광선만 회수한다."""
from pathlib import Path
import json
import math
import sys
import numpy as np
from arena_autonomy import bypass_entry_trial as et
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call
from review import read,lines

def main():
    here=Path(__file__).resolve().parent;folder=here/'closed_loop_v2';run=folder/'run1';tg=read(run/'report.json')['case_window']['green_sim_s']
    events=lines(run/'follow_decisions.jsonl');header=next(events)['data'];motion=MotionConfig(**header['motion_config'])
    ctrl=BypassHandoffFollow(FollowConfig(**header['follow_config']),HandoffExecutionConfig.from_mapping(header['execution_config']),allow_simulation_trial=True)
    original=et.tracked_straight_wall;rows=[];seen=set()
    def collect(sc,cfg,side,remembered):
        fit,evidence=original(sc,cfg,side,remembered)
        if sc.t_ref_s-tg>=4.7 and sc.source_first_s not in seen:
            seen.add(sc.source_first_s)
            a=np.diff(remembered[:,1])[0]/np.diff(remembered[:,0])[0];b=remembered[0,1]-a*remembered[0,0]
            physical=b+(1 if side=='left' else -1)*cfg.wall_offset_m*math.hypot(1.,a)
            matched=[r for r in sc.rays if r.status=='return' and -.4<=r.hit_xy_m[0]<=1.8
                and abs(r.hit_xy_m[1]-a*r.hit_xy_m[0]-physical)/math.hypot(1.,a)<=2*cfg.measurement_padding_m]
            pieces=[]
            for ray in matched:
                if not pieces or ray.source_index!=pieces[-1][-1].source_index+1 or math.dist(ray.hit_xy_m,pieces[-1][-1].hit_xy_m)>.1+.035*min(np.linalg.norm(ray.hit_xy_m),np.linalg.norm(pieces[-1][-1].hit_xy_m)):pieces.append([])
                pieces[-1].append(ray)
            summary=[]
            for piece in pieces:
                indices=[r.source_index for r in piece];xx=[r.hit_xy_m[0] for r in piece]
                summary.append(dict(indices=[min(indices),max(indices)],n=len(piece),x=[min(xx),max(xx)]))
            gaps=[]
            for p,q in zip(pieces,pieces[1:]):
                left,right=p[-1],q[0];mid=sc.rays[left.source_index+1:right.source_index]
                gaps.append(dict(indices=[left.source_index,right.source_index],status=sorted(set(r.status for r in mid)),
                    distance=math.dist(left.hit_xy_m,right.hit_xy_m)))
            rows.append(dict(t=sc.t_ref_s-tg,fit=fit,reason=evidence['reason'],pieces=summary,gaps=gaps))
        return fit,evidence
    et.tracked_straight_wall=collect
    try:
        for event in events:
            if event['kind']=='reset':ctrl.reset()
            if event['kind']!='call':continue
            data=event['data']
            if data['input']['reference_s']>tg+6.1:break
            replay_call(ctrl,data['input'],motion)
    finally:et.tracked_straight_wall=original
    with (folder/'wall_probe.json').open('x',encoding='utf-8') as f:json.dump(rows,f,indent=1);f.write('\n')
    for row in rows:print(round(row['t'],3),row['fit'],row['reason'])
if __name__=='__main__':main()
