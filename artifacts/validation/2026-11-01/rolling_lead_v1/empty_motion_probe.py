"""정지 시설만 있는 실제 단독 입력에서 새 이동 근거가 우회 여유를 잘못 풀 수 있는지 본다."""
from collections import Counter
import json
from pathlib import Path
import sys
from time import perf_counter
import numpy as np
from recent_forward_candidate import RecentFollow
from arena_autonomy.bypass_handoff import HandoffExecutionConfig
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene

HERE=Path(__file__).resolve().parent
SOURCE=HERE.parent/'race_acceleration_v1/run1'


def run():
    with (HERE/'run1/follow_decisions.jsonl').open() as f:
        execution=HandoffExecutionConfig.from_mapping(json.loads(next(f))['data']['execution_config'])
    counts=Counter();key=None;cost=[];changes=[]
    with (SOURCE/'follow_decisions.jsonl').open() as f:
        for e in map(json.loads,f):
            d=e['data']
            if e['kind']=='header':
                cfg=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
                control=RecentFollow(cfg,execution,allow_simulation_trial=True)
            if e['kind']=='reset':control.reset();key=None
            if e['kind']!='call':continue
            x=d['input'];scan=x['scan'];k=(x['epoch'],scan['frame_id'],scan['sec'],scan['nanosec'])
            if k==key:continue
            key=k
            args,scan,hist=restore_call(x,motion)
            scene=motion_scene(scan,hist,args['reference_s'],timing_verified=args['timing_verified'],
                               epoch=args['epoch'],rear_half_angle_deg=args['rear_half_angle_deg'])
            start=perf_counter();monitor=control.monitor.update(scene,hist);cost.append(perf_counter()-start)
            v,angle,q=d['output']
            base,base_info=super(RecentFollow,control)._follow_entry_space_current(scene,(v,angle,q),x['previous_steering_rad'],v)
            new,info=control._follow_entry_space_current(scene,(v,angle,q),x['previous_steering_rad'],v)
            counts['fresh_scans']+=1;counts['positive_motion_hint']+=int(control.monitor.recent.rate>0)
            counts['higher_reserve_cap']+=int(new>base+1e-7)
            if new>base+1e-7:
                changes.append(dict(t=x['reference_s'],base=base,new=new,rate=control.monitor.recent.rate,
                    reason=info['reason'],normal=v,wall=q.get('path_wall')))
    return dict(counts=dict(counts),changes=changes,monitor_ms_percentiles=(1000*np.percentile(cost,[50,95,100])).tolist(),
        scope='실제 단독 관측의 스캔별 첫 호출. 기록된 정상 경로에서 우회 여유 함수만 대조했으며 전체 G1/G2 폐루프 결과는 아님. 비용은 해당 PC의 모니터 전체이며 Jetson 50 Hz 증명이 아님.')


if __name__=='__main__':
    result=run()
    with (HERE/'empty_motion_probe_corrected.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
    print(result['counts'],result['monitor_ms_percentiles'])
