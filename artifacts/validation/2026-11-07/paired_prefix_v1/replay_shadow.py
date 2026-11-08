"""소스 변경 없이 기하 후보를 저장 입력 전체에 연결해 비교한다. 폐루프 주행 아님."""
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_events, replay_call
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy import corridor_observer as observer

HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('paired_prefix_shadow_probe',HERE/'probe.py')
probe=importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
old=observer.short_prefix_evidence


def candidate(points,side,config):
    # 현재 선택형 직각 경로가 켜진 구성에만 새 역할 조합을 조사한다.
    return probe.candidate(points,side,config,allow_long_preferred=config.piecewise_wall_trial)


result={}
for name,relative in [('old_off','course_section_v1/run1'),('corner_on','piecewise_corner_v1/run1')]:
    events=[json.loads(s) for s in (HERE.parent/relative/'follow_decisions.jsonl').read_text().splitlines()]
    baseline=replay_events(iter(events))
    header=events[0]['data']
    ctl=LocalFollow(FollowConfig(**header['follow_config']))
    motion=MotionConfig(**header['motion_config'])
    observer.short_prefix_evidence=candidate
    rows=[]
    for e in events[1:]:
        if e['kind']=='reset':ctl.reset()
        elif e['kind']=='call':
            q=e['data'];out=replay_call(ctl,q['input'],motion);previous=q['output']
            rows.append(dict(t=q['input']['reference_s'],old_speed=previous[0],speed=out[0],
                old_steer=previous[1],steer=out[1],reason=out[2].get('reason'),
                selected=out[2].get('boundary_selection'),corridor=out[2].get('corridor')))
    observer.short_prefix_evidence=old
    summary=dict(baseline={k:baseline[k] for k in ('calls','matched')},
        command_changes=sum(abs(r['old_speed']-r['speed'])>1e-9 or abs(r['old_steer']-r['steer'])>1e-9 for r in rows),
        after31_positive=sum(r['t']>=31. and r['speed']>0 for r in rows),
        first_new_positive=next((r for r in rows if r['old_speed']==0 and r['speed']>0),None),
        last=rows[-1])
    result[name]=summary
    (HERE/(name+'_shadow_rows.json')).write_text(json.dumps(rows,indent=1,default=str),encoding='utf-8')
(HERE/'replay_shadow.json').write_text(json.dumps(result,indent=2,default=str),encoding='utf-8')
print(json.dumps(result,default=str))
