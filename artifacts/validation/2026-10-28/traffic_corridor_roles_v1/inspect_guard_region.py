"""GUARD의 횡접근 거절 뒤 실제 영역 검사 결과를 출력 연결 없이 확인한다."""
import json
from pathlib import Path
from types import SimpleNamespace
from arena_autonomy.follow_trace import freeze,restore_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.pass_release_trial import (lateral_config,motion_region,region_blocked,
    rear_fringe_risk,same_path_risk,SelectorConfig)

BASE=Path(__file__).resolve().parent
targets={round(r['t_s'],6):r for r in json.loads((BASE/'comparison/target_frames.json').read_text())
         if r['candidate'][2].get('selector_guard_candidate')}
trace=BASE.parent/'traffic_release_fringe_v1/run1_course/follow_decisions.jsonl'
results=[]
with trace.open() as stream:
    header=json.loads(next(stream))['data']
    cfg=FollowConfig(**header['follow_config'])
    motion=MotionConfig(**header['motion_config'])
    owner=SimpleNamespace(follow_config=cfg)
    for line in stream:
        event=json.loads(line)
        if event['kind']!='call':continue
        t=round(event['data']['input']['reference_s'],6)
        if t not in targets:continue
        data,scan,history=restore_call(event['data']['input'],motion)
        scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                           epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        detail=targets[t]['candidate'][2]
        guard=detail['selector_guard_candidate']
        context={}
        region=motion_region(scene,data['speed_mps'],guard['target_speed_mps'],guard['steering_rad'],
            data['previous_steering_rad'],data['dt_s'],cfg,lateral_config(cfg),context=context)
        fringe=rear_fringe_risk(owner,scene,scan,region,context)
        same=same_path_risk(owner,scene,scan,region,context,data['speed_mps'],guard['target_speed_mps'],
            guard['steering_rad'],data['previous_steering_rad'],detail)
        unknown=context['queries'][~context['free'] & ~context['masked']] if context else []
        results.append(dict(t_s=t,original_reason=detail['selector_reason'],guarded_sides=guard['guarded_sides'],
            region=region,strict_region_blocked=region_blocked(region,SelectorConfig()),
            rear_fringe=fringe,same_path=same,unknown_points=unknown,
            bounded_region_blocked=region_blocked(region,SelectorConfig(),
                                                 bounded_unknown_risk=fringe['accepted'] or same['accepted'])))
out=BASE/'guard_region.json'
if out.exists():raise FileExistsError(out)
out.write_text(json.dumps(freeze(dict(scope='fixed-input downstream diagnostic only; no command output or state restoration',
                                    results=results)),ensure_ascii=False,indent=2),encoding='utf-8')
print(json.dumps(freeze([{k:v for k,v in r.items() if k!='unknown_points'} for r in results]),ensure_ascii=False))
