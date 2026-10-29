"""짧은 경로 기하만 교체한 개루프 비교. 실행 프로세스 밖의 주행 코드는 바꾸지 않는다.

기존 CorridorObserver 시간 확인과 LocalFollow/통과 선택기를 그대로 실행한다.
새 후보를 실제 /drive에 연결하지 않으며 저장된 운동은 출력 변화와 무관하게 고정이다.
"""
import argparse
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import time
from unittest.mock import patch

import numpy as np

ROOT=Path(__file__).resolve().parents[1]
for directory in ('scripts','src/arena_autonomy','src/arena_vehicle_interface'):
    sys.path.insert(0,str(ROOT/directory))

from arena_autonomy import corridor_observer as observer
from arena_autonomy.corner_corridor import corridor_evidence
from arena_autonomy.follow_trace import replay_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from probe_short_boundary_prefix import prefix_geometry

ORIGINAL_PROPOSAL=observer.current_proposal


def short_proposal(scene,speed,side,previous,dt,config,**kwargs):
    normal=ORIGINAL_PROPOSAL(scene,speed,side,previous,dt,config,**kwargs)
    if normal['reason']!='long_outer_reference_unavailable':
        return normal
    geometry=prefix_geometry(scene.occupied_endpoints_xy_m,side,config)
    if not geometry['supported']:
        return normal
    # 이 프로세스의 함수 호출 동안만 기하 결과를 주입한다. 기본 파일/런타임 설정은 그대로다.
    with patch.object(observer,'corridor_evidence',return_value=geometry):
        result=ORIGINAL_PROPOSAL(scene,speed,side,previous,dt,
                                replace(config,max_speed_mps=min(.35,config.max_speed_mps)),**kwargs)
    if not result['point_occupancy_supported']:
        return result
    age=scene.t_ref_s-scene.source_first_s
    available=max(0.,result['observed_path_ahead_m']-config.length_m/2-config.measurement_padding_m)
    target=result['proposed_speed_mps']
    v=max(speed,target)
    budget=.06+v*(age+.10)+v*v/4
    result.update(short_prefix_shadow=True,path_budget_m=available,stop_budget_m=budget)
    if budget>available:
        result.update(point_occupancy_supported=False,proposed_speed_mps=0.,reason='short_prefix_stop_budget')
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--trace',required=True,type=Path)
    parser.add_argument('--output',required=True,type=Path)
    parser.add_argument('--until',type=float,default=120.5)
    args=parser.parse_args();args.trace=args.trace.resolve();args.output=args.output.resolve()
    args.output.mkdir(parents=True,exist_ok=False)
    counts=Counter();reasons=Counter();changes=[];first=None;maximum_ms=0.
    originals=[Path(__file__),ROOT/'scripts/probe_short_boundary_prefix.py',
        ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py',
        ROOT/'src/arena_autonomy/arena_autonomy/corridor_observer.py',
        ROOT/'src/arena_autonomy/arena_autonomy/pass_release_trial.py',
        ROOT/'src/arena_autonomy/arena_autonomy/rear_fringe_trial.py',args.trace]
    before={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in originals}
    with args.trace.open() as file:
        header=json.loads(next(file))['data']
        config=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
        follow=LocalFollow(config)
        with patch.object(observer,'current_proposal',side_effect=short_proposal):
            for line in file:
                row=json.loads(line)
                if row['kind']=='reset':
                    follow.reset();counts['reset']+=1
                    continue
                if row['kind']!='call':continue
                data=row['data'];stamp=data['input']['reference_s']
                if stamp>args.until:break
                start=time.perf_counter()
                output=replay_call(follow,data['input'],motion)
                maximum_ms=max(maximum_ms,(time.perf_counter()-start)*1000)
                counts['calls']+=1
                old=data['output'];reasons[output[2]['reason']]+=1
                different=not np.allclose(output[:2],old[:2],rtol=0,atol=1e-10)
                if output[2].get('corridor_selected'):counts['corridor_selected']+=1
                if different:
                    counts['command_changed']+=1
                    detail=output[2];guard=detail.get('selector_guard_candidate') or {}
                    record=dict(t_s=stamp,source_s=data['input']['scan']['sec']+data['input']['scan']['nanosec']*1e-9,
                        baseline=old[:2],candidate=list(output[:2]),baseline_reason=old[2]['reason'],
                        reason=detail['reason'],selector_reason=detail.get('selector_reason'),
                        corridor_selected=detail.get('corridor_selected'),corridor=detail.get('corridor_candidate'),
                        guard_accepted=guard.get('accepted'),region=guard.get('region'),
                        rear_fringe=guard.get('rear_fringe'))
                    changes.append(record)
                    if first is None:first=record
    after={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in originals}
    result=dict(scope='open-loop policy comparison with recorded sensor/motion inputs, no ROS output or closed-loop proof',
        trial='short prefix geometry injected into unchanged time/occupancy/selector machinery',
        independent_safety_node_executed=False,counts=dict(counts),reasons=dict(reasons),first_change=first,
        maximum_wall_call_ms=maximum_ms,source_sha256=before,source_unchanged=before==after)
    (args.output/'changes.json').write_text(json.dumps(freeze(changes),indent=2),encoding='utf-8')
    (args.output/'report.json').write_text(json.dumps(freeze(result),ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(freeze({k:result[k] for k in ('counts','first_change','maximum_wall_call_ms','source_unchanged')})))
    if not result['source_unchanged']:raise RuntimeError('analysis source changed during replay')


if __name__=='__main__':main()
