#!/usr/bin/env python3
"""선택형 관측 보완을 기존 입력에 연결한 반사실 판단. 미래 주행 성공 판정 아님."""
import argparse
from collections import Counter
from dataclasses import replace
import json
from pathlib import Path
import sys
import time
import zipfile

from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import observation_support, observation_lateral
from audit_corridor_runtime import digest

ROOT=Path(__file__).resolve().parents[1]


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    before=digest(args.capture)
    sources={}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            path=Path(name).resolve()
            if path.is_relative_to(ROOT) and path.suffix=='.py':sources[path.relative_to(ROOT).as_posix()]=digest(path)
    # 지연 import되는 회랑 모듈도 실행 전에 명시적으로 보존한다.
    for filename in ('corridor_observer','corner_corridor'):
        name=f'src/arena_autonomy/arena_autonomy/{filename}.py'
        sources[name]=digest(ROOT/name)
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in sources:archive.write(ROOT/name,name)
    reasons=Counter();rows=[];calls=0;changed=0;costs=[]
    with args.capture.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':
                motion=MotionConfig(**event['data']['motion_config'])
                config=replace(FollowConfig(**event['data']['follow_config']),recent_observation_enabled=True)
                ctl=LocalFollow(config)
            elif event['kind']=='reset':
                ctl.reset()
            elif event['kind']=='call':
                data,scan,h=restore_call(event['data']['input'],motion)
                scene=motion_scene(scan,h,data['reference_s'],timing_verified=data['timing_verified'],
                    epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
                began=time.perf_counter()
                output=ctl.command(scene,h,data['speed_mps'],data['side'],data['previous_steering_rad'],data['dt_s'])
                cost=(time.perf_counter()-began)*1000
                calls+=1;costs.append(cost);reasons[output[2]['reason']]+=1
                previous=event['data']['output']
                is_changed=output[0]!=previous[0] or output[1]!=previous[1]
                changed+=is_changed
                if is_changed or output[2].get('recent_observation',{}).get('required'):
                    rows.append(dict(seq=event['seq'],time_s=data['reference_s'],old_reason=previous[2]['reason'],
                        old_speed_mps=previous[0],old_steering_rad=previous[1],new_speed_mps=output[0],
                        new_steering_rad=output[1],reason=output[2]['reason'],decision_ms=cost,
                        selected=output[2].get('recent_observation_selected',False),
                        observation=output[2].get('recent_observation'),changed=is_changed))
    report=dict(scope='saved_input_counterfactual_not_closed_loop',calls=calls,changed_commands=changed,
        selected=sum(r['selected'] for r in rows),reasons=dict(reasons),maximum_decision_ms=max(costs),rows=rows,
        source_sha256=sources,input_sha256=before,input_unchanged=before==digest(args.capture),
        source_unchanged=all(digest(ROOT/name)==h for name,h in sources.items()),
        new_driving_verified=False,old_failure_changed=False,control_mode_enabled_by_default=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','source_sha256')}))


if __name__=='__main__':main()
