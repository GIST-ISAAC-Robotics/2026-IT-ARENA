#!/usr/bin/env python3
"""새 저속 직진 관측 계약의 저장 입력 대조. 제어 출력/미래 궤적을 만들지 않는다."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import zipfile

from arena_autonomy.follow_trace import restore_call, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.linear_observation import linear_observation, quiet_rotation, straight_path_supported
from arena_autonomy.bounded_recovery import recovery_evidence
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest
from summarize_corner_trace import summarize

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
            if path.is_relative_to(ROOT) and path.suffix=='.py': sources[path.relative_to(ROOT).as_posix()]=digest(path)
    for name in ('tests/test_linear_observation.py','tests/test_bounded_recovery.py'):
        path=ROOT/name
        sources[path.relative_to(ROOT).as_posix()]=digest(path)
    with zipfile.ZipFile(args.output/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in sources: archive.write(ROOT/name,name)
    with args.capture.open() as stream:
        integrity=summarize(json.loads(line) for line in stream)
    rows=[]
    seen=set()
    with args.capture.open() as stream:
        for line in stream:
            event=json.loads(line)
            if event['kind']=='header':
                motion=MotionConfig(**event['data']['motion_config'])
                config=FollowConfig(**event['data']['follow_config'])
            if event['kind']!='call' or event['data']['output'][2]['reason']!='follow_front_unobserved': continue
            data,scan,history=restore_call(event['data']['input'],motion)
            key=(scan.header.stamp.sec,scan.header.stamp.nanosec)
            if key in seen: continue
            seen.add(key)
            scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
                epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
            result=linear_observation(scene,max(0.,data['speed_mps']),length=config.length_m,width=config.width_m,
                standstill_gap=config.standstill_gap_m,measurement_padding=config.measurement_padding_m,
                object_motion_bound=config.object_motion_bound_mps)
            path,info=local_path(scene.occupied_endpoints_xy_m,data['side'],config.wall_offset_m)
            path_ok=path is not None and straight_path_supported(path,result.get('required_distance_m',2.5))
            bounded=recovery_evidence(scene,history,data['speed_mps'],data['previous_steering_rad'],path,config)
            rows.append(dict(seq=event['seq'],time_s=data['reference_s'],observation=result,
                bounded_recovery=bounded,
                quiet_yaw=quiet_rotation(history,data['reference_s']),previous_steering_rad=data['previous_steering_rad'],
                current_simple_path_supported=bool(path_ok),path_info=info,measured_speed_mps=data['speed_mps']))
    report=dict(scope='saved_unknown_scenes_linear_observation_only',capture_integrity=integrity,
        scenes=len(rows),observation_supported=sum(r['observation']['supported'] for r in rows),
        reasons=dict(Counter(r['observation']['reason'] for r in rows)),
        all_prerequisites=sum(r['observation']['supported'] and r['quiet_yaw'] and
            r['measured_speed_mps']>=0 and abs(r['previous_steering_rad'])<=.001 and r['current_simple_path_supported'] for r in rows),
        bounded_prerequisites=sum(r['bounded_recovery']['eligible'] for r in rows),
        bounded_reasons=dict(Counter(r['bounded_recovery']['reason'] for r in rows)),
        rows=rows,input_sha256=before,source_sha256=sources,input_unchanged=before==digest(args.capture),
        source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()),runtime_changed=False,driving_authorized=False)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rows','source_sha256','capture_integrity')}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 2


if __name__=='__main__':
    raise SystemExit(main())
