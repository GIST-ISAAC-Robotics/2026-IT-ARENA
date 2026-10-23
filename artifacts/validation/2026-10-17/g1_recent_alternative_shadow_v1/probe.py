"""실패 시점에서 허용 조향 변화/목표 속도의 관측 가능성만 대조한다.

새 제어기나 주행이 아니며 각도가 통과해도 도로/경로/곡선 연속성은 승인하지 않는다.
실제 모터축 속도, 직전 조향, 점유, 광선 나이 및 보호식은 그대로 둔다.
"""
from collections import Counter
import json
from pathlib import Path
import sys
import zipfile

from arena_autonomy.follow_trace import restore_call, freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.observation_support import RecentScenes, sweep_support
from arena_autonomy.corner_corridor import corridor_evidence
from audit_corridor_runtime import digest

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
CAPTURE=ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'


def main():
    assert not (OUT/'report.json').exists()
    before=digest(CAPTURE)
    cache=RecentScenes();rows=[];last_source=None;first_reject=None;stopped_collected=0
    for line in CAPTURE.open():
        event=json.loads(line)
        if event['kind']=='header':
            cfg=FollowConfig(**event['data']['follow_config'])
            motion=MotionConfig(**event['data']['motion_config'])
        if event['kind']=='reset':cache.reset()
        if event['kind']!='call':continue
        d,raw,h=restore_call(event['data']['input'],motion)
        if d['reference_s']<62.5:continue
        scene=motion_scene(raw,h,d['reference_s'],timing_verified=d['timing_verified'],
                           epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        sources,_=cache.observe(scene,h,transport=True)
        reason=event['data']['output'][2]['reason']
        fresh=scene.source_first_s!=last_source
        last_source=scene.source_first_s
        take=event['seq'] in (3151,3156)
        if reason=='follow_observation_unconfirmed':first_reject=event['seq']
        if first_reject is not None and reason=='follow_boundary_unconfirmed' and fresh and stopped_collected<3:
            take=True;stopped_collected+=1
        if take:
            cases=[]
            # 실제 입력의 signed 속도를 보존한다. 역행이면 그 자체로 거절된다.
            previous=d['previous_steering_rad']
            for delta in (-.06,-.03,0.,.03,.06):
                angle=previous+delta
                if abs(angle)>cfg.max_steering_rad:continue
                for target in (.05,.1,.2):
                    support=sweep_support(sources,d['speed_mps'],target,angle,previous,cfg,
                        protected_sensor_pose=(motion.lidar_x_m,motion.lidar_y_m,motion.lidar_yaw_rad))
                    cases.append(dict(steering=angle,delta=delta,target=target,**support))
            walls={}
            for side in ('left','right'):
                path,info=local_path(scene.occupied_endpoints_xy_m,side,cfg.wall_offset_m)
                walls[side]=dict(path_available=path is not None,info=freeze(info))
            geometry=corridor_evidence(scene.occupied_endpoints_xy_m,d['side'],cfg)
            rows.append(dict(seq=event['seq'],time_s=d['reference_s'],source_first_s=scene.source_first_s,
                recorded_reason=reason,recorded_speed=event['data']['output'][0],
                actual_motor_speed=d['speed_mps'],previous_steering=previous,side=d['side'],
                recorded_boundary=event['data']['output'][2],walls=walls,
                corridor={k:freeze(v) for k,v in geometry.items() if k!='path'},cases=cases))
        if stopped_collected>=3:break
    names={}
    for module in tuple(sys.modules.values()):
        filename=getattr(module,'__file__',None)
        if filename:
            p=Path(filename).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':names[p.relative_to(ROOT).as_posix()]=digest(p)
    with zipfile.ZipFile(OUT/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in names:archive.write(ROOT/name,name)
        archive.write(__file__,'probe.py')
    report=dict(scope='fixed_actual_motion_alternative_observation_only',runtime_changed=False,
        driving_authorized=False,road_and_path_feasibility_verified=False,rows=rows,
        source_sha256=names,input_sha256=before,input_unchanged=before==digest(CAPTURE),
        source_unchanged=all(digest(ROOT/n)==v for n,v in names.items()))
    (OUT/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    for row in rows:
        print(json.dumps(dict(seq=row['seq'],time_s=row['time_s'],speed=row['actual_motor_speed'],
            previous=row['previous_steering'],supported=sum(x['supported'] for x in row['cases']),
            tested=len(row['cases']),reasons=dict(Counter(x['reason'] for x in row['cases'])),
            walls=row['walls'],corridor=row['corridor'])))
    return 0 if rows and report['input_unchanged'] and report['source_unchanged'] else 1

if __name__=='__main__':raise SystemExit(main())
