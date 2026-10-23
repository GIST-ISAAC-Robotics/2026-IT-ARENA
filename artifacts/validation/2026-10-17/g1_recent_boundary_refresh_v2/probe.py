"""최초 정지 뒤 경계 갱신 거절을 같은 유효 기억 안에서 분해한다. 출력 변경 없음."""
import json
from pathlib import Path
import sys
import zipfile

import numpy as np
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,boundary_support,drop_protrusions,path_agrees
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene,transport_points
from review_corridor_handoffs import geometric_difference
from audit_corridor_runtime import digest

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
CAPTURE=ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'

def main():
    assert not (OUT/'report.json').exists()
    before=digest(CAPTURE);selected={}
    for line in CAPTURE.open():
        event=json.loads(line)
        if event['kind']=='header':
            cfg=FollowConfig(**event['data']['follow_config']);motion=MotionConfig(**event['data']['motion_config'])
        elif event['kind']=='call' and event['seq'] in (3156,3161):selected[event['seq']]=event
    old,new=selected[3156],selected[3161]
    d,raw,h=restore_call(new['data']['input'],motion)
    scene=motion_scene(raw,h,d['reference_s'],timing_verified=d['timing_verified'],
        epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
    original=old['data']['output'][2]
    carried=transport_points(original['path_points'],h,old['data']['input']['reference_s'],scene.t_ref_s)
    points=np.asarray(scene.occupied_endpoints_xy_m,dtype=float)
    # 실제 경계 지지 선택 순서와 같이 원시/기억 지지/돌출 제외를 따로 기록한다.
    supported=boundary_support(points,carried,d['side'],cfg)
    trimmed=drop_protrusions(supported,carried,d['side'],cfg)
    removed=len(supported)-len(trimmed)
    cases={}
    for label,p in [('unfiltered',points),('history_support',supported),('protrusions_removed',trimmed)]:
        path,info=local_path(p,d['side'],cfg.wall_offset_m)
        cases[label]=dict(points=len(p),info=freeze(info),
            enough=bool(path is not None and info['boundary_observed_ahead_m']>=cfg.boundary_min_ahead_m and
                        abs(info['wall_distance_m']-cfg.wall_offset_m)<=cfg.boundary_anchor_error_m),
            agrees=bool(path is not None and path_agrees(path,carried,cfg.boundary_agreement_m)),
            difference=None if path is None else geometric_difference(path,carried))
    names={}
    for module in tuple(sys.modules.values()):
        filename=getattr(module,'__file__',None)
        if filename:
            p=Path(filename).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':names[p.relative_to(ROOT).as_posix()]=digest(p)
    with zipfile.ZipFile(OUT/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in names:archive.write(ROOT/name,name)
        archive.write(__file__,'probe.py')
    report=dict(scope='boundary_refresh_same_input_not_runtime_change',old_seq=3156,new_seq=3161,
        previous_path_source_s=original['path_source_s'],current_ref=scene.t_ref_s,
        previous_age=scene.t_ref_s-original['path_source_s'],ttl=cfg.memory_ttl_s,
        carried_path=carried.tolist(),cases=cases,protrusions_removed=removed,
        input_sha256=before,input_unchanged=before==digest(CAPTURE),source_sha256=names,
        source_unchanged=all(digest(ROOT/n)==v for n,v in names.items()),runtime_changed=False)
    (OUT/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('cases','carried_path','source_sha256')}))
    print(json.dumps({k:{a:b for a,b in v.items() if a!='info'} for k,v in cases.items()}))
    return 0 if report['input_unchanged'] and report['source_unchanged'] else 1

if __name__=='__main__':raise SystemExit(main())
