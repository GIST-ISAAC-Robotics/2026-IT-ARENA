"""선호 벽과 코너 안쪽/바깥쪽 역할의 혼동 여부만 확인. 경로 채택/주행 없음."""
import json
from pathlib import Path
import sys
import zipfile

from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,path_agrees
from arena_autonomy.local_path import local_path
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corner_corridor import corridor_evidence
from audit_corridor_runtime import digest

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
CAPTURE=ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'


def main():
    assert not (OUT/'report.json').exists()
    before=digest(CAPTURE);rows=[]
    target={3151,3156,3166,3171,3176,3201,3301,3401,3501}
    for line in CAPTURE.open():
        event=json.loads(line)
        if event['kind']=='header':
            cfg=FollowConfig(**event['data']['follow_config']);motion=MotionConfig(**event['data']['motion_config'])
        if event['kind']!='call' or event['seq'] not in target:continue
        d,raw,h=restore_call(event['data']['input'],motion)
        scene=motion_scene(raw,h,d['reference_s'],timing_verified=d['timing_verified'],
            epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        points=scene.occupied_endpoints_xy_m
        preferred,preferred_info=local_path(points,d['side'],cfg.wall_offset_m)
        candidates={}
        for inner in ('left','right'):
            geometry=corridor_evidence(points,inner,cfg)
            candidates[inner]={k:freeze(v) for k,v in geometry.items()}
            candidates[inner]['agrees_preferred_observed_path']=bool(geometry['supported'] and preferred is not None and
                path_agrees(geometry['path'],preferred,cfg.boundary_agreement_m))
        rows.append(dict(seq=event['seq'],time_s=d['reference_s'],preferred_side=d['side'],
            reason=event['data']['output'][2]['reason'],preferred_info=freeze(preferred_info),candidates=candidates))
    names={}
    for module in tuple(sys.modules.values()):
        filename=getattr(module,'__file__',None)
        if filename:
            p=Path(filename).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':names[p.relative_to(ROOT).as_posix()]=digest(p)
    with zipfile.ZipFile(OUT/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in names:archive.write(ROOT/name,name)
        archive.write(__file__,'probe.py')
    report=dict(scope='role_assignment_geometry_only',runtime_changed=False,driving_authorized=False,
        input_sha256=before,input_unchanged=before==digest(CAPTURE),source_sha256=names,
        source_unchanged=all(digest(ROOT/n)==v for n,v in names.items()),rows=rows)
    (OUT/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    for row in rows:
        print(json.dumps(dict(seq=row['seq'],preferred=row['preferred_side'],
            roles={k:{n:v.get(n) for n in ('supported','reason','outer_side','common_prefix_m','agrees_preferred_observed_path')}
                   for k,v in row['candidates'].items()})))
    return 0 if report['input_unchanged'] and report['source_unchanged'] and len(rows)==len(target) else 1

if __name__=='__main__':raise SystemExit(main())
