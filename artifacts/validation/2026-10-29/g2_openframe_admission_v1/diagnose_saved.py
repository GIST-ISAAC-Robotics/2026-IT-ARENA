"""저장한 Gazebo 스캔의 진입 관측만 비교한다. 새 주행/정지차 증명이 아니다."""
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import numpy as np
BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.bypass_handoff import BlockingPersistence,HandoffExecutionConfig,front_patch

def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def groups(scene):
    result=[];current=[];previous=None
    for ray in scene.rays:
        p=ray.hit_xy_m
        ok=p is not None and .20<=p[0]<=1.60 and abs(p[1])<=.24
        if not ok or (previous is not None and ray.source_index!=previous+1):
            if current:result.append(current);current=[]
        if ok:current.append(list(p))
        previous=ray.source_index
    if current:result.append(current)
    return [dict(n=len(p),median_xy=np.median(p,axis=0).tolist()) for p in result]

def main():
    out=BASE/'saved1';out.mkdir(exist_ok=False)
    paths=[p for p in (ROOT/'src').rglob('*.py') if '__pycache__' not in p.parts]+[Path(__file__)]
    hashes={p.relative_to(ROOT).as_posix():digest(p) for p in paths}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in paths:z.write(p,p.relative_to(ROOT))
    reports=[]
    for name in ('g2_entry_gazebo_v1','g2_openframe_v1'):
        path=BASE.parent/name/'run1/g2_decisions.jsonl';rows=[];first=None;lastkey=None;footer=None
        cfg=HandoffExecutionConfig(preparation_policy='observed_entry_space_v1',persistence_window_policy='trim_before_compare_v1')
        monitor=BlockingPersistence(cfg)
        with path.open(encoding='utf-8') as stream:
            for line in stream:
                e=json.loads(line)
                if e['kind']=='header':motion=MotionConfig(**e['data']['motion_config'])
                elif e['kind']=='reset':monitor.reset()
                elif e['kind']=='footer':footer=e['data']
                elif e['kind']=='call':
                    d,scan,history=restore_call(e['data']['input'],motion)
                    scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
                    result=monitor.update(scene,history)
                    key=(scene.source_epoch,scene.source_first_s)
                    if key==lastkey:continue
                    lastkey=key
                    if first is None:first=d['reference_s']
                    patch,reason=front_patch(scene)
                    rows.append(dict(reference_s=d['reference_s'],elapsed_s=d['reference_s']-first,source_s=scene.source_first_s,
                        observed_speed_mps=d['speed_mps'],previous_steering_rad=d['previous_steering_rad'],
                        groups=groups(scene),patch_count=0 if patch is None else len(patch),reason=reason,persistence=result))
        if footer is None or any(footer.get(k)!=0 for k in ('failed','dropped','pending','capture_errors')):raise ValueError('bad preserved trace')
        (out/(name+'.json')).write_text(json.dumps(freeze(rows),indent=2,allow_nan=False)+'\n',encoding='utf-8')
        initial=[r for r in rows if r['elapsed_s']<.8]
        reports.append(dict(source=name,input_sha256=digest(path),distinct_scans=len(rows),first_time_s=first,
            initial_0_8s=dict(scans=len(initial),reason_counts=dict(Counter(r['reason'] for r in initial)),
                max_group=max((g['n'] for r in initial for g in r['groups']),default=0),first_groups=initial[0]['groups']),
            full_reason_counts=dict(Counter(r['reason'] for r in rows)),first_patch_s=next((r['elapsed_s'] for r in rows if r['patch_count']),None),
            first_ready_s=next((r['elapsed_s'] for r in rows if r['persistence']['ready']),None)))
    report=dict(scope='Stored manual G2 trajectory: observation diagnostics only. No automatic admission closed loop or permanent deadlock proof.',
        execution_config=asdict(cfg),cases=reports,source_sha256=hashes,source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()))
    (out/'report.json').write_text(json.dumps(freeze(report),indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(dict(cases=reports,source_unchanged=report['source_unchanged']),indent=2))
if __name__=='__main__':main()
