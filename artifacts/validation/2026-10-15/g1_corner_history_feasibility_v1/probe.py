"""낙관적 동일 정적 성분 가정의 0.3초 기억 지름. 주행/벽 연결 증명 아님."""
from collections import Counter
from dataclasses import dataclass
import json
from pathlib import Path
import zipfile

import numpy as np
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene, transport_points
from inspect_corner_geometry import components, digest
from profile_follow_trace import SNAPSHOT_MODULES

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]


def diameter(points):
    if len(points)<2:return 0.
    p=np.asarray(points)
    return float(np.max(np.linalg.norm(p[:,None]-p[None],axis=2)))


@dataclass
class Witness:
    seq: int
    source_first: float
    reference: float
    epoch: int
    side: str
    points: np.ndarray


def retained(witnesses, now, epoch, side, ttl):
    return [w for w in witnesses if w.epoch==epoch and w.side==side and 0<=now-w.source_first<ttl]


def self_check():
    p=np.array([[0.,0.],[.6,.2]])
    a=Witness(1,.1,.2,0,'left',p)
    assert retained([a],.39,0,'left',.3)==[a]
    assert retained([a],.4,0,'left',.3)==[]
    assert retained([a],.2,1,'left',.3)==[]
    assert retained([a],.2,0,'right',.3)==[]
    assert retained([a],.09,0,'left',.3)==[]
    assert diameter([])==0 and diameter([[1.,2.]])==0
    assert abs(diameter(p)-np.sqrt(.4))<1e-12


def main():
    self_check()
    target=BASE/'report.json'
    if target.exists():raise FileExistsError(target)
    raw=BASE.parent/'g1_solo_lap_v4/run/follow_decisions.jsonl'
    before=digest(raw);witnesses=[];rows=[];last_source=None;last_time=None
    with raw.open() as stream:
        for line in stream:
            e=json.loads(line)
            if e['kind']=='header':
                cfg=FollowConfig(**e['data']['follow_config'])
                motion=MotionConfig(**e['data']['motion_config'])
            elif e['kind']=='reset':witnesses=[];last_source=last_time=None
            elif e['kind']=='call' and 41.8<=e['data']['input']['reference_s']<=44.8:
                data,scan,h=restore_call(e['data']['input'],motion)
                now=data['reference_s'];side=data['side'];epoch=data['epoch']
                scene=motion_scene(scan,h,now,timing_verified=data['timing_verified'],epoch=epoch,
                                   rear_half_angle_deg=data['rear_half_angle_deg'])
                if last_source==scene.source_first_s:continue
                if ((last_source is not None and scene.source_first_s<last_source) or
                    (last_time is not None and now<last_time)):
                    witnesses=[]
                last_source,last_time=scene.source_first_s,now
                witnesses=retained(witnesses,now,epoch,side,cfg.memory_ttl_s)
                p=np.asarray(scene.occupied_endpoints_xy_m);parts=components(p,side)
                sign=1 if side=='left' else -1
                part=min(parts,key=lambda c:float(np.min(np.linalg.norm(p[c['indices']]-[0,sign*cfg.wall_offset_m],axis=1)))) if parts else None
                current=p[part['indices']] if part is not None else np.empty((0,2))
                if len(current):
                    witnesses.append(Witness(e['seq'],scene.source_first_s,now,epoch,side,current))
                projected=[];sources=[];error=None
                for w in witnesses:
                    try:
                        shifted=transport_points(w.points,h,w.reference,now)
                    except MotionUnavailable as exc:
                        error=str(exc);break
                    projected.append(shifted)
                    sources.append(dict(seq=w.seq,source_first_s=w.source_first,age_s=now-w.source_first,
                                        reference_s=w.reference,point_count=len(w.points)))
                combined=np.vstack(projected) if projected and error is None else np.empty((0,2))
                current_extent=diameter(current);pooled_extent=diameter(combined)
                rows.append(dict(seq=e['seq'],time_s=now,current_extent_m=current_extent,
                    pooled_extent_m=pooled_extent,source_witnesses=sources,transport_error=error,
                    pooled_reaches_length=error is None and pooled_extent>=cfg.boundary_min_ahead_m,
                    current_reaches_length=current_extent>=cfg.boundary_min_ahead_m,
                    recorded_speed_mps=e['data']['output'][0],recorded_reason=e['data']['output'][2]['reason'],
                    same_static_component_assumed=True,identity_certified=False,connected_wall_certified=False,
                    driving_authorized=False))
    paths=[f'src/{pkg}/{pkg}/{mod}.py' for pkg,mods in SNAPSHOT_MODULES.items() for mod in mods]
    paths+=['scripts/inspect_corner_geometry.py','scripts/profile_follow_trace.py',str(Path(__file__).relative_to(ROOT))]
    hashes={p:digest(ROOT/p) for p in paths}
    with zipfile.ZipFile(BASE/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in paths:z.write(ROOT/p,p)
    result=dict(scope='optimistic_saved_input_geometry_not_tracking_or_closed_loop',rows=rows,
        input_sha256=before,input_unchanged=before==digest(raw),source_sha256=hashes,
        source_unchanged=all(digest(ROOT/p)==v for p,v in hashes.items()),driving_authorized=False,
        source_age_limit_s=cfg.memory_ttl_s,self_check_passed=True)
    target.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(calls=len(rows),input_unchanged=result['input_unchanged'],source_unchanged=result['source_unchanged'],
        rows=[{k:v for k,v in r.items() if k in ('seq','time_s','current_extent_m','pooled_extent_m','pooled_reaches_length','recorded_reason','transport_error')} for r in rows])))


if __name__=='__main__':main()
