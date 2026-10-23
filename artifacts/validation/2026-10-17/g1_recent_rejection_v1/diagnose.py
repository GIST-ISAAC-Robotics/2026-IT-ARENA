"""첫 실제 관측 거절의 깊이/횡방향/후방 원인 분리. 명령 수정이나 새 주행 아님."""
from collections import Counter
import json
from pathlib import Path
import sys
import zipfile
import numpy as np

from arena_autonomy import observation_support as obs
from arena_autonomy.follow_trace import restore_call,equal_output,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.observation_lateral import protected_queries,reachable_from_unknown
from audit_observation_history import lateral_support
from audit_corridor_runtime import digest

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent
CAPTURE=ROOT/'artifacts/validation/2026-10-17/g1_recent_solo_v1/run/follow_decisions.jsonl'

def main():
    assert not (OUT/'report.json').exists()
    before=digest(CAPTURE)
    original=obs.sweep_support;calls=[]
    def captured(*args,**kwargs):
        result=original(*args,**kwargs)
        calls.append((args,kwargs,result))
        return result
    obs.sweep_support=captured
    rejected=None;later=[];checked=0;mismatches=[]
    for line in CAPTURE.open():
        event=json.loads(line);kind=event['kind']
        if kind=='header':
            cfg=FollowConfig(**event['data']['follow_config']);motion=MotionConfig(**event['data']['motion_config'])
            controller=LocalFollow(cfg)
        elif kind=='reset':controller.reset()
        elif kind=='call':
            d,scan,h=restore_call(event['data']['input'],motion)
            scene=motion_scene(scan,h,d['reference_s'],timing_verified=d['timing_verified'],
                epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
            calls.clear()
            output=controller.command(scene,h,d['speed_mps'],d['side'],d['previous_steering_rad'],d['dt_s'])
            checked+=1
            if not equal_output(freeze(output),event['data']['output']):mismatches.append(event['seq'])
            if output[2]['reason']=='follow_observation_unconfirmed' and rejected is None:
                assert len(calls)==1
                args,kwargs,support=calls[0]
                sources,speed,target,steer,previous,config=args
                queries=np.unique(np.vstack([obs.sweep_queries(scene,a,support['required_distance_m'],cfg)[0]
                    for a in set((steer,previous))]),axis=0)
                protected=protected_queries(queries,kwargs['protected_sensor_pose'])
                depth_union=np.zeros(len(queries),bool);accepted_union=depth_union.copy();source_rows=[]
                for s in sources:
                    padding=np.array([.025+cfg.measurement_padding_m+cfg.object_motion_bound_mps*(s.t_ref_s-r.acquired_at_s) for r in s.rays])
                    depth=obs.depth_support(s,queries,padding)
                    rejected_lateral=reachable_from_unknown(s,queries[protected],cfg)
                    reachable=np.zeros(len(queries),bool);reachable[protected]=rejected_lateral
                    accepted=depth&~reachable;depth_union|=depth;accepted_union|=accepted
                    source_rows.append(dict(first_s=s.source_first_s,depth=int(depth.sum()),accepted=int(accepted.sum()),
                        unknown=[dict(index=r.source_index,status=r.status,origin=r.ray_origin_xy_m,direction=r.direction_xy,
                                      acquired_at_s=r.acquired_at_s) for r in s.rays if r.status=='unknown']))
                missing=~accepted_union
                indices=np.flatnonzero(missing)
                witness=[]
                for s in sources:
                    _,_,examples=lateral_support(s,queries[missing&protected][:30],cfg)
                    witness.append(dict(first_s=s.source_first_s,examples=examples))
                rejected=dict(seq=event['seq'],t_ref_s=d['reference_s'],speed_mps=speed,target_mps=target,
                    proposed_steer=steer,previous_steer=previous,support=support,sources=source_rows,
                    no_depth=int((~depth_union).sum()),lateral_only=int((depth_union&missing).sum()),
                    protected_missing=int((missing&protected).sum()),rear_missing=int((missing&~protected).sum()),
                    missing_queries=queries[indices].tolist(),independent_witness=witness)
                assert int(accepted_union.sum())==support['supported_queries']
            elif rejected is not None and len(later)<12:
                later.append(dict(seq=event['seq'],t_ref_s=d['reference_s'],output=freeze(output)))
            if len(later)==12:break
    obs.sweep_support=original
    names={}
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':names[p.relative_to(ROOT).as_posix()]=digest(p)
    with zipfile.ZipFile(OUT/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as archive:
        for name in names:archive.write(ROOT/name,name)
        archive.write(__file__,'diagnose.py')
    report=dict(scope='first_rejected_record_input_not_new_driving',replayed=checked,mismatches=mismatches,
        rejected=rejected,later=later,source_sha256=names,input_sha256=before,input_unchanged=before==digest(CAPTURE),
        source_unchanged=all(digest(ROOT/n)==h for n,h in names.items()))
    (OUT/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('rejected','later','source_sha256')}))
    print(json.dumps({k:v for k,v in rejected.items() if k not in ('missing_queries','independent_witness','sources')}))
    return 0 if not mismatches and report['input_unchanged'] and report['source_unchanged'] else 1

if __name__=='__main__':raise SystemExit(main())
