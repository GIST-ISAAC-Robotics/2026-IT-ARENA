"""고정된 기존 경로에서 짧은 후방 면 해석만 바꾼 민감도. 채택 정책/전체 제어기 아님."""
from collections import Counter
from pathlib import Path
import hashlib,json,math
import numpy as np
from arena_autonomy.follow_trace import restore_call,motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.bypass_entry_trial import _compare_entries
from arena_autonomy.moving_pass import MovingEntryConfig
from arena_autonomy.moving_pass_wall_padding import current_outer_and_rear_padding
from arena_autonomy.known_structure import classify,parse_sectors
from arena_autonomy.wall_reference_support import observed_rear_wall_face

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
TRACE=ROOT/'artifacts/validation/2026-11-08/moving_pass_wall_mask_v1/straight_moving_lead/run1/follow_decisions.jsonl'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before=sha(TRACE);rows=[];seen=set();mismatches=[]
for line in TRACE.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':cfg=FollowConfig(**d['follow_config']);mc=MotionConfig(**d['motion_config'])
    if e['kind']!='call' or d['output'][0]>0:continue
    result=d['output'][2].get('moving_pass',{}).get('entry',{})
    if not result.get('candidates'):continue
    row=result['candidates'][0];side=row.get('current_outer_wall')
    if side not in ('left','right') or not row.get('path_points'):continue
    original,scan,history=restore_call(d['input'],mc)
    scene=motion_scene(scan,history,original['reference_s'],timing_verified=original['timing_verified'],epoch=original['epoch'],rear_half_angle_deg=original['rear_half_angle_deg'])
    key=(scene.source_first_s,side)
    if key in seen:continue
    seen.add(key);wall=result['reference']['walls'][side]
    sectors=parse_sectors(cfg.known_structure_sectors_json)
    _,_,classified=classify(scan,sectors)
    explained={frozenset(r['rays'].tolist()) for r in classified if r['explained']}
    indices=np.asarray([r.source_index for r in scene.rays if r.status=='return'])
    baseline_padding,baseline_proof=current_outer_and_rear_padding(scene,cfg,side,wall,raw_scan=scan,sectors=sectors)
    fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
    # .20 m는 기존 경로 지지 조건이다. 정체성이 불명확한 짧은 면을 벽으로 가정한 민감도만 측정한다.
    face=observed_rear_wall_face(scene,cfg,side,fit,wall,explained,minimum_rear_span_m=.20,rear_limit_m=-1.5)
    mask=np.isin(indices,face['source_ray_indices']) if face else np.zeros(len(indices),bool)
    short_padding=baseline_padding.copy();short_padding[mask]=.025+cfg.measurement_padding_m
    wall_enabled=result['stationary_outer_boundary_requested'] and result['outer_wall_fresh_confirmations']>=2
    assert wall_enabled==('outer_boundary_padding' in result)
    guide=dict(reference=result['reference'],outer_side=side,path=row['path_points'],
        offset_m=row['offset_m'],transition_m=row['transition_m'],stationary_outer_boundary=wall_enabled)
    variants={}
    for name,pad in [('original',baseline_padding),('short_face_static_diagnostic',short_padding)]:
        provider=lambda *args,**kwargs:(pad.copy(),dict(baseline_proof,diagnostic_only=name!='original'))
        out=_compare_entries(scene,scene.t_ref_s,original['speed_mps'],original['previous_steering_rad'],original['dt_s'],
            config=cfg,entry=MovingEntryConfig(speed_cap_mps=1.2,profile='faster_v1'),guide=guide,padding_provider=provider)
        x=out['candidates'][0]
        variants[name]={k:x.get(k) for k in ['eligible','reason','target_speed_mps','steering_rad','command_clearance_m','motion_region']}
    base=variants['original']
    if base['reason']!=row['reason'] or not np.allclose([base['target_speed_mps'],base['steering_rad'],base['command_clearance_m']],
            [row['target_speed_mps'],row['steering_rad'],row['command_clearance_m']],atol=1e-9,rtol=0):
        mismatches.append(dict(t=scene.t_ref_s,recorded={k:row[k] for k in ['reason','target_speed_mps','steering_rad','command_clearance_m']},reproduced=base))
    rows.append(dict(t=scene.t_ref_s,source=scene.source_first_s,short_face=face is not None,wall_enabled=wall_enabled,
        changed_points=int(np.count_nonzero(short_padding<baseline_padding-1e-12)) if wall_enabled else 0,variants=variants))
summary=dict(scope='sensitivity on fixed recorded guide only; NOT a selected wall classification or new trajectory',
    scenes=len(rows),baseline_mismatches=len(mismatches),input_unchanged=sha(TRACE)==before,input_sha256=before,
    current_short_face_scenes=sum(r['short_face'] for r in rows),
    variants={name:dict(eligible=sum(r['variants'][name]['eligible'] for r in rows),
        eligible_positive=sum(r['variants'][name]['eligible'] and r['variants'][name]['target_speed_mps']>0 for r in rows),
        reasons=Counter(r['variants'][name]['reason'] for r in rows)) for name in ('original','short_face_static_diagnostic')})
for name,data in [('sensitivity_v2_summary.json',summary),('sensitivity_v2_rows.json',rows),('sensitivity_v2_mismatches.json',mismatches)]:
    (A/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps(summary,ensure_ascii=False))
assert not mismatches and summary['input_unchanged']
