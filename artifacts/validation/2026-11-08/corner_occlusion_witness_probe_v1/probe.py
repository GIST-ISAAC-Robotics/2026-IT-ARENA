"""첫 코너 가림 실패의 고정 입력 대조. 실제 주행/운영 연결은 하지 않는다."""
from dataclasses import replace
import json
from pathlib import Path
import numpy as np
from arena_autonomy.follow_trace import FollowConfig,MotionConfig,restore_call,freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.corner_boundary_support import propose as original
import candidate

A=Path(__file__).resolve().parent
captures=json.loads((A.parent/'corner_r41_cause_audit_v1/captures.json').read_bytes())
trace=A.parent/'current_corner_support_v1/run1/follow_decisions.jsonl'
rows=[];seen_proofs=[];old=candidate.far_face_occlusion
def audit(*args):
    r=old(*args);seen_proofs.append(r);return r
candidate.far_face_occlusion=audit
for line in trace.open():
    e=json.loads(line);d=e['data']
    if e['kind']=='header':cfg=FollowConfig(**d['follow_config']);mc=MotionConfig(**d['motion_config'])
    if e['kind']!='call':continue
    t=d['input']['reference_s']
    if not any(abs(t-w)<1e-8 for w in (35.72,35.82,35.92)):continue
    rec=min(captures,key=lambda q:abs(q['t']-t));assert abs(rec['t']-t)<1e-8
    args,scan,hist=restore_call(d['input'],mc)
    scene=motion_scene(scan,hist,t,timing_verified=args['timing_verified'],epoch=args['epoch'],rear_half_angle_deg=args['rear_half_angle_deg'])
    points=np.asarray(scene.occupied_endpoints_xy_m);ref=np.asarray(rec['carried_path'])
    source=d['output'][2]['normal']['path_source_s']
    age=.218  # 이 세 fresh 호출의 직전 현재 경로 나이는 원 기록에서 모두 .218초였다.
    before=original(points,ref,'right',cfg,age)
    seen_proofs.clear();after=candidate.propose(points,ref,'right',cfg,age,scene=scene)
    row=dict(t=t,before=before[1],after=after[1],far_attempts=list(seen_proofs))
    if after[0] is not None and before[0] is not None:
        row['same_path']=bool(np.array_equal(after[0],before[0]))
    if after[0] is not None and before[0] is None:
        ids=set(after[1]['far_occlusion_support']['foreground_source_indices'])
        hidden=replace(scene,rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.source_index in ids else r for r in scene.rays))
        # 원래 점 목록을 유지해도 광선 증거가 없으면 추가 경로는 불허해야 한다.
        removed=candidate.propose(points,ref,'right',cfg,age,scene=hidden)
        expired=candidate.propose(points,ref,'right',cfg,cfg.memory_ttl_s,scene=scene)
        no_scene=candidate.propose(points,ref,'right',cfg,age)
        row['counterexamples']=dict(no_foreground=removed[0] is None,expired=expired[0] is None,no_rays=no_scene[0] is None)
    rows.append(row)
(A/'probe.json').write_text(json.dumps(freeze(rows),indent=2))
print(json.dumps([{k:v for k,v in r.items() if k not in ('before','after')}|dict(before=r['before']['accepted'],after=r['after']['accepted']) for r in rows]))
