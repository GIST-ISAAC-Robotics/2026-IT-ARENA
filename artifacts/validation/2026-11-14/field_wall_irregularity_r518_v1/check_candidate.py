"""후보만 process-local 로드. 공통 소스/입력은 불변이다."""
from pathlib import Path
import copy, importlib.util, json, sys
from dataclasses import replace
import numpy as np
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(A/'source_snapshot/arena_autonomy'),str(A/'source_snapshot/arena_vehicle_interface'),str(ROOT/'scripts'),str(A/'test_deps')]
old_cross=np.cross
def compat(a,b,*args,**kwargs):
    aa,bb=np.asarray(a),np.asarray(b)
    if not args and not kwargs and aa.shape[-1]==bb.shape[-1]==2:return aa[...,0]*bb[...,1]-aa[...,1]*bb[...,0]
    return old_cross(a,b,*args,**kwargs)
np.cross=compat
import arena_autonomy.bypass_wall_fit as mod
baseline=mod.tracked_straight_wall
spec=importlib.util.spec_from_file_location('candidate_fit',A/'bypass_wall_fit.candidate.py')
candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)
from arena_autonomy.follow_trace import restore_call, freeze, comparable_decision, equal_output
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.scene_motion import motion_scene
source=A.parent/'continuous_pass_preview_drive_r513_v1/run1/follow_decisions.jsonl'
for line in source.open():
    r=json.loads(line)
    if r['kind']=='runtime_evaluator_header':header=r['data']['header'];break
cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
data=json.loads((A/'current/input_seq90_bend20_gap10.json').read_text())
d,scan,history=restore_call(data,motion)
scene=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
report=json.loads((A/'current_report.json').read_text())
identity=next(x for x in report['probes'] if x['key'][1]==90 and x['variant']=='identity')
a,b,lo,hi=identity['wall_captures'][0]['fit'];xx=np.array([lo,hi]);line=np.c_[xx,a*xx+b]
probe=next(x for x in report['probes'] if x['key'][1]==90 and x['variant']=='bend20_gap10')
ids=set(probe['wall_ray_indices']);missing=set(probe['removed_wall_ray_indices'])
results=[]
def check(label,s,joined,full_fit=None):
    before=copy.deepcopy(s);fit,detail=candidate.tracked_straight_wall(s,cfg,'left',line)
    count=detail.get('small_wall_joints_joined',0)
    assert bool(count)==joined,(label,fit,detail)
    assert s==before,(label,'scene mutated')
    if full_fit is not None:assert bool(fit is not None and fit[3]>1.5)==full_fit,(label,fit)
    results.append(dict(case=label,fit=fit,joins=count,reason=detail['reason'],scene_unchanged=True))
check('10mm_joint_between_bent_wall_pieces',scene,True,True)
base_fit,base_detail=baseline(scene,cfg,'left',line)
assert base_fit[3]<.5
# 실제 반환이 끼면 새 연결부 helper는 잇지 않는다. 기존 좁은 가림체 경로는 별도이며
# 실제 반환을 점유에서 제거하지 않는다. 그 경우 baseline 결과와 같아야 한다.
rays=tuple(replace(r,status='return',hit_xy_m=(r.ray_origin_xy_m[0]+.35*r.direction_xy[0],r.ray_origin_xy_m[1]+.35*r.direction_xy[1])) if r.source_index in missing else r for r in scene.rays)
obstacle=replace(scene,rays=rays)
check('intervening_actual_obstacle_return',obstacle,False)
assert baseline(obstacle,cfg,'left',line)==candidate.tracked_straight_wall(obstacle,cfg,'left',line)
rays=tuple(replace(r,status='return',hit_xy_m=(r.ray_origin_xy_m[0]+3.*r.direction_xy[0],r.ray_origin_xy_m[1]+3.*r.direction_xy[1])) if r.source_index in missing else r for r in scene.rays)
check('background_return_through_joint',replace(scene,rays=rays),False,False)
rays=tuple(replace(r,status='masked') if r.source_index in missing else r for r in scene.rays)
check('masked_ray_is_not_wall_joint',replace(scene,rays=rays),False,False)
rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.source_index in ids and r.status=='return' and r.hit_xy_m[0]>.45 else r for r in scene.rays)
check('true_wall_end',replace(scene,rays=rays),False,False)
rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.source_index in ids and r.status=='return' and .20<r.hit_xy_m[0]<.70 else r for r in scene.rays)
check('large_missing_wall_section',replace(scene,rays=rays),False,False)
rays=tuple(replace(r,status='unknown',hit_xy_m=None) if r.source_index%3==0 else r for r in scene.rays)
check('frequent_missing_rays_are_not_long_pieces',replace(scene,rays=rays),False,False)
# 기본 입력 전체는 상세 판단까지 동일해야 한다.
current=json.loads((A/'current_baseline_outputs.json').read_text());proposed=json.loads((A/'candidate_baseline_outputs.json').read_text())
full=[x['key']==y['key'] and equal_output(comparable_decision(x['output']),comparable_decision(y['output'])) for x,y in zip(current,proposed)]
assert all(full) and len(full)==124
(A/'counterexamples.json').write_text(json.dumps(freeze(dict(cases=results,unperturbed_full_matches=sum(full),total=len(full))),indent=2),encoding='utf-8')
print(json.dumps(dict(counterexamples=len(results),unperturbed_full_matches=sum(full))),flush=True)
if '--counterexamples-only' in sys.argv:raise SystemExit(0)
# 같은 후보를 기존 관련 회귀에 적용한다.
mod.tracked_straight_wall=candidate.tracked_straight_wall
import pytest
result=pytest.main(['-q',str(ROOT/'tests/test_bypass_wall_fit.py'),str(ROOT/'tests/test_moving_pass_wall_padding.py'),str(ROOT/'tests/test_moving_pass.py')])
(A/'pytest_exit.json').write_text(json.dumps(dict(exit_code=int(result))),encoding='utf-8')
raise SystemExit(result)
