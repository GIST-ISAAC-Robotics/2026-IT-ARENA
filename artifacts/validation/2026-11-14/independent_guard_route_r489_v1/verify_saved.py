"""원 default exact 및 r485 저장입력 대표. full guard 내부상태 재현이 아니다."""
import run_windows
from run_windows import A, ROOT
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import numpy as np

sys.path.insert(0,str(A/'stage/tests'))
from test_independent_guard_route import policy,scan,history,evaluate
from arena_autonomy import partial_observation as candidate
from arena_autonomy import known_structure as new_structure
from arena_autonomy.follow_trace import freeze,restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_path import mask_scan,swept_limit,braking_speed
from arena_autonomy.scan_input import project_scan,scan_first_stamp
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy


def module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    obj=importlib.util.module_from_spec(spec);sys.modules[name]=obj;spec.loader.exec_module(obj)
    return obj


def encoded(value):
    def normalize(v):
        if isinstance(v,(set,frozenset)):return dict(set_values=sorted(v))
        if isinstance(v,dict):return {k:normalize(x) for k,x in v.items()}
        if isinstance(v,(list,tuple)):return [normalize(x) for x in v]
        return v
    return json.dumps(freeze(normalize(value)),sort_keys=True,separators=(',',':'))


def decision_encoded(value,state=False):
    result=copy.deepcopy(value)
    fit=(result.get('surface_cache') or (None,None,None))[2] if state else result.get('surface_correspondence')
    if isinstance(fit,dict) and 'processing_ms' in fit:
        assert math.isfinite(fit['processing_ms']) and fit['processing_ms']>=0
        fit['processing_ms']=0. # 기존 follow_trace 계약의 비결정적 진단 하나만 제외.
    return encoded(result)


old=module('r489_original_partial',A/'base/src/arena_autonomy/arena_autonomy/partial_observation.py')
old_structure=module('r489_original_structure',A/'base/src/arena_autonomy/arena_autonomy/known_structure.py')
exact=[]
for name,seeds in (
    ('fully_observed',[(.3,[],[],.8),(.5,[],[],.8)]),
    ('warmup_then_partial',[(.3,[375],[],.4),(.5,[],[],.4),(.7,[375],[],.4)]),
    ('real_missing_latch',[(.3,[],[[.06,.15]],.4),(.5,[375],[],.4),(.7,[],[],.4),(.9,[],[],.4)]),
    ('reobserved',[(.3,[],[[.06,.15]],.4),(.5,[375],[[.06,.15]],.4)]),
    ('speed_outside',[(.3,[],[],.4),(.5,[375],[],1.5)])):
    baseline=policy(old.PartialObservationPolicy,False)
    current=policy(candidate.PartialObservationPolicy,False)
    for first,missing,points,speed in seeds:
        raw=scan(first)
        for index in missing:raw.ranges[index]=float('nan')
        a=evaluate(baseline,raw,points=points,speed=speed)
        b=evaluate(current,raw,points=points,speed=speed)
        row=dict(case=name,source=first,output_exact=decision_encoded(a)==decision_encoded(b),
                 state_exact=decision_encoded(vars(baseline),True)==decision_encoded(vars(current),True),
                 raw_output_exact=encoded(a)==encoded(b),raw_state_exact=encoded(vars(baseline))==encoded(vars(current)),
                 baseline_output=freeze(a),candidate_output=freeze(b),reason=b['reason'])
        assert row['output_exact'] and row['state_exact'],row
        exact.append(row)

sectors='[[0.0,3.0]]'
before=policy(old_structure.structure_policy(old.PartialObservationPolicy,sectors),False)
after=policy(new_structure.structure_policy(candidate.PartialObservationPolicy,sectors),False)
for p in (before,after):p.structure_previous=(np.array([[.2,0.]]),.5)
for first in (.5,.7):
    raw=mask_scan(scan(first),dict(rear_half_angle_deg=0.,sectors_deg=[[0.,3.]]))
    a=evaluate(before,raw,points=[],speed=1.)
    b=evaluate(after,raw,points=[],speed=1.)
    row=dict(case='default_structure_hidden',source=first,output_exact=encoded(a)==encoded(b),
             state_exact=encoded(vars(before))==encoded(vars(after)),reason=b['reason'])
    assert row['output_exact'] and row['state_exact'],row
    exact.append(row)
(A/'default_exact.json').write_text(json.dumps(dict(passed=True,
    exact_comparison_excludes_only='surface_correspondence.processing_ms / state.surface_cache[2].processing_ms',
    numeric_tolerance=0,rows=exact),indent=2),encoding='utf-8')

source=ROOT/'artifacts/validation/2026-11-14/solo_variation_recovery_r485_v1/failure_inputs'
paths=[source/n for n in ('runtime_headers.json','follow_records.jsonl','autonomy_commands.json','README.json')]
hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
header=json.loads(paths[0].read_text())['runtime_evaluator_header']['data']['header']
config=MotionConfig(**header['motion_config']);cfg=header['follow_config']
records=[json.loads(line)['record'] for line in paths[1].read_text().splitlines()]
inputs=[r for r in records if r['kind']=='runtime_plan_submitted']
commands=[r['data'] for r in json.loads(paths[2].read_text())]
cls=new_structure.structure_policy(RecoveryPolicy,cfg['known_structure_sectors_json'])
rows=[]
for seq in (2893,2905,2911,2916):
    r=next(r for r in inputs if r['seq']==seq)
    raw=r['data']['input'];data,scan_input,h=restore_call(raw,config)
    now=data['reference_s']
    command=max((c for c in commands if c['sim_time_s']<=now),key=lambda c:c['sim_time_s'])
    speed=command['speed_command_mps'];steering=command['steering_command_rad']
    # 실제 guard 이전 safe 조향은 저장되지 않았다. 입력 planner previous angle을
    # 대체 관측값으로 명시하며, cold 객체라 이전 숨은 기억의 복원은 주장하지 않는다.
    previous=data['previous_steering_rad']
    masked=mask_scan(scan_input,data['rear_half_angle_deg'])
    points,ids,_=project_scan(masked,h,now,timing_verified=data['timing_verified'])
    measured=h.wheels.available(now)[1][-1]
    outputs={}
    for selected in (False,True):
        p=policy(cls,selected)
        result=p.evaluate(masked,points,h,now,max(speed,abs(measured)),steering,previous,
            cfg['wheelbase_m'],cfg['length_m'],cfg['width_m'])
        clearance=swept_limit(points,steering,cfg['wheelbase_m'],length=cfg['length_m'],width=cfg['width_m'])
        cap=braking_speed(clearance,now-scan_first_stamp(masked))
        value=min(speed,cap,result['speed_cap_mps'] if result['speed_cap_mps'] is not None else speed) if result['allowed'] else 0.
        outputs['selected' if selected else 'legacy']=dict(partial=freeze(result),
            actual_swept_clearance_m=clearance,actual_braking_cap_mps=cap,derived_safe_speed_mps=value)
    assert outputs['selected']['partial']['allowed'],outputs['selected']
    rows.append(dict(seq=seq,reference_s=now,command_observed_at_s=command['sim_time_s'],
        command_speed_mps=speed,command_steering_rad=steering,
        previous_steering_source='planner saved input; not exact guard previous safe',
        measured_signed_speed_mps=measured,source_input_sha256=hashlib.sha256(encoded(raw).encode()).hexdigest(),
        projected_points=len(ids),**outputs))
assert hashes=={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
result=dict(scope='saved original r485 sensor/motion/command contract comparison; cold independent policy; no full guard replay',
    guard_state_exact=False,hardware_execution=False,ros_execution=False,source_files_unchanged=True,
    inputs=hashes,rows=rows)
(A/'saved_input_comparison.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(dict(default_exact_rows=len(exact),saved_cases=[dict(seq=r['seq'],
    old=r['legacy']['partial']['reason'],new=r['selected']['partial']['reason'],
    old_speed=r['legacy']['derived_safe_speed_mps'],new_speed=r['selected']['derived_safe_speed_mps']) for r in rows])))
