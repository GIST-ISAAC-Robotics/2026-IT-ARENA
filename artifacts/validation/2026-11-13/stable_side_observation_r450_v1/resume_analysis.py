"""기존 합성은 재실행하지 않고 집계, 출발 유지에 필요한 두 helper 사례만 추가."""
import ast
from dataclasses import replace
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

B=Path(__file__).resolve().parent;O=B/'result_v2';O.mkdir(exist_ok=False)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
preserved=[B/'analyze.py',B/'original_static_side_return.py',B/'PAUSED.md',*sorted((B/'result_v1').glob('*.json'))]
before={str(p.relative_to(B)):sha(p) for p in preserved}
spec=importlib.util.spec_from_file_location('r450_original_resume',B/'original_static_side_return.py')
mod=importlib.util.module_from_spec(spec);sys.modules[spec.name]=mod;spec.loader.exec_module(mod)
Tracker,Config=mod.StaticSideTracker,mod.StaticSideConfig
syntax=ast.parse((B/'analyze.py').read_text(encoding='utf8'))
parts=[n for n in syntax.body if (isinstance(n,ast.ClassDef) and n.name=='StableRelativeDiagnostic') or
       (isinstance(n,ast.FunctionDef) and n.name=='snapshot')]
exec(compile(ast.Module(body=parts,type_ignores=[]),'original_r450_candidate_only','exec'))

def intervals(rows,key):
    active=None;out=[]
    for r in rows:
        if r[key] and active is None:active=r['t']
        if not r[key] and active is not None:
            out.append(dict(first_true_s=active,first_false_s=r['t']));active=None
    if active is not None:out.append(dict(first_true_s=active,first_false_s=None,observed_through_s=rows[-1]['t']))
    return out

def classify_latch(rows):
    """rows가 clear된 관찰에서 철회하는 별도 진단. 제어/상태에 적용하지 않는다."""
    active=False;previous=[];out=[]
    for source in rows:
        r=dict(source);current=r['candidate_state']['row_data']
        # 기존 창 이동은 7행을 공유한다. clear/reset 후에는 이전 행이 남지 않는다.
        cleared=bool(previous) and not any(tuple(x) in {tuple(y) for y in current} for x in previous)
        if cleared:active=False
        if r['stable_relative']:active=True
        r.update(rows_cleared=cleared,hold_until_rows_clear=active)
        out.append(r);previous=current
    return out

saved=json.loads((B/'result_v1/synthetic_rows.json').read_text(encoding='utf8'))
saved_analyses=[]
events={'stopped_neighbor_then_moves_0.2':1.,'stopped_neighbor_then_moves_0.05':1.,
        'long_gap':1.5,'signed_negative_helper_input':1.1}
for case in saved:
    rows=classify_latch(case['rows']);event=events.get(case['name'])
    first_false=next((r['t'] for r in rows if event is not None and r['t']>event and not r['stable_relative']),None)
    # 공백/음수 입력 자체가 철회 조건이면 같은 관찰 시각을 포함한다.
    if case['name'] in ('long_gap','signed_negative_helper_input'):
        first_false=next((r['t'] for r in rows if r['t']>=event and not r['stable_relative']),None)
    reacquire=next((r['t'] for r in rows if first_false is not None and r['t']>first_false and r['stable_relative']),None)
    saved_analyses.append(dict(name=case['name'],source='result_v1/synthetic_rows.json; no reexecution',
        intervals=intervals(rows,'stable_relative'),latch_intervals=intervals(rows,'hold_until_rows_clear'),
        event_s=event,first_false_after_event_s=first_false,
        delay_s=None if first_false is None or event is None else first_false-event,
        reacquired_s=reacquire,reacquire_delay_from_first_false_s=None if reacquire is None else reacquire-first_false,
        cleared_at_s=[r['t'] for r in rows if r['rows_cleared']],
        recompute_latch_difference_times=[r['t'] for r in rows if r['stable_relative']!=r['hold_until_rows_clear']],
        rows=rows))

def start_case(name,front_mode):
    tracker=StableRelativeDiagnostic();rows=[];previous_speed=0.;travel=0.
    for i in range(31):
        t=i/10;speed=0. if t<=1. else .1
        if i:travel+=(previous_speed+speed)*.5*.1
        previous_speed=speed
        # 둘 다 1초까지 정지. 이후 (a) 고정 상대 (b) 자차와 동속 상대.
        front=.6-travel if front_mode=='stationary' else .6
        tracker.advance_to(t,speed);answer=tracker.observe(t,front)
        rows.append(dict(t=t,ego_speed=speed,front_ds=front,stable_relative=answer,
                         strict_float=tracker.strict_float_result,candidate_state=snapshot(tracker)))
    rows=classify_latch(rows)
    return dict(name=name,speed_change_interval_s=[1.,1.1],first_nonzero_speed_sample_s=1.1,
        assumption='측정 양끝 속도와 trapezoid로 합성 travel; 실제 가속/슬립/장애물 시험 아님.',
        intervals=intervals(rows,'stable_relative'),latch_intervals=intervals(rows,'hold_until_rows_clear'),
        strict_float_intervals=intervals(rows,'strict_float'),rows=rows,
        recompute_latch_difference_times=[r['t'] for r in rows if r['stable_relative']!=r['hold_until_rows_clear']])
new_cases=[start_case('ego_starts_0.1_stationary_return','stationary'),start_case('ego_and_neighbor_start_together_0.1','same_speed')]
observed=json.loads((B/'result_v1/observed_helper_rows.json').read_text(encoding='utf8'))
observed_hold={}
for side in ('left','right'):
    translated=[dict(t=r['reference_s'],stable_relative=bool(r['sides'][side]['stable_relative']),
                    candidate_state=r['sides'][side]['after_observe_candidate']) for r in observed if r['can_observe']]
    checked=classify_latch(translated)
    observed_hold[side]=dict(recomputed_intervals=intervals(checked,'stable_relative'),held_intervals=intervals(checked,'hold_until_rows_clear'),
                            difference_times=[r['t'] for r in checked if r['stable_relative']!=r['hold_until_rows_clear']])

summary=dict(existing_synthetic_cases_rerun=0,new_helper_cases=2,full_plan_calls=0,control_or_occupancy_changes=False,
    old_config=vars(Config()),new_config=vars(StableRelativeDiagnostic().config),
    numeric_time='원 r450 후보의 strict float 및 ns .7초 대조를 그대로 사용. 판정선 신규 변경 없음.',
    saved_cases=[{k:v for k,v in r.items() if k!='rows'} for r in saved_analyses],
    new_cases=[{k:v for k,v in r.items() if k!='rows'} for r in new_cases],observed_latch=observed_hold,
    signed_scope='기존 forward_drive abs를 바꾸지 않았다. 직접 음수 helper 반례는 실제 caller가 원 signed 값을 보존한다는 증거가 아니다.',
    window_drift_limit_mps=.05/.7,
    drift_limit_scope='일정 이동·10Hz·길이8 창의 결정론적 합성에서 .7초 이동이 .05m 이하이면 잔차 reset을 유발하지 않을 수 있음. 센서 보증치 아님.',
    source_hashes_before=before,source_hashes_after={str(p.relative_to(B)):sha(p) for p in preserved},
    windows_only=True,wsl_used=False,jetson_used=False,subprocesses_started=False)
assert summary['source_hashes_before']==summary['source_hashes_after']
for name,obj in [('saved_case_delays.json',saved_analyses),('start_transition_rows.json',new_cases),('summary.json',summary)]:
    (O/name).write_text(json.dumps(obj,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in summary.items() if k not in ('source_hashes_before','source_hashes_after')},ensure_ascii=False,indent=2))
