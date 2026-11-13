"""기록된 국소 GUARD 입력만 비교한다. 전체 plan 재생이나 실제 주행의 대용이 아니다."""
from pathlib import Path
import copy
import hashlib
import json
import sys

B = Path(__file__).resolve().parent
S = B.parent / 'guard_crawl_saved_r445_v1' / 'stage_v3'
sys.path[:0] = [str(S / 'src/arena_autonomy'), str(S / 'src/arena_vehicle_interface')]
from arena_autonomy.numeric_kernels import configure
configure('python')
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, restore_call, freeze
from arena_autonomy.scene_motion import motion_scene


def differences(a, b, path='$'):
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):
            return [dict(path=path, actual_keys=list(a), expected_keys=list(b))]
        return sum((differences(a[k], b[k], path+'.'+k) for k in a), [])
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [dict(path=path, lengths=[len(a),len(b)])]
        return sum((differences(x,y,path+f'[{i}]') for i,(x,y) in enumerate(zip(a,b))), [])
    if type(a) is float or type(b) is float:
        if isinstance(a,(int,float)) and isinstance(b,(int,float)) and abs(a-b)<=1e-12:
            return []
    if a != b:
        return [dict(path=path, actual=a, expected=b)]
    return []


def run():
    out = B / 'result_v1'
    out.mkdir(exist_ok=False)
    trace = S / 'inputs/follow_decisions.jsonl'
    rows = [json.loads(line) for line in trace.open(encoding='utf8')]
    header = rows[0]['data']
    cfg = FollowConfig(**header['follow_config'])
    motion_cfg = MotionConfig(**header['motion_config'])
    checks=[]
    for row in rows:
        if row['kind']!='call' or not 895<=row['seq']<=998:
            continue
        data=row['data']; normal=data['output'][2]['normal']
        evidence=normal['selector_guard_candidate']
        assert evidence['trigger']=='parallel_hold_reference_not_straight'
        assert evidence['baseline_reason']=='local_follow'
        assert normal['selector_mode']=='GUARD'
        info,scan,history=restore_call(data['input'],motion_cfg)
        scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
            epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'],
            timing_policy=info.get('timing_policy','verified_only'))
        owner=ps.PassReleaseSelector(cfg)
        owner.sides=copy.deepcopy(normal['selector_sides'])
        owner.guard_anchor={name:entry['anchor_m'] for name,entry in evidence['guarded_sides'].items()}
        # trigger는 reference_not_straight. 이 국소 함수는 last_choice/last_present에 의존하지 않는다.
        owner.last_choice=normal['selector_choice']
        view=ps.threat_view(scene,normal['path_points'],owner.lat)
        base_detail={**normal,'reason':evidence['baseline_reason']}
        baseline=(evidence['baseline_speed_mps'],evidence['steering_rad'],base_detail)
        parallel=(data['output'][0],data['output'][1],{'reason':evidence['trigger']})
        # forward_drive의 기존 abs 전달을 재현한다. 후보는 원 signed 값이 음수면 시험하지 않는다.
        signed=info['speed_mps']
        args=(scene,view,baseline,parallel,abs(signed),evidence['previous_steering_rad'],
              info['dt_s'],scene.t_ref_s-scene.source_first_s,{},scan)
        original=owner._guard_curve_at_cap(*args)
        actual=original[2]['selector_guard_candidate']
        errors=differences(freeze(actual),evidence)
        errors+=differences(original[2]['selector_reason'],normal['selector_reason'],'$.reason')
        rechecks=[]
        if not errors:
            for saved in normal['selector_guard_speed_recheck']['attempts']:
                checked=owner._guard_curve_at_cap(*args,target_cap=saved['target_mps'])
                cand=checked[2]['selector_guard_candidate']
                observed=dict(target_mps=cand['target_speed_mps'],accepted=bool(checked[0]>0 and cand['accepted']),
                    reason=checked[2]['selector_reason'],region=cand['region'])
                errors+=differences(freeze(observed),saved,'$.retry')
                rechecks.append(observed)
        candidate=None
        if not errors and signed>=0:
            checked=owner._guard_curve_at_cap(*args,target_cap=.1)
            candidate=dict(speed_mps=checked[0],steering_rad=checked[1],reason=checked[2]['selector_reason'],
                detail=checked[2]['selector_guard_candidate'])
        checks.append(dict(seq=row['seq'],signed_speed_mps=signed,original_match=not errors,
            differences=errors,lower_original=rechecks,candidate=candidate))
        # Reconstruction failure is evidence, not permission to fill missing state.
        if errors:
            break
    modules={k:str(Path(v.__file__).resolve()) for k,v in sys.modules.items()
             if k.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(v,'__file__',None)}
    assert all(str(S.resolve()) in v for v in modules.values())
    summary=dict(scope='Recorded local GUARD function inputs, not full plan state replay or closed-loop driving',
        trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
        original_rows=len(checks),original_matches=sum(c['original_match'] for c in checks),
        candidate_evaluations=sum(c['candidate'] is not None for c in checks),
        candidate_accepted=sum(c['candidate'] is not None and c['candidate']['speed_mps']>0 for c in checks),
        passed=bool(checks) and all(c['original_match'] for c in checks),
        source_mode='python',full_plan_reconstructed=False,common_changes=0,
        actor_truth_used=False,geometry_or_limits_changed=False,module_paths=modules)
    (out/'checks.json').write_text(json.dumps(checks,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in summary.items() if k!='module_paths'},ensure_ascii=False))
    if checks and checks[-1]['differences']:
        print(json.dumps(checks[-1]['differences'][:5],ensure_ascii=False))
    return 0 if summary['passed'] else 2


if __name__=='__main__':
    raise SystemExit(run())
