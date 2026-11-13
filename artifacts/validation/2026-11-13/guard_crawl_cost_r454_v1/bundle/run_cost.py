"""r451 국소 입력의 실제 ARM native 비용. 전체 plan/주행 재생이 아니다."""
from pathlib import Path
import copy, hashlib, json, math, os, statistics, sys, time, traceback

ROOT = Path(__file__).resolve().parent
from arena_autonomy import numeric_kernels, _numeric_kernels, pass_release_trial as ps
from arena_autonomy.follow_trace import FollowConfig, MotionConfig, restore_call, freeze
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.race_runtime_policy import guard_crawl
from arena_autonomy.race_runtime_policy.retained_recheck._present import supported


def differences(a, b, path='$'):
    if isinstance(a, dict) and isinstance(b, dict):
        if set(a) != set(b):
            return [dict(path=path, actual_keys=list(a), expected_keys=list(b))]
        return sum((differences(a[k], b[k], path+'.'+k) for k in a), [])
    if isinstance(a, list) and isinstance(b, list):
        if len(a) != len(b):
            return [dict(path=path, lengths=[len(a), len(b)])]
        return sum((differences(x, y, path+f'[{i}]') for i, (x, y) in enumerate(zip(a, b))), [])
    if type(a) is float or type(b) is float:
        if isinstance(a, (int, float)) and isinstance(b, (int, float)) and abs(a-b) <= 1e-12:
            return []
    return [] if a == b else [dict(path=path, actual=a, expected=b)]


def digest(p): return hashlib.sha256(Path(p).read_bytes()).hexdigest()


def stats(values):
    if not values: return dict(n=0)
    ordered=sorted(values)
    return dict(n=len(values), first=values[0], median=statistics.median(values),
                p95=ordered[math.ceil(.95*len(values))-1], maximum=max(values),
                greater_than_100_ms=sum(v>100 for v in values))


def main():
    out=ROOT/'cost'; out.mkdir(exist_ok=False)
    records=[]; error=None
    input_paths=list((ROOT/'inputs').glob('*'))
    before={p.name:digest(p) for p in input_paths}
    numeric_kernels.configure('native_v1')
    assert os.uname().machine=='aarch64' and numeric_kernels.classification_enabled()
    assert Path(_numeric_kernels.__file__).resolve().is_relative_to(ROOT)
    expected={r['seq']:r for r in json.loads((ROOT/'inputs/expected.json').read_bytes())}
    header=json.loads((ROOT/'inputs/header.json').read_bytes())
    cfg=FollowConfig(**header['follow_config']); motion_cfg=MotionConfig(**header['motion_config'])
    rows=[json.loads(line) for line in (ROOT/'inputs/selected.jsonl').open()]
    assert len(rows)==104 and [r['seq'] for r in rows]==list(range(895,999))
    with (out/'rows.jsonl').open('x') as output:
        try:
            for row in rows:
                start=time.perf_counter_ns()
                data=row['data']; normal=data['output'][2]['normal']; evidence=normal['selector_guard_candidate']
                assert evidence['trigger']=='parallel_hold_reference_not_straight' and evidence['baseline_reason']=='local_follow'
                info,scan,history=restore_call(data['input'],motion_cfg)
                scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
                    epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'],
                    timing_policy=info.get('timing_policy','verified_only'))
                owner=ps.PassReleaseSelector(cfg)
                owner.sides=copy.deepcopy(normal['selector_sides'])
                owner.guard_anchor={name:entry['anchor_m'] for name,entry in evidence['guarded_sides'].items()}
                owner.last_choice=normal['selector_choice']
                view=ps.threat_view(scene,normal['path_points'],owner.lat)
                base_detail={**normal,'reason':evidence['baseline_reason']}
                baseline=(evidence['baseline_speed_mps'],evidence['steering_rad'],base_detail)
                parallel=(*data['output'][:2],{'reason':evidence['trigger']})
                signed=info['speed_mps']
                args=(scene,view,baseline,parallel,abs(signed),evidence['previous_steering_rad'],
                      info['dt_s'],scene.t_ref_s-scene.source_first_s,{},scan)
                before_state=copy.deepcopy((owner.sides,owner.guard_anchor,owner.last_choice))
                timed=[]; original_method=owner._guard_curve_at_cap
                def timed_guard(*args, **kwargs):
                    began=time.perf_counter_ns()
                    result=original_method(*args, **kwargs)
                    elapsed=time.perf_counter_ns()-began
                    timed.append(dict(target_cap=kwargs.get('target_cap',.35),elapsed_ns=elapsed))
                    return result
                owner._guard_curve_at_cap=timed_guard
                setup_ns=time.perf_counter_ns()-start
                began=time.perf_counter_ns()
                original=owner._guard_curve_original(*args)
                original_ns=time.perf_counter_ns()-began
                assert [v['target_cap'] for v in timed]==[.35,.25,.15]
                errors=differences(freeze(original[2]['selector_guard_candidate']),evidence,'$.original.guard')
                errors+=differences(original[2]['selector_reason'],normal['selector_reason'],'$.original.reason')
                errors+=differences(freeze(original[2]['selector_guard_speed_recheck']),normal['selector_guard_speed_recheck'],'$.original.retry')
                rec=dict(seq=row['seq'],signed_speed_mps=signed,setup_ns=setup_ns,original_ns=original_ns,
                    original_calls=copy.deepcopy(timed),original_match=not errors,differences=errors,
                    original_output=freeze(original),candidate_evaluated=False)
                if not errors:
                    # 실제 계산된 세 검사 증거를 전달한다. 기록으로 새 검사 결과를 덮지 않는다.
                    selected=(original[0],original[1],{**normal,**original[2]})
                    began=time.perf_counter_ns()
                    result=guard_crawl.assess(owner,scene,baseline,selected,abs(signed),
                        evidence['previous_steering_rad'],info['dt_s'],scan,measured=signed)
                    adapter_ns=time.perf_counter_ns()-began
                    candidate_calls=timed[3:]
                    assert len(candidate_calls)==1 and candidate_calls[0]['target_cap']==.1
                    low_ns=candidate_calls[0]['elapsed_ns']
                    checked=freeze(result); retry=checked[2].get('selector_guard_crawl_retry') or {}
                    errors+=differences(checked,expected[row['seq']]['adapter_result'],'$.integrated')
                    current_supported=supported((result[0],result[1],{**data['output'][2],'normal':result[2]}))
                    errors+=differences(current_supported,True,'$.current_supported')
                    rec.update(candidate_evaluated=True,adapter_ns=adapter_ns,low_guard_ns=low_ns,
                        adapter_other_ns=adapter_ns-low_ns,total_guard_with_adapter_ns=original_ns+adapter_ns,
                        candidate_calls=candidate_calls,adapter_match=not errors,candidate_output=checked,
                        accepted=retry.get('applied') is True and result[0]>.0,current_supported=current_supported)
                owner._guard_curve_at_cap=original_method
                rec['selector_state_unchanged']=(owner.sides,owner.guard_anchor,owner.last_choice)==before_state
                assert rec['selector_state_unchanged']
                records.append(rec)
                output.write(json.dumps(rec,allow_nan=False)+'\n'); output.flush()
                if errors: break
        except BaseException as exc:
            error=repr(exc);(out/'traceback.txt').write_text(traceback.format_exc())
    modules={k:str(Path(v.__file__).resolve()) for k,v in sys.modules.items()
             if k.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(v,'__file__',None)}
    external={k:v for k,v in modules.items() if not Path(v).is_relative_to(ROOT)}
    changed=[p.name for p in input_paths if digest(p)!=before[p.name]]
    costs={k:stats([r[k]/1e6 for r in records if k in r]) for k in
        ('original_ns','adapter_ns','low_guard_ns','adapter_other_ns','total_guard_with_adapter_ns','setup_ns')}
    for cap in (.35,.25,.15):
        costs['cap_'+str(cap)+'_ns']=stats([v['elapsed_ns']/1e6 for r in records for v in r['original_calls'] if v['target_cap']==cap])
    report=dict(passed=error is None and len(records)==104 and not external and not changed
        and all(not r['differences'] and r['selector_state_unchanged'] for r in records),error=error,
        rows=len(records),original_matches=sum(r['original_match'] for r in records),
        candidate_evaluations=sum(r['candidate_evaluated'] for r in records),
        candidate_accepted=sum(r.get('accepted',False) for r in records),
        current_supported=sum(r.get('current_supported',False) for r in records),
        first_mismatch=next((dict(seq=r['seq'],differences=r['differences']) for r in records if r['differences']),None),
        costs_ms=costs,source_mode='native_v1',python=sys.version,native_file=_numeric_kernels.__file__,
        native_sha256=digest(_numeric_kernels.__file__),abi=_numeric_kernels.ABI_VERSION,
        changed_inputs=changed,module_paths=modules,external_project_imports=external,
        comparator='r451 absolute float 1e-12 and exact integers/structure',warmup_calls=0,
        scope='recorded local GUARD function inputs; no full plan replay, ROS load or driving',
        timing='original three-cap function then adapter per row; perf_counter_ns; one timing-only wrapper per guard call',
        timing_excludes='input restore/scene/view setup, comparison/serialization, process/import/setup, independent current execution',
        current_supported_is_dispatch_contract_only=True,physical_driving_verified=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('module_paths','first_mismatch','costs_ms')}),flush=True)
    if report['first_mismatch']: print(json.dumps(report['first_mismatch'])[:5000],flush=True)
    return 0 if report['passed'] else 2


if __name__=='__main__': raise SystemExit(main())
