"""고정 .2/.15/.1 목표의 저장 current 비교. ROS/주행/장치/공통 수정 없음."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys
import time
import traceback
from collections import Counter

from arena_autonomy.follow_trace import freeze,restore_call
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.runtime_dual_state import current_request
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy import pass_release_trial as ps
from fixed_target_candidate import try_from_current_result,DIAGNOSTIC

MASKED=('large_motion_masked_toward_left','large_motion_masked_toward_right')
def encoded(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def digest(value):return hashlib.sha256(encoded(value)).hexdigest()
def difference(a,b,path='$'):
    if type(a) is not type(b):return dict(path=path,expected_type=type(a).__name__,actual_type=type(b).__name__)
    if isinstance(a,dict):
        if a.keys()!=b.keys():return dict(path=path,missing=sorted(a.keys()-b.keys()),extra=sorted(b.keys()-a.keys()))
        for key in a:
            found=difference(a[key],b[key],path+'.'+key)
            if found:return found
    elif isinstance(a,list):
        if len(a)!=len(b):return dict(path=path,expected_length=len(a),actual_length=len(b))
        for i,(x,y) in enumerate(zip(a,b)):
            found=difference(x,y,path+'['+str(i)+']')
            if found:return found
    elif a!=b:return dict(path=path,expected=a,actual=b)
    return None

def raw_recorded(expected):
    result=dict(expected);cap=result.pop('new_plan_deceleration_cap',None)
    if cap is not None:result['speed_mps']=cap['checked_target_mps']
    return result

def eligible(before,current):
    trial=before.get(DIAGNOSTIC) or {};region=trial.get('region') or {};proof=trial.get('proof') or {}
    speed=current['speed_mps']
    return (before['speed_mps']==0 and before['reason']=='route_current_large_motion_region_unobserved'
        and trial.get('applied') is False and trial.get('reason')=='current_crop_full_region_rejected'
        and trial.get('blocked') in MASKED and proof.get('accepted') is True
        and type(speed) in (int,float) and math.isfinite(speed) and 0<=speed<=.15
        and region.get('masked_front_samples')==0 and region.get('queries',0)>0
        and trial.get('checked_target_mps',0)>.1)

def compact(result):
    trial=result.get(DIAGNOSTIC) or {}
    return dict(speed_mps=result['speed_mps'],steering_rad=result['steering_rad'],reason=result['reason'],
        clearance_m=result.get('clearance_m'),crop=trial,output_sha256=digest(result))

def stats(values):
    if not values:return dict(n=0)
    values=sorted(values)
    return dict(n=len(values),minimum_ms=values[0],median_ms=statistics.median(values),
        p95_ms=values[min(len(values)-1,math.ceil(.95*len(values))-1)],maximum_ms=values[-1])

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--root',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True);args=parser.parse_args()
    root=args.root.resolve();out=args.output.resolve();out.mkdir(exist_ok=False)
    provenance=json.loads((root/'preparation.json').read_bytes())
    assert provenance['numeric_source_sha256'][provenance['copied_native_binary']['path'].replace('\\','/')]==provenance['copied_native_binary']['sha256']
    def source_changes():
        return [name for name,want in provenance['frozen_package_files'].items()
            if hashlib.sha256((root/name).read_bytes()).hexdigest()!=want]
    assert not source_changes()
    assert ps.SelectorConfig().masked_tail_allowance_m==.01
    assert all(hashlib.sha256((root/'inputs'/name).read_bytes()).hexdigest()==want for name,want in provenance['inputs'].items())
    rows=[];error=None;started=time.monotonic();evaluators=[]
    try:
        with (out/'rows.jsonl').open('x',encoding='utf-8') as stream:
            for dataset in ('r437','r426'):
                header=json.loads((root/'inputs'/f'{dataset}_header.json').read_bytes())
                evaluator=Evaluator('current',header);evaluators.append(evaluator)
                records=[json.loads(line) for line in (root/'inputs'/f'{dataset}_currents.jsonl').open(encoding='utf-8')]
                if dataset=='r437':records.sort(key=lambda row:row['trace_seq']!=643)
                for record in records:
                    if time.monotonic()-started>240:raise TimeoutError('saved comparison exceeded 240 seconds')
                    source_digest=digest({key:record[key] for key in ('plan_input','plan_output','current','retained')})
                    assert source_digest==record['input_sha256']
                    data=current_request(record['plan_output'],record['plan_input'],record['current'],
                        retained=record['retained'],crop_current_selection=True)
                    request_digest=digest(data)
                    t0=time.perf_counter();baseline=evaluator(data);base_ms=(time.perf_counter()-t0)*1000
                    expected_diff=difference(raw_recorded(record['expected']),baseline)
                    if expected_diff:raise AssertionError(dict(dataset=dataset,seq=record['trace_seq'],baseline_diff=expected_diff))
                    use_low=eligible(baseline,data['current'])
                    row=dict(dataset=dataset,index=record['index'],trace_seq=record['trace_seq'],
                        input_sha256=source_digest,request_sha256=request_digest,baseline_exact=True,
                        measured_speed_mps=data['current']['speed_mps'],original_plan_target_mps=data['output'][0],
                        eligible=use_low,baseline=compact(baseline),baseline_full_ms=base_ms,trials={})
                    if dataset=='r437' or use_low:
                        info,scan,history=restore_call(data['current'],evaluator.motion)
                        scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
                            epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'],timing_policy=evaluator.time_policy)
                        for cap in (.2,.15,.1):
                            t0=time.perf_counter()
                            candidate=freeze(try_from_current_result(baseline,data,scene,history,evaluator.follow.config,
                                registered_environment=evaluator.corner_budget,
                                core_rebase=evaluator.current_policy.core_policy.rebase,trial_cap_mps=cap))
                            elapsed=(time.perf_counter()-t0)*1000
                            if cap==.2:
                                found=difference(baseline,candidate)
                                if found:raise AssertionError(dict(dataset=dataset,seq=record['trace_seq'],same_cap_diff=found))
                            geometry_keys=('steering_rad','reference_s','path_source_s','original_reference_s','path_points')
                            assert all(candidate.get(k)==baseline.get(k) for k in geometry_keys)
                            proof=(candidate.get(DIAGNOSTIC) or {}).get('proof') or {}
                            region=(candidate.get(DIAGNOSTIC) or {}).get('region') or {}
                            if candidate['speed_mps']>0:
                                assert candidate[DIAGNOSTIC]['applied'] and 0<candidate['speed_mps']<=cap
                                assert region['masked_front_samples']==0 and region['masked_tail_extent_m']<=.01
                                assert not region['unobserved'] or proof['accepted'] is True
                            row['trials'][str(cap)]=dict(result=compact(candidate),additional_current_check_ms=elapsed)
                    # 실제 연결을 가정한 변경은 eligibility 이후 단일 cap 재검사뿐이다.
                    row['gated_candidate_outputs']={}
                    for cap in (.15,.1):
                        trial=row['trials'].get(str(cap));after=trial['result'] if use_low and trial and trial['result']['speed_mps']>0 else row['baseline']
                        row['gated_candidate_outputs'][str(cap)]=dict(speed_mps=after['speed_mps'],output_sha256=after['output_sha256'],
                            unchanged=after['output_sha256']==row['baseline']['output_sha256'])
                        if baseline['speed_mps']>0 or not use_low:assert after['output_sha256']==digest(baseline)
                    assert digest(data)==request_digest
                    assert digest({key:record[key] for key in ('plan_input','plan_output','current','retained')})==source_digest
                    row['input_unchanged']=True;stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush();rows.append(row)
                    if len(rows)%50==0:print(json.dumps(dict(completed=len(rows),dataset=dataset)),flush=True)
    except BaseException as exc:
        error=repr(exc);(out/'traceback.txt').write_text(traceback.format_exc(),encoding='utf-8')
    finally:
        for evaluator in evaluators:evaluator.close()
    datasets={}
    for dataset in ('r437','r426'):
        selected=[row for row in rows if row['dataset']==dataset]
        summary=dict(cases=len(selected),baseline_exact=sum(row['baseline_exact'] for row in selected),
            baseline_positive=sum(row['baseline']['speed_mps']>0 for row in selected),eligible=sum(row['eligible'] for row in selected),
            baseline_reasons=dict(Counter(row['baseline']['reason'] for row in selected)),
            measured_speed_range=([min(row['measured_speed_mps'] for row in selected),max(row['measured_speed_mps'] for row in selected)] if selected else None),
            baseline_full_cost=stats([row['baseline_full_ms'] for row in selected]),targets={})
        for cap in (.2,.15,.1):
            attempted=[row for row in selected if str(cap) in row['trials']]
            results=[row['trials'][str(cap)]['result'] for row in attempted]
            regions=[r['crop']['region'] for r in results if r['crop'].get('region')]
            summary['targets'][str(cap)]=dict(attempted=len(attempted),accepted=sum(r['speed_mps']>0 for r in results),
                proof_accepted=sum((r['crop'].get('proof') or {}).get('accepted') is True for r in results),
                blocked=dict(Counter(str(r['crop'].get('blocked')) for r in results)),
                tail_range_m=([min(r['masked_tail_extent_m'] for r in regions),max(r['masked_tail_extent_m'] for r in regions)] if regions else None),
                unobserved_range=([min(r['unobserved'] for r in regions),max(r['unobserved'] for r in regions)] if regions else None),
                additional_check_cost=stats([row['trials'][str(cap)]['additional_current_check_ms'] for row in attempted]))
            if cap!=.2:
                summary['targets'][str(cap)].update(gated_recovered=sum(row['eligible'] and not row['gated_candidate_outputs'][str(cap)]['unchanged'] for row in selected),
                    noneligible_exact=sum(not row['eligible'] and row['gated_candidate_outputs'][str(cap)]['unchanged'] for row in selected),
                    baseline_positive_preserved=sum(row['baseline']['speed_mps']>0 and row['gated_candidate_outputs'][str(cap)]['unchanged'] for row in selected))
        datasets[dataset]=summary
    imports={name:str(Path(module.__file__).resolve()) for name,module in sys.modules.items()
        if name.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(module,'__file__',None)}
    external={name:path for name,path in imports.items() if not Path(path).is_relative_to(root/'src')}
    report=dict(passed=error is None and len(rows)==409 and not source_changes() and not external,
        error=error,total_rows=len(rows),elapsed_s=time.monotonic()-started,datasets=datasets,
        input_mutations=sum(not row['input_unchanged'] for row in rows),frozen_source_changes=source_changes(),
        imports=imports,external_imports=external,masked_tail_allowance_m=ps.SelectorConfig().masked_tail_allowance_m,
        source='r437 frozen Python plus exact archived native binary',hardware_or_ros_started=False,
        timing_scope='WSL sequential saved current compute; no ROS/IPC/Jetson deadline claim')
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    print(json.dumps({key:report[key] for key in ('passed','error','total_rows','elapsed_s','datasets')},ensure_ascii=False),flush=True)
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
