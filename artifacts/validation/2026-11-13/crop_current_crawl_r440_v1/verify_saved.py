"""r440 전체 Evaluator 저장409개 비교. 원 r438 baseline을 재계산하지 않는다."""
from pathlib import Path
import hashlib
import json
import math
import statistics
import sys
import time
import traceback
from collections import Counter

from arena_autonomy.follow_trace import freeze
from arena_autonomy.runtime_dual_state import current_request
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy.race_runtime_policy import crop_current_candidate as crop

ROOT=Path(__file__).resolve().parent
def encoded(value):return json.dumps(value,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def digest(value):return hashlib.sha256(encoded(value)).hexdigest()
def raw_expected(expected):
    result=dict(expected);cap=result.pop('new_plan_deceleration_cap',None)
    if cap is not None:result['speed_mps']=cap['checked_target_mps']
    return result
def stats(values):
    if not values:return dict(n=0)
    values=sorted(values)
    return dict(n=len(values),min_ms=values[0],median_ms=statistics.median(values),
        p95_ms=values[math.ceil(.95*len(values))-1],max_ms=values[-1])

def main():
    out=ROOT/'saved_result';out.mkdir(exist_ok=False)
    records=[json.loads(line) for name in ('r437','r426') for line in (ROOT/'inputs'/f'{name}_currents.jsonl').open()]
    evidence=json.loads((ROOT/'inputs/r438_expected.json').read_bytes())
    observed=[];core_calls=[];original=crop._try_once
    def spy(*args,**kwargs):
        cap=kwargs['target_cap'];core=kwargs['core_rebase']
        def actual_core(output,submitted,current,*rest):
            core_calls.append(dict(target=output[0],measured=current['speed_mps']))
            return core(output,submitted,current,*rest)
        kwargs['core_rebase']=actual_core
        answer=original(*args,**kwargs);observed.append(cap);return answer
    crop._try_once=spy
    rows=[];error=None;started=time.monotonic();workers={}
    try:
        with (out/'rows.jsonl').open('x',encoding='utf-8') as stream:
            for record in records:
                dataset=record['dataset'];key=dataset+':'+str(record['trace_seq']);prior=evidence[key]
                if dataset not in workers:
                    workers[dataset]=Evaluator('current',json.loads((ROOT/'inputs'/f'{dataset}_header.json').read_bytes()))
                data=current_request(record['plan_output'],record['plan_input'],record['current'],
                    retained=record['retained'],crop_current_selection=True)
                before=digest(data);observed.clear();core_calls.clear()
                t0=time.perf_counter();result=workers[dataset](data);cost=(time.perf_counter()-t0)*1000
                expected=raw_expected(record['expected']);trial=result.get(crop.DIAGNOSTIC) or {}
                changed=prior['eligible']
                if changed:
                    assert dataset=='r437' and result['speed_mps']==.1,(key,result)
                    assert observed==[.2,.1] and [r['target'] for r in core_calls]==[.2,.1],(key,observed,core_calls)
                    assert all(r['measured']==record['current']['speed_mps'] for r in core_calls)
                    assert trial['attempted_targets']==2 and trial['final_full_checks']==2
                    assert trial['crawl_retry']['applied'] is True
                    assert trial['crawl_retry']['result']==prior['fixed_01_crop'],key
                    assert all(result.get(k)==expected.get(k) for k in ('steering_rad','reference_s','path_source_s','original_reference_s','path_points'))
                    assert result['current_route_proof']['accepted'] is True
                    assert result['current_route_proof']['region']==trial['region']
                    assert result['current_route_proof']['bounded_current_risk']==trial
                    assert trial['region']['masked_tail_extent_m']<=.01 and trial['region']['masked_front_samples']==0
                    assert trial['proof']['accepted'] is True and trial['unknown_filled'] is False
                else:
                    assert result==expected,(key,'noneligible output mismatch')
                    assert .1 not in observed and len(core_calls)<=1,(key,observed,core_calls)
                assert len(observed)<=2 and len(core_calls)<=2
                assert digest(data)==before
                row=dict(dataset=dataset,trace_seq=record['trace_seq'],changed=changed,input_sha256=record['input_sha256'],
                    input_unchanged=True,original_speed_mps=expected['speed_mps'],speed_mps=result['speed_mps'],
                    noneligible_exact=not changed,attempt_caps=list(observed),core_calls=list(core_calls),
                    current_full_cost_ms=cost,output=result)
                rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
    except BaseException as exc:
        error=repr(exc);(out/'traceback.txt').write_text(traceback.format_exc(),encoding='utf-8')
    finally:
        crop._try_once=original
        for worker in workers.values():worker.close()
    summary={}
    for dataset in ('r437','r426'):
        chosen=[r for r in rows if r['dataset']==dataset]
        summary[dataset]=dict(n=len(chosen),changed=sum(r['changed'] for r in chosen),
            unchanged=sum(r['noneligible_exact'] for r in chosen),positive=sum(r['speed_mps']>0 for r in chosen),
            original_positive_preserved=sum(r['original_speed_mps']>0 and r['noneligible_exact'] for r in chosen),
            cost=stats([r['current_full_cost_ms'] for r in chosen]))
    imports={name:str(Path(module.__file__).resolve()) for name,module in sys.modules.items()
        if name.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(module,'__file__',None)}
    external={name:path for name,path in imports.items() if not Path(path).is_relative_to(ROOT/'src')}
    report=dict(passed=error is None and len(rows)==409 and not external,error=error,count=len(rows),datasets=summary,
        elapsed_s=time.monotonic()-started,imports=imports,external_imports=external,
        input_mutations=sum(not row['input_unchanged'] for row in rows),
        timing_scope='WSL current full Evaluator; no Windows timing and no ROS/IPC/Jetson deadline claim')
    (out/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps({k:report[k] for k in ('passed','error','count','datasets','elapsed_s')},ensure_ascii=False))
    return 0 if report['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
