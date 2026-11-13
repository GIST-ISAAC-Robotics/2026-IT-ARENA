"""저장 시뮬 수치 99개/계약을 한 번씩 계산한다. 타이머는 전체 Evaluator 호출만 포함."""
from pathlib import Path
import argparse,hashlib,json,math,os,statistics,sys,time
from collections import Counter

from contract_reference import estimated_header,estimated_record,request
from diff_utils import comparison
from arena_autonomy.race_runtime_policy.evaluator import Evaluator

ROOT=Path(__file__).resolve().parent
def encoded(v):return json.dumps(v,sort_keys=True,separators=(',',':'),allow_nan=False).encode()
def sha(v):return hashlib.sha256(encoded(v)).hexdigest()
def stats(rows):
    v=sorted(r['evaluator_ms'] for r in rows)
    return dict(n=len(v),median_ms=statistics.median(v),p95_ms=v[math.ceil(.95*len(v))-1],
        max_ms=max(v),over_100ms=sum(x>100. for x in v)) if v else dict(n=0)
def final_decision(o):
    return {**{k:o.get(k) for k in ('speed_mps','steering_rad','reason','reference_s','path_source_s',
        'original_reference_s','path_points','independent_guard_required','runtime_authorized')},
        'current_route_accepted':o.get('current_route_proof',{}).get('accepted')}
def counts(o):
    crop=o.get('current_crop_component_retry') or {};corner=o.get('current_corner_distance_budget') or {}
    return dict(crop_targets=crop.get('attempted_targets',0),crop_full_checks=crop.get('final_full_checks',0),
        crop_applied=crop.get('applied',False),corner_targets=corner.get('additional_target_attempts',0),
        corner_full_checks=corner.get('final_full_checks',0),corner_applied=corner.get('applied',False))


def run(contract):
    records=list(map(json.loads,(ROOT/'inputs/selected.jsonl').open()))
    expected=json.loads((ROOT/'inputs/x86_expected.json').read_bytes())
    headers={d:json.loads((ROOT/'inputs'/f'{d}_header.json').read_bytes()) for d in ('r437','r426')}
    out=ROOT/'cost'/contract;out.mkdir(parents=True,exist_ok=False)
    workers={};rows=[];startup=[];failure=None
    try:
        with (out/'rows.jsonl').open('x') as stream:
            for index,original in enumerate(records):
                d=original['dataset'];key=d+':'+str(original['trace_seq'])
                first=d not in workers
                if first:
                    h=estimated_header(headers[d]) if contract=='estimated' else headers[d]
                    begin=time.perf_counter_ns();workers[d]=Evaluator('current',h)
                    startup.append(dict(dataset=d,constructor_ms=(time.perf_counter_ns()-begin)/1e6))
                record=estimated_record(original) if contract=='estimated' else original
                data=request(record);before=sha(data)
                begin=time.perf_counter_ns();value=workers[d](data);elapsed=(time.perf_counter_ns()-begin)/1e6
                assert before==sha(data),'input mutation'
                if contract=='estimated':
                    assert data['current']['timing_verified'] is False
                    assert data['current']['scan']['measurements']['time_basis']=='unverified_estimate'
                diff=comparison(expected[key],value) if contract=='simulation' else None
                row=dict(index=index,key=key,dataset=d,trace_seq=record['trace_seq'],role=record['selected_role'],
                    contract=contract,evaluator_ms=elapsed,first_for_dataset=first,input_sha256=before,
                    input_unchanged=True,output=value,final_decision=final_decision(value),diagnostic_counts=counts(value),
                    x86_comparison=diff)
                rows.append(row);stream.write(json.dumps(row,allow_nan=False)+'\n');stream.flush()
    except BaseException as exc:
        import traceback
        failure=repr(exc);(out/'traceback.txt').write_text(traceback.format_exc())
    finally:
        for worker in workers.values():worker.close()
    imports={n:str(Path(m.__file__).resolve()) for n,m in sys.modules.items()
        if n.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(m,'__file__',None)}
    external={n:p for n,p in imports.items() if not Path(p).is_relative_to(ROOT/'src')}
    totals=Counter()
    for r in rows:totals.update(r['diagnostic_counts'])
    report=dict(cost_collection_completed=failure is None and len(rows)==99 and not external,error=failure,
        contract=contract,machine=os.uname().machine,calls=len(rows),additional_warmup_calls=0,
        all_calls=stats(rows),after_first_contract_call=stats(rows[1:]),
        r437=stats([r for r in rows if r['dataset']=='r437']),
        first_calls=[{k:r[k] for k in ('key','evaluator_ms','first_for_dataset')} for r in rows if r['first_for_dataset']],
        constructors=startup,counts=dict(totals),reasons=dict(Counter(r['output']['reason'] for r in rows)),
        positive=sum(r['output']['speed_mps']>0 for r in rows),external_imports=external,imports=imports,
        physical_measurement=False,timing_accuracy_verified=False,devices_opened=False)
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k!='imports'}),flush=True)
    return 0 if report['cost_collection_completed'] else 1


def summarize():
    rows={c:list(map(json.loads,(ROOT/'cost'/c/'rows.jsonl').open())) for c in ('simulation','estimated')}
    pairs=[]
    for a,b in zip(rows['simulation'],rows['estimated']):
        assert a['key']==b['key'];same=a['final_decision']==b['final_decision']
        pairs.append(dict(key=a['key'],same_arm_final_decision_exact=same,
            final_decision_comparison=comparison(a['final_decision'],b['final_decision']),
            simulation_counts=a['diagnostic_counts'],estimated_counts=b['diagnostic_counts']))
    x86=[r['x86_comparison'] for r in rows['simulation']]
    categories=Counter();maxima={}
    for c in x86:
        categories.update(c['categories'])
        for d in c['differences']:
            if 'absolute_difference' in d:maxima[d['category']]=max(maxima.get(d['category'],0),d['absolute_difference'])
    reports={c:json.loads((ROOT/'cost'/c/'report.json').read_bytes()) for c in rows}
    invariant=len(pairs)==99 and all(p['same_arm_final_decision_exact'] for p in pairs)
    original_equal=all(c['existing_comparator_equal'] for c in x86)
    result=dict(passed=invariant and original_equal,cost_collection_completed=all(r['cost_collection_completed'] for r in reports.values()),
        same_arm_invariants_passed=invariant,x86_existing_comparator_passed=original_equal,
        x86_exact_rows=sum(c['exact_tree_equal'] for c in x86),x86_existing_comparator_failed_rows=sum(not c['existing_comparator_equal'] for c in x86),
        x86_categories=dict(categories),x86_max_absolute_differences=maxima,calls=198,contracts=reports,
        unchanged_input_count=99,synthetic_contract_copy=True,actual_wheel_values_synthesized=False,
        full_ros_realtime_verified=False,physical_driving_verified=False)
    (ROOT/'cost/comparison.json').write_text(json.dumps(pairs,indent=2)+'\n')
    (ROOT/'cost/report.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='contracts'}),flush=True)
    # x86 차이는 원 failed 판정으로 보존하되 완료된 계측을 재실행하지 않는다.
    return 0 if result['cost_collection_completed'] and invariant else 1


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('mode',choices=('simulation','estimated','summarize'));a=p.parse_args()
    raise SystemExit(summarize() if a.mode=='summarize' else run(a.mode))
