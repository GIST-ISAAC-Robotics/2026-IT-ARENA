"""패키지 evaluator를223개에1회 호출하고 내부 baseline/최종 출력을 각각 원 결과와 exact 대조."""
import argparse
from collections import Counter
import copy
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import time
import traceback
from types import SimpleNamespace as NS


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def first_difference(a,b,path='$'):
    if type(a) is not type(b): return dict(path=path,kind='type',expected=repr(a),actual=repr(b))
    if isinstance(a,dict):
        if a.keys()!=b.keys(): return dict(path=path,kind='keys',missing=sorted(a.keys()-b.keys()),extra=sorted(b.keys()-a.keys()))
        for key in a:
            difference=first_difference(a[key],b[key],path+'.'+key)
            if difference:return difference
    elif isinstance(a,list):
        if len(a)!=len(b):return dict(path=path,kind='length',expected=len(a),actual=len(b))
        for index,(x,y) in enumerate(zip(a,b)):
            difference=first_difference(x,y,path+'['+str(index)+']')
            if difference:return difference
    elif a!=b:return dict(path=path,kind='value',expected=a,actual=b)
    return None


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    destination=parser.parse_args().output.resolve();root=Path(__file__).resolve().parent
    assert destination.parent==root and not destination.exists();destination.mkdir()
    manifest=json.loads((root/'bundle_manifest.json').read_bytes())
    def changes():return [name for name,sha in manifest['files'].items() if not (root/name).is_file() or digest(root/name)!=sha]
    assert not changes()
    from arena_autonomy.follow_trace import freeze
    from arena_autonomy.race_runtime_policy.evaluator import Evaluator
    from arena_autonomy.race_runtime_policy import corner_candidate
    header=json.loads((root/'inputs/header.json').read_bytes())
    evaluator=Evaluator('current',header)
    original=evaluator.current_policy
    class CurrentCapture:
        """이 evaluator 인스턴스만 감싼다. 패키지 모듈/전역 함수는 변경하지 않는다."""
        def __init__(self):
            self.core_policy=NS(rebase=self.core);self.clear()
        def clear(self):self.calls=self.extra_core_calls=0;self.baseline=None;self.baseline_ms=None
        def rebase(self,*args,**kwargs):
            self.calls+=1;start=time.perf_counter_ns()
            result=original.rebase(*args,**kwargs)
            self.baseline_ms=(time.perf_counter_ns()-start)/1e6;self.baseline=freeze(result)
            return result
        def core(self,*args,**kwargs):
            self.extra_core_calls+=1
            return original.core_policy.rebase(*args,**kwargs)
    capture=CurrentCapture();evaluator.current_policy=capture
    records=[json.loads(line) for line in (root/'inputs/current.jsonl').read_text(encoding='utf8').splitlines()]
    expected=[json.loads(line) for line in (root/'inputs/expected.jsonl').read_text(encoding='utf8').splitlines()]
    negatives=[json.loads(line) for line in (root/'inputs/counterexamples.jsonl').read_text(encoding='utf8').splitlines()]
    assert len(records)==223 and len(expected)==226 and len(negatives)==3
    references={(row['phase'],row['index'] if row['phase']=='stored' else row['name']):row for row in expected}
    results=[];negative_results=[];error=None;difference=None;source_changes=[]
    def evaluate(record,phase,stream):
        nonlocal difference
        key=record['index'] if phase=='stored' else record['name'];reference=references[(phase,key)]
        data=record['data'];before_input=copy.deepcopy(data);capture.clear();start=time.perf_counter_ns()
        if phase=='counterexample' and record['name']=='current_time_unverified':
            try:evaluator(data)
            except ValueError as exc:
                assert str(exc)=='current corner budget requires verified input timing',repr(exc)
                assert capture.calls==capture.extra_core_calls==0 and data==before_input
                assert reference['input_rejected'] and not reference['candidate_evaluated']
                row=dict(phase=phase,name=key,index=record['index'],passed=True,input_rejected=True,
                    baseline_evaluated=False,candidate_evaluated=False,stage='runtime_input_timing_gate',
                    exception=repr(exc),original_stage=reference['stage'],original_exception=reference['exception'],
                    physical_stop_verified=False)
            else:raise AssertionError('unverified input was not rejected')
        else:
            after=evaluator(data);total_ms=(time.perf_counter_ns()-start)/1e6
            assert capture.calls==1 and data==before_input
            for arm,actual in (('baseline',capture.baseline),('candidate',after)):
                difference=first_difference(reference[arm],actual)
                if difference:raise AssertionError((phase,key,arm,difference))
            diagnostic=after.get(corner_candidate.DIAGNOSTIC) or {}
            applied=diagnostic.get('applied') is True
            assert capture.extra_core_calls==diagnostic.get('additional_target_attempts',0)<=1
            assert diagnostic.get('final_full_checks',0)<=1
            if phase=='counterexample':assert not applied and after['speed_mps']==0
            if not reference['eligible']:assert after==capture.baseline
            elif not applied:assert {k:v for k,v in after.items() if k!=corner_candidate.DIAGNOSTIC}==capture.baseline
            row=dict(phase=phase,index=record['index'],name=record.get('name'),passed=True,
                baseline_exact=True,candidate_exact=True,baseline_calls=capture.calls,
                extra_core_calls=capture.extra_core_calls,eligible=reference['eligible'],applied=applied,
                baseline=capture.baseline,candidate=after,input_unchanged=True,
                baseline_wall_ms=capture.baseline_ms,evaluator_wall_ms=total_ms)
        stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n');stream.flush()
        return row
    try:
        with (destination/'rows.jsonl').open('x',encoding='utf8') as stream:
            for record in records:results.append(evaluate(record,'stored',stream))
            for record in negatives:negative_results.append(evaluate(record,'counterexample',stream))
    except Exception as exc:
        error=repr(exc);(destination/'traceback.txt').write_text(traceback.format_exc(),encoding='utf8')
    finally:
        evaluator.close();source_changes=changes()
        imported={};external=[]
        for name,module in sorted(sys.modules.items()):
            if name.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(module,'__file__',None):
                path=Path(module.__file__).resolve()
                if not path.is_relative_to(root):external.append(dict(module=name,path=str(path)))
                else:imported[name]=dict(path=path.relative_to(root).as_posix(),sha256=digest(path))
        baseline_positive=sum(r['baseline']['speed_mps']>0 for r in results)
        candidate_positive=sum(r['candidate']['speed_mps']>0 for r in results)
        report=dict(passed=error is None and len(results)==223 and len(negative_results)==3
            and not source_changes and not external,error=error,first_difference=difference,
            completed_current=len(results),completed_counterexamples=len(negative_results),
            baseline_exact_count=sum(r['baseline_exact'] for r in results),
            candidate_exact_count=sum(r['candidate_exact'] for r in results),
            baseline_positive=baseline_positive,candidate_positive=candidate_positive,
            non_target_unchanged=sum(not r['eligible'] for r in results),
            applied_indices=[r['index'] for r in results if r['applied']],
            baseline_calls=sum(r['baseline_calls'] for r in results),
            extra_core_calls=sum(r['extra_core_calls'] for r in results),
            counterexamples=[{k:v for k,v in r.items() if k not in ('baseline','candidate')} for r in negative_results],
            source_changes=source_changes,imported_sources=imported,external_project_imports=external,
            follow_config=asdict(evaluator.follow.config),motion_config=asdict(evaluator.motion),
            reference_execution='r396 full_current_v3 actual revised-manifest run; initial freeze violation preserved',
            no_tolerance_or_field_removal=True,global_proof_replacement=False,
            historical795_repeated=False,original_r386_pipeline_passed=False,
            simulation_started=False,physical_device_opened=False,default_enabled=False,
            timing_scope='single stored x86 diagnostic; not a real-time or command-expiry result')
        (destination/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
        print(json.dumps({k:v for k,v in report.items() if k not in ('imported_sources','follow_config','motion_config','counterexamples')},ensure_ascii=False))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
