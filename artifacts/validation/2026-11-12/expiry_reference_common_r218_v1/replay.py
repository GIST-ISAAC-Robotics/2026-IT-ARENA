"""원 prefix와 공통 선택의 전체 비교, 기본 OFF 보존. WSL 고정 입력용."""
from pathlib import Path
import argparse
import copy
import gzip
import hashlib
import importlib.util
import json
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
TRACES={
    'r212':A.parent/'static_side_runtime_r212_v1/two_actor_run1/run1/follow_decisions.jsonl',
    'r217':A.parent/'expiry_null_runtime_r217_v1/two_actor_run1/run1/follow_decisions.jsonl'}
STORE=ROOT/'build/r218_replay';STORE.mkdir(exist_ok=True)


def run(mode):
    selected=mode!='baseline-off'
    if mode=='baseline-on':
        path=A.parent/'expiry_candidate_null_r216_v1/candidate.py'
        spec=importlib.util.spec_from_file_location('original_expiry',path)
        route=importlib.util.module_from_spec(spec);spec.loader.exec_module(route);route.install()
        reset_expiry=route.reset_reference
    elif mode=='verify':
        from arena_autonomy.expiry_reference import reset_after_source_expiry
        reset_expiry=reset_after_source_expiry
    reports=[]
    cases=[('on',tag) for tag in TRACES] if selected else [('off','r217')]
    if mode=='verify':cases.append(('off','r217'))
    for policy,tag in cases:
        events=[json.loads(line) for line in TRACES[tag].open()]
        header=copy.deepcopy(events[0]['data'])
        if mode=='verify':header['follow_config']['expiry_route_reference_trial']=policy=='on'
        owner,motion=make_controller(header);outputs=[];recorded_errors=[]
        for event in events[1:]:
            data=event['data']
            if event['kind']=='reset':
                if policy=='on' and data.get('reason')=='decision_source_expired':reset_expiry(owner)
                else:owner.reset()
            if event['kind']!='call':continue
            result=comparable_decision(freeze(replay_call(owner,data['input'],motion)))
            outputs.append([event['seq'],result])
            if tag=='r217' and policy=='on' and not equal_output(result,comparable_decision(data['output'])):
                recorded_errors.append(event['seq'])
        cache=STORE/f'{tag}_{policy}.json.gz'
        mismatches=[]
        if mode=='verify':
            with gzip.open(cache,'rt') as stream:expected=json.load(stream)
            assert len(expected)==len(outputs)
            mismatches=[seq for (seq,result),(old_seq,want) in zip(outputs,expected)
                        if seq!=old_seq or not equal_output(result,want)]
        else:
            with gzip.open(cache,'xt') as stream:json.dump(outputs,stream,separators=(',',':'))
        reports.append(dict(trace=tag,policy=policy,calls=len(outputs),mismatches=mismatches,
                            recorded_mismatches=recorded_errors,
                            trace_sha256=hashlib.sha256(TRACES[tag].read_bytes()).hexdigest(),
                            cache_sha256=hashlib.sha256(cache.read_bytes()).hexdigest()))
    with (A/f'{mode}.json').open('x') as stream:json.dump(reports,stream,indent=2);stream.write('\n')
    print(json.dumps(reports),flush=True)
    assert not any(r['mismatches'] or r['recorded_mismatches'] for r in reports)


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['baseline-off','baseline-on','verify'])
    run(parser.parse_args().mode)
