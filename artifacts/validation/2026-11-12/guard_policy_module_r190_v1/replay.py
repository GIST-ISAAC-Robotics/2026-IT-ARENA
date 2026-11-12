"""별도 프로세스의 원/공통 정책 전체 출력 재생. 명령 발행 없음."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output

A=Path(__file__).resolve().parent;ROOT=A.parents[3]


def load(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    return module


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--policy',choices=['original','prefix','module'],required=True)
    parser.add_argument('--trace',choices=['r182','r187'],required=True)
    parser.add_argument('--name',required=True)
    args=parser.parse_args()
    trace=A.parent/({'r182':'guard_approach_runtime_r182_v1','r187':'expiry_route_runtime_r187_v1'}[args.trace])/ 'two_actor_run1/run1/follow_decisions.jsonl'
    events=[json.loads(line) for line in trace.open()]
    if args.policy=='prefix':load('r190_prior',A.parent/'guard_approach_cap_r181_v1/candidate.py').install()
    route=None
    if args.trace=='r187':
        route=load('r190_reference',A.parent/'expiry_route_reference_r186_v1/candidate_v2.py');route.install()
    header=events[0]['data']
    if args.policy=='module':header['follow_config']['guard_observation_policy']='selected_r181'
    owner,motion=make_controller(header)
    calls=0;mismatches=[];digest=hashlib.sha256()
    output=ROOT/'build'/('r190_'+args.name+'.jsonl')
    with output.open('x',encoding='utf-8') as stream:
        for event in events[1:]:
            d=event['data']
            if event['kind']=='reset':
                if route is not None and d.get('reason')=='decision_source_expired':route.reset_reference(owner)
                else:owner.reset()
            if event['kind']!='call':continue
            result=freeze(replay_call(owner,d['input'],motion));calls+=1
            normalized=comparable_decision(result)
            if args.policy!='original' and not equal_output(normalized,comparable_decision(d['output'])):
                mismatches.append(event['seq'])
            line=json.dumps(dict(seq=event['seq'],output=normalized),sort_keys=True,separators=(',',':'))+'\n'
            stream.write(line);digest.update(line.encode())
    summary=dict(calls=calls,mismatches=mismatches,output_sha256=digest.hexdigest(),
                 policy=args.policy,trace=args.trace,trace_sha256=hashlib.sha256(trace.read_bytes()).hexdigest(),
                 scope='saved output equivalence only; no controller publication or new driving')
    with (A/(args.name+'.json')).open('x') as stream:json.dump(summary,stream,indent=2);stream.write('\n')
    print(json.dumps(summary));assert not mismatches


if __name__=='__main__':main()
