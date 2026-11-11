"""직전 method와 수정 method를 같은 저장 입력에 대조한다. 폐루프 주행 아님."""
import ast
from collections import Counter
import hashlib
import json
from pathlib import Path
import zipfile

from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
import arena_autonomy.local_follow as module

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
PREVIOUS=A.parent/'follow_rebase_r126_v1/two_actor_run1'
SOURCE='src/arena_autonomy/arena_autonomy/local_follow.py'


def run(path,fn,label):
    digest=hashlib.sha256(path.read_bytes()).hexdigest()
    events=[json.loads(line) for line in path.open(encoding='utf8')]
    follow,motion=make_controller(events[0]['data']);outputs=[];prior=[]
    module.LocalFollow._baseline_command=fn
    try:
        for row in events[1:]:
            if row['kind']=='reset':follow.reset()
            elif row['kind']=='call':
                actual=freeze(replay_call(follow,row['data']['input'],motion))
                if not equal_output(comparable_decision(actual),comparable_decision(row['data']['output'])):
                    prior.append(row['seq'])
                outputs.append(dict(seq=row['seq'],input_ref_s=row['data']['input']['reference_s'],output=actual))
    finally:module.LocalFollow._baseline_command=CURRENT
    assert hashlib.sha256(path.read_bytes()).hexdigest()==digest
    with (A/(label+'.jsonl')).open('x') as stream:
        for row in outputs:stream.write(json.dumps(row)+'\n')
    return outputs,prior,digest


CURRENT=module.LocalFollow._baseline_command


def main():
    registration=json.loads((PREVIOUS/'registered.json').read_bytes())
    with zipfile.ZipFile(PREVIOUS/'extra_sources.zip') as archive:old=archive.read(SOURCE)
    digest=hashlib.sha256(old).hexdigest();assert digest==registration['extra_sources'][SOURCE]
    tree=ast.parse(old.decode())
    cls=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='LocalFollow')
    method=next(node for node in cls.body if isinstance(node,ast.FunctionDef) and node.name=='_baseline_command')
    namespace=vars(module).copy()
    exec(compile(ast.Module(body=[method],type_ignores=[]),'r126_frozen_baseline_method','exec'),namespace)
    baseline=namespace['_baseline_command']
    cases=dict(r126=PREVIOUS/'run1/follow_decisions.jsonl',
        r91=A.parent/'control_10hz_r91_v1/two_actor_run4/run1/follow_decisions.jsonl')
    result={}
    for name,path in cases.items():
        before,prior,sha=run(path,baseline,name+'_before')
        after,current_prior,_=run(path,CURRENT,name+'_after')
        changed=[];commands=[];zero_to_positive=[];positive_to_zero=[];confirmations=[];violations=[]
        for x,y in zip(before,after):
            assert x['seq']==y['seq']
            if not equal_output(comparable_decision(x['output']),comparable_decision(y['output'])):changed.append(x['seq'])
            if not equal_output(x['output'][:2],y['output'][:2]):commands.append(x['seq'])
            if x['output'][0]<=0<y['output'][0]:zero_to_positive.append(x['seq'])
            if x['output'][0]>0>=y['output'][0]:positive_to_zero.append(x['seq'])
            normal=y['output'][2].get('normal',y['output'][2])
            if normal.get('recent_boundary_confirmation'):
                confirmations.append(x['seq'])
                if (normal.get('boundary_from_memory') is not False
                        or not 0<=y['input_ref_s']-normal['path_source_s']<.3):violations.append(x['seq'])
        result[name]=dict(calls=len(before),trace_sha256=sha,baseline_prior_difference_sequences=prior,
            candidate_prior_difference_count=len(current_prior),whole_output_changes=changed,command_changes=commands,
            zero_to_positive=zero_to_positive,positive_to_zero=positive_to_zero,
            current_confirmation_sequences=confirmations,current_source_contract_violations=violations,
            reason_pairs=dict(Counter(str(((x['output'][2].get('normal') or {}).get('reason'),
                (y['output'][2].get('normal') or {}).get('reason'))) for x,y in zip(before,after)
                if not equal_output(x['output'][:2],y['output'][:2]))))
    result['implementation']=dict(before=digest,after=hashlib.sha256((ROOT/SOURCE).read_bytes()).hexdigest(),
        scope='only the baseline method swapped; other current source identical across variants')
    with (A/'comparison.json').open('x') as stream:json.dump(result,stream,indent=2)
    print(json.dumps({k:{key:(len(value) if isinstance(value,list) else value) for key,value in row.items()}
        for k,row in result.items()}))


if __name__=='__main__':main()
