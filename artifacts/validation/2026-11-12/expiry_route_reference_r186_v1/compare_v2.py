"""원 결과 재현 후 source 만료 reset의 최근 관측 참조만 보존하는 저장 대조.

제어 연결 없음. 경로/명령 허가·위협 상태는 원 reset 그대로 버린다.
"""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import candidate_v2
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output

A=Path(__file__).resolve().parent
TRACE=A.parent/'guard_approach_runtime_r182_v1/two_actor_run1/run1/follow_decisions.jsonl'


def retain_reference_reset(owner):
    local=owner.normal
    reference=local.recent_path
    if local.path is not None and local.path_source is not None:
        if reference is None or local.path_source>reference[2]:
            reference=(local.path,local.path_ref,local.path_source)
    copied=copy.deepcopy(reference)
    metadata={name:getattr(local,name) for name in ('epoch','side','last_ref','last_source')}
    owner.reset()
    if copied is not None:
        local.recent_path=copied
        for name,value in metadata.items():setattr(local,name,value)
    assert local.path is None and local.held and local.path_wall is None
    return copied is not None


def main():
    path=A.parent/'guard_approach_cap_r181_v1/candidate.py'
    spec=importlib.util.spec_from_file_location('r185_recorded_policy',path)
    policy=importlib.util.module_from_spec(spec);spec.loader.exec_module(policy);policy.install()
    events=[json.loads(s) for s in TRACE.open()]
    originals={};mismatches=[];rows=[];resets=[];changed=0
    for name in ('baseline','reference_only'):
        if name=='reference_only':
            source=candidate_v2.install()
            (A/'modified_baseline_command_v2.py').write_text(source,encoding='utf8')
        owner,motion=make_controller(events[0]['data'])
        for event in events[1:]:
            if event['kind']=='reset':
                reason=event['data'].get('reason')
                if name=='reference_only' and reason=='decision_source_expired':
                    resets.append(dict(seq=event['seq'],reason=reason,retained=candidate_v2.reset_reference(owner)))
                else:owner.reset()
            if event['kind']!='call':continue
            d=event['data'];out=freeze(replay_call(owner,d['input'],motion))
            if name=='baseline':
                originals[event['seq']]=out
                if not equal_output(comparable_decision(out),comparable_decision(d['output'])):
                    mismatches.append(event['seq'])
            else:
                old=originals[event['seq']]
                differs=not equal_output(comparable_decision(out),comparable_decision(old))
                changed+=differs
                if differs or d['input']['reference_s']>=58:
                    def brief(value):
                        n=value[2].get('normal',{})
                        return dict(speed=value[0],steering=value[1],mode=n.get('selector_mode'),
                            path_wall=n.get('path_wall'),selection=n.get('boundary_selection'),
                            reason=n.get('reason'),recent=n.get('recent_boundary_confirmation'),
                            source=n.get('path_source_s'),memory=n.get('boundary_from_memory'))
                    rows.append(dict(seq=event['seq'],reference_s=d['input']['reference_s'],
                                     before=brief(old),after=brief(out)))
    result=dict(calls=len(originals),baseline_mismatches=mismatches,changed=changed,
                resets=resets,rows=rows,trace_sha256=hashlib.sha256(TRACE.read_bytes()).hexdigest(),
                scope='recorded-input counterfactual only; no changed trajectory, no runtime connection')
    with (A/'reset_comparison_v2.json').open('x') as out:json.dump(result,out,indent=2);out.write('\n')
    print(json.dumps({k:v for k,v in result.items() if k!='rows'}))
    assert not mismatches


if __name__=='__main__':main()
