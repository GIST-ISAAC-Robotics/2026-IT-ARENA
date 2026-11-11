"""기존 개별 대표를 공통 교통 옵션으로 묶었을 때 저장 판단이 달라지는지 비교한다."""
import argparse
import copy
import hashlib
import json
from collections import Counter
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output


def combined(header):
    out=copy.deepcopy(header)
    out['follow_config'].update(narrow_occluder_bridge=True,junction_continuity=True,
        standing_wall_reacquire=True,traffic_boundary_refresh_trial=False)
    out['execution_config']['moving_pass_profile']='medium_v1'
    return out


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('case',choices=['recessed_stationary','medium_recessed'])
    args=parser.parse_args()
    path={
        'recessed_stationary':A/'stationary_recessed_r60/run1/follow_decisions.jsonl',
        'medium_recessed':ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1/follow_decisions.jsonl',
    }[args.case]
    out=A/f'combined_profile_{args.case}';out.mkdir(exist_ok=False)
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={str(p.relative_to(ROOT)):digest(p) for p in [path,*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]}
    counts=Counter();changes=[];controllers={};amendments={}
    with (out/'changes.jsonl').open('x') as stream:
        for event in read_events(path):
            data=event['data']
            if event['kind']=='header':
                proposal=combined(data)
                for section in ('follow_config','execution_config'):
                    amendments[section]={k:dict(before=data[section].get(k),after=v) for k,v in proposal[section].items()
                        if data[section].get(k)!=v}
                for key,h in [('recorded_settings',data),('combined_settings',proposal)]:
                    controllers[key],motion=build_controller(h)
            elif event['kind']=='reset':
                for c in controllers.values():c.reset()
            elif event['kind']=='call':
                counts['calls']+=1;outputs={}
                for key,c in controllers.items():
                    result=freeze(replay_call(c,data['input'],motion));outputs[key]=result
                    counts[key+'_zeros']+=result[0]<=1e-6
                if not equal_output(outputs['recorded_settings'][:2],data['output'][:2]):
                    counts['current_code_vs_recorded_command_changes']+=1
                old,new=outputs['recorded_settings'],outputs['combined_settings']
                if not equal_output(old[:2],new[:2]):
                    counts['profile_command_changes']+=1
                    a,b=old[2].get('normal') or old[2],new[2].get('normal') or new[2]
                    row=dict(t=data['input']['reference_s'],before=old[:2],after=new[:2],
                        old_reason=a.get('reason'),new_reason=b.get('reason'),
                        before_state=old[2].get('handoff_state'),after_state=new[2].get('handoff_state'))
                    stream.write(json.dumps(row)+'\n')
                    if len(changes)<12:changes.append(row)
    result=dict(scope='configuration compatibility on saved inputs; no closed-loop behavior proof',
        case=args.case,changes=amendments,counts=dict(counts),examples=changes,
        source_sha256=sources,source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('examples','source_sha256')}))


if __name__=='__main__':main()
