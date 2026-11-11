"""통합 교통에서 기존 선택형 코너 관측 갱신 ON/OFF만 저장 입력으로 비교한다. 새 구현 없음."""
import copy
import hashlib
import json
import sys
from collections import Counter
from pathlib import Path

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output


def main():
    path=HERE/'two_actor_run1/run1/follow_decisions.jsonl'
    output=HERE/'boundary_refresh_probe';output.mkdir(exist_ok=False)
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    files=[path,*sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))]
    hashes={str(p.relative_to(ROOT)):sha(p) for p in files}
    controllers={};counts=Counter();examples=[];reasons=Counter()
    with (output/'rows.jsonl').open('w') as stream:
        for event in read_events(path):
            data=event['data']
            if event['kind']=='header':
                assert data['follow_config']['traffic_boundary_refresh_trial'] is False
                for name in ('r60_baseline','existing_corner_refresh'):
                    header=copy.deepcopy(data)
                    if name=='existing_corner_refresh':header['follow_config']['traffic_boundary_refresh_trial']=True
                    controllers[name],motion=build_controller(header)
            elif event['kind']=='reset':
                for ctl in controllers.values():ctl.reset()
            elif event['kind']=='call':
                row=dict(t=data['input']['reference_s'])
                for name,ctl in controllers.items():
                    out=freeze(replay_call(ctl,data['input'],motion));detail=out[2].get('normal') or out[2]
                    row[name]=dict(command=out[:2],reason=detail.get('reason'),
                        selection=detail.get('boundary_selection'),memory=detail.get('boundary_from_memory'),
                        path_source_s=detail.get('path_source_s'),selector=detail.get('selector_reason'),
                        candidate_ahead_m=detail.get('boundary_candidate_ahead_m'),
                        current_corner_support=detail.get('current_corner_support'),
                        current_corner_prefix_agreement=detail.get('current_corner_prefix_agreement'))
                    counts[name+'_zeros']+=out[0]<=1e-6
                    if 38.7<=row['t']<=39.7:counts[name+'_boundary_window_zeros']+=out[0]<=1e-6
                counts['calls']+=1
                if not equal_output(row['r60_baseline']['command'],row['existing_corner_refresh']['command']):
                    counts['changed_commands']+=1
                    k=(row['r60_baseline']['reason'],row['existing_corner_refresh']['reason'],row['existing_corner_refresh']['selection'])
                    reasons[k]+=1
                    if len(examples)<8 or (38.7<=row['t']<=39.7 and len(examples)<20):examples.append(row)
                stream.write(json.dumps(row)+'\n')
    result=dict(scope='saved input comparison; existing traffic_boundary_refresh_trial only; no closed-loop claim',
        counts=dict(counts),reason_changes=[dict(before=k[0],after=k[1],selection=k[2],count=v) for k,v in reasons.items()],
        examples=examples,hashes=hashes,source_unchanged=all(sha(ROOT/p)==h for p,h in hashes.items()))
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('examples','hashes')}),flush=True)


if __name__=='__main__':main()
