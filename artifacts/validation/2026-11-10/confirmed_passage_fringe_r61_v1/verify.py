"""r61 연결을 보존 r60과 독립 상태로 대조한다. 입력은 같은 파일이며 출력은 구분한다."""
from collections import Counter
from contextlib import contextmanager
import hashlib
import json
from pathlib import Path
import sys
import types
import zipfile

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path.insert(0,str(ROOT/'artifacts/validation/2026-11-09/signal_posts_markers_v1'))
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output
import arena_autonomy.pass_release_trial as ps


def main():
    out=A/'replay';out.mkdir(exist_ok=False)
    with zipfile.ZipFile(A/'before.zip') as z:source=z.read('pass_release_trial.py')
    old=types.ModuleType('preserved_r60_selector')
    old.__file__=str(A/'before.zip')+'!pass_release_trial.py'
    sys.modules[old.__name__]=old
    exec(compile(source,old.__file__,'exec'),old.__dict__)
    current=ps.PassReleaseSelector._select
    @contextmanager
    def version(method):
        ps.PassReleaseSelector._select=method
        try:yield
        finally:ps.PassReleaseSelector._select=current
    paths=dict(corner=A.parent/'corner_parallel_continuity_v1/run1/follow_decisions.jsonl',
        recessed=A.parent/'facility_traffic_stability_v1/stationary_recessed_r60/run1/follow_decisions.jsonl',
        medium_recessed=ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1/follow_decisions.jsonl')
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    results={}
    for name,path in paths.items():
        input_sha=sha(path);counts=Counter();rows=[];controllers={}
        with (out/f'{name}_changes.jsonl').open('x') as stream:
            for event in read_events(path):
                data=event['data']
                if event['kind']=='header':
                    for key in ('before','current'):controllers[key],motion=build_controller(data)
                elif event['kind']=='reset':
                    for c in controllers.values():c.reset()
                elif event['kind']=='call':
                    outputs={};counts['calls']+=1
                    for key,method in [('before',old.PassReleaseSelector._select),('current',current)]:
                        with version(method):outputs[key]=freeze(replay_call(controllers[key],data['input'],motion))
                        counts[key+'_zeros']+=outputs[key][0]<=1e-6
                    if not equal_output(outputs['before'][:2],outputs['current'][:2]):
                        counts['command_changes']+=1
                        detail=outputs['current'][2].get('normal') or outputs['current'][2]
                        row=dict(t=data['input']['reference_s'],before=outputs['before'][:2],after=outputs['current'][:2],
                            reason=detail.get('selector_reason'),recheck=detail.get('selector_confirmed_fringe_recheck'))
                        stream.write(json.dumps(row)+'\n');rows.append(row)
        results[name]=dict(counts=dict(counts),examples=rows[:10],input_sha256=input_sha,input_unchanged=sha(path)==input_sha)
    result=dict(scope='same saved inputs, independent states, only selector method changes; not new motion',
        runs=results,sources=sources,source_unchanged=all(sha(ROOT/p)==h for p,h in sources.items()))
    (out/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(runs={k:v['counts'] for k,v in results.items()},source_unchanged=result['source_unchanged'])))
    assert result['source_unchanged'] and all(v['input_unchanged'] for v in results.values())


if __name__=='__main__':main()
