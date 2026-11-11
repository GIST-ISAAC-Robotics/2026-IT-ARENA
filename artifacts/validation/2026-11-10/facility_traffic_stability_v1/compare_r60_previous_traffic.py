"""r60의 작은 분기 변경을 기존 추월·피추월 입력에서 직전 함수와 대조한다."""
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
import arena_autonomy.pass_release_trial as target


def main():
    output=A/'r60_previous_traffic';output.mkdir(exist_ok=False)
    snapshot=A/'before_r60.zip'
    with zipfile.ZipFile(snapshot) as z: source=z.read('pass_release_trial.py')
    # 같은 클래스의 _guard_curve만 바꾸고 전체 제어기 상태는 각각 독립한다.
    module=types.ModuleType('r60_previous_selector_eval')
    module.__file__=str(snapshot)+'!pass_release_trial.py'
    sys.modules[module.__name__]=module
    exec(compile(source,module.__file__,'exec'),module.__dict__)
    current=target.PassReleaseSelector._guard_curve
    before=module.PassReleaseSelector._guard_curve
    @contextmanager
    def version(function):
        target.PassReleaseSelector._guard_curve=function
        try:yield
        finally:target.PassReleaseSelector._guard_curve=current
    inputs=dict(
        medium_recessed=ROOT/'artifacts/validation/2026-11-08/medium_recessed_pass_v1/straight_moving_lead/run1/follow_decisions.jsonl',
        rear_overtake=ROOT/'artifacts/validation/2026-11-08/rear_overtake_measured_v1/left_rear_overtakes/run1/follow_decisions.jsonl')
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    sources={str(p.relative_to(ROOT)):digest(p) for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')}
    results={}
    for name,path in inputs.items():
        sha=digest(path);controllers={};counts=Counter();changes=[]
        with (output/f'{name}_changes.jsonl').open('x') as stream:
            for event in read_events(path):
                data=event['data']
                if event['kind']=='header':
                    for key in ('before','current'):controllers[key],motion=build_controller(data)
                elif event['kind']=='reset':
                    for ctl in controllers.values():ctl.reset()
                elif event['kind']=='call':
                    outputs={}
                    for key,fun in [('before',before),('current',current)]:
                        with version(fun):outputs[key]=freeze(replay_call(controllers[key],data['input'],motion))
                        counts[key+'_zeros']+=outputs[key][0]<=1e-6
                    counts['calls']+=1
                    if not equal_output(outputs['before'][:2],outputs['current'][:2]):
                        counts['command_changes']+=1
                        row=dict(t=data['input']['reference_s'],before=outputs['before'][:2],current=outputs['current'][:2],
                                 detail=outputs['current'][2].get('normal') or outputs['current'][2])
                        stream.write(json.dumps(row)+'\n')
                        if len(changes)<8:changes.append(row)
        results[name]=dict(counts=dict(counts),examples=changes,input_sha256=sha,input_unchanged=digest(path)==sha)
    result=dict(scope='same current stack, only r60 guard function versus archived predecessor; saved input not new motion',
                snapshot_sha256=digest(snapshot),runs=results,sources=sources,
                source_unchanged=all(digest(ROOT/p)==h for p,h in sources.items()))
    (output/'summary.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(dict(runs={k:v['counts'] for k,v in results.items()},source_unchanged=result['source_unchanged'])))


if __name__=='__main__':main()
