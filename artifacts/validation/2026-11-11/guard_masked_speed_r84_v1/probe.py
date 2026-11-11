"""r83 고정 입력: GUARD 감속 재검사 전후의 상태/명령 비교. 새 주행 아님."""
import collections,hashlib,json
from pathlib import Path
from arena_autonomy import follow_trace as ft
from arena_autonomy.pass_release_trial import PassReleaseSelector

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
trace=ROOT/'artifacts/validation/2026-11-11/wall_occluder_r83_v1/two_actor_run1/run1/follow_decisions.jsonl'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
source=ROOT/'src/arena_autonomy/arena_autonomy/pass_release_trial.py'
before,source_sha=sha(trace),sha(source)
wrapper=PassReleaseSelector._guard_curve;original=ft.replay_call;summary={};commands={}
assert not (A/'saved_comparison.json').exists()
for label in ('baseline','candidate'):
    PassReleaseSelector._guard_curve=PassReleaseSelector._guard_curve_at_cap if label=='baseline' else wrapper
    rows=[];current=None;reasons=collections.Counter();attempts=collections.Counter()
    def capture(*args,**kwargs):
        value=original(*args,**kwargs);n=value[2].get('normal',value[2]);r=n.get('selector_guard_speed_recheck') or {}
        rows.append(dict(seq=current['seq'],time=current['data']['input']['reference_s'],command=ft.freeze(value[:2]),
            reason=n.get('selector_reason'),recheck=r))
        reasons[n.get('selector_reason')]+=1
        if r:attempts['applied' if r['applied'] else 'rejected']+=1
        return value
    def events():
        global current
        for line in trace.open(encoding='utf8'):
            current=json.loads(line);yield current
    ft.replay_call=capture
    try:result=ft.replay_events(events())
    finally:ft.replay_call=original;PassReleaseSelector._guard_curve=wrapper
    summary[label]=dict(calls=result['calls'],full_matched=result['matched'],reasons=dict(reasons),rechecks=dict(attempts))
    commands[label]=rows
    out=ROOT/f'build/r84_{label}_commands.json';assert not out.exists()
    out.write_text(json.dumps(rows,ensure_ascii=False,indent=1)+'\n')
    print(json.dumps(dict(stage=label,**summary[label]),ensure_ascii=False),flush=True)
changes=[dict(seq=a['seq'],time=a['time'],before=a['command'],after=b['command'],reason=b['reason'])
         for a,b in zip(commands['baseline'],commands['candidate']) if not ft.equal_output(a['command'],b['command'])]
result=dict(scope='same frozen r83 inputs; old candidate path at cap=None exactly reproduces prior behavior; no new driving',
    input_sha256=before,input_unchanged=sha(trace)==before,source_sha256=source_sha,source_unchanged=sha(source)==source_sha,
    summary=summary,changed_commands=len(changes),zero_to_positive=sum(c['before'][0]<=1e-10<c['after'][0] for c in changes),
    positive_to_zero=sum(c['after'][0]<=1e-10<c['before'][0] for c in changes),first_changes=changes[:12])
assert result['input_unchanged'] and result['source_unchanged']
assert summary['baseline']['full_matched']==summary['baseline']['calls']
(A/'saved_comparison.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k not in ('summary','first_changes')},ensure_ascii=False))
