"""r82 실제 고정 입력에서 단절 인접 지주 분리만 교체 비교. 새 주행이 아니다."""
import hashlib,json
from pathlib import Path
from arena_autonomy import follow_trace as ft
from arena_autonomy import local_path as lp

root=Path(__file__).resolve().parents[4]
out=Path(__file__).resolve().parent
trace=root/'artifacts/validation/2026-11-11/path_sampling_r82_v1/two_actor_run1/run1/follow_decisions.jsonl'
assert not (out/'saved_comparison.json').exists()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before=sha(trace);source=root/'src/arena_autonomy/arena_autonomy/local_path.py';source_sha=sha(source)
original=ft.replay_call;old=lp.narrow_occluders
data={}

def proposed(points,broken,**kwargs):
    # 밀집 광선의 G2 직선벽 재관측은 기존 동작으로 남긴다.
    return old(points,broken,**kwargs) if kwargs else lp.break_adjacent_occluders(points,broken)

for label in ('baseline','candidate'):
    rows=[];current=None
    if label=='candidate':lp.narrow_occluders=proposed
    def capture(*args,**kwargs):
        value=original(*args,**kwargs)
        n=value[2].get('normal',value[2])
        rows.append(dict(seq=current['seq'],time=current['data']['input']['reference_s'],
            command=ft.freeze(value[:2]),reason=n.get('reason'),baseline_reason=n.get('baseline_reason'),
            boundary_selection=n.get('boundary_selection')))
        return value
    def events():
        global current
        for line in trace.open(encoding='utf-8'):
            current=json.loads(line)
            yield current
    ft.replay_call=capture
    try:result=ft.replay_events(events())
    finally:ft.replay_call=original;lp.narrow_occluders=old
    path=root/f'build/r83_{label}_commands.json'
    assert not path.exists()
    path.write_text(json.dumps(rows,ensure_ascii=False,indent=1)+'\n',encoding='utf-8')
    data[label]=dict(calls=result['calls'],full_outputs_matched=result['matched'],rows=rows)
    print(json.dumps(dict(stage=label,calls=result['calls'],full_outputs_matched=result['matched'])),flush=True)

def summarize(lo,hi):
    result={}
    for label,values in data.items():
        rows=[r for r in values['rows'] if lo<=r['time']<hi]
        reasons={}
        for r in rows:reasons[str(r['baseline_reason'])]=reasons.get(str(r['baseline_reason']),0)+1
        result[label]=dict(calls=len(rows),zero=sum(abs(r['command'][0])<1e-10 for r in rows),baseline_reasons=reasons)
    return result

changes=[dict(seq=a['seq'],time=a['time'],before=a['command'],after=b['command'],
              before_reason=a['baseline_reason'],after_reason=b['baseline_reason'])
         for a,b in zip(data['baseline']['rows'],data['candidate']['rows'])
         if not ft.equal_output(a['command'],b['command'])]
report=dict(scope='same saved r82 inputs; candidate substituted only for default local narrow_occluder call; not closed loop',
    input_sha256=before,input_unchanged=sha(trace)==before,source_sha256=source_sha,source_unchanged=sha(source)==source_sha,
    baseline={k:v for k,v in data['baseline'].items() if k!='rows'},
    candidate={k:v for k,v in data['candidate'].items() if k!='rows'},
    windows={f'{lo}-{hi}':summarize(lo,hi) for lo,hi in [(0,20),(20,43.6),(43.6,48.3),(48.3,65),(65,100)]},
    changed_commands=len(changes),first_changes=changes[:15],
    zero_to_positive=sum(c['before'][0]<=1e-10 and c['after'][0]>1e-10 for c in changes),
    positive_to_zero=sum(c['before'][0]>1e-10 and c['after'][0]<=1e-10 for c in changes))
assert report['input_unchanged'] and report['source_unchanged']
(out/'saved_comparison.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(report,ensure_ascii=False))
