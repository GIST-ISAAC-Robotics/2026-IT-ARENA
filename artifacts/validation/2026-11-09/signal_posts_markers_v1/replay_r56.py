"""C 실패와 이전 기둥 없는 정지차 기록을 연결 전/후로 대조한다. 열린 고리 비교다."""
import json,hashlib,types,zipfile
from pathlib import Path
from replay_tools import build_controller,read_events
from arena_autonomy.follow_trace import replay_call,freeze,equal_output,comparable_decision
A=Path(__file__).resolve().parent;R=A.parents[3]
module=types.ModuleType('r55_forecast')
with zipfile.ZipFile(A/'before_r56.zip') as z:
    exec(compile(z.read('src/arena_autonomy/arena_autonomy/bypass_handoff_corridor_trial.py'),'r55_forecast','exec'),module.__dict__)
out=[]
for name,path in [('c_posts',A/'c_stationary_posts/run1/follow_decisions.jsonl'),
                  ('r18',R/'artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/run1/follow_decisions.jsonl')]:
    if not path.exists():raise FileNotFoundError(path)
    digest=hashlib.sha256(path.read_bytes()).hexdigest();rows=[]
    for e in read_events(path):
        d=e['data']
        if e['kind']=='header':
            baseline,mc=build_controller(d);candidate,_=build_controller(d)
            baseline.bypass.guide._padding=types.MethodType(module.ForecastGuide._padding,baseline.bypass.guide)
            baseline.bypass.guide.padding_provider=baseline.bypass.guide._padding
        elif e['kind']=='reset':baseline.reset();candidate.reset()
        elif e['kind']=='call':
            a=freeze(replay_call(baseline,d['input'],mc));b=freeze(replay_call(candidate,d['input'],mc))
            detail=b[2].get('bypass',{}).get('outer_boundary_padding',{}).get('nearby_face_motion',{})
            rows.append(dict(t=d['input']['reference_s'],before=a[:2],after=b[:2],
                same=equal_output(a[:2],b[:2]),baseline_matches=equal_output(comparable_decision(a),comparable_decision(d['output'])),
                changed_rays=detail.get('changed',0),behavior=b[2].get('behavior')))
    (A/f'r56_{name}_rows.json').write_text(json.dumps(rows,indent=1)+'\n')
    out.append(dict(name=name,calls=len(rows),baseline_mismatches=sum(not r['baseline_matches'] for r in rows),
        command_changes=sum(not r['same'] for r in rows),zero_before=sum(r['before'][0]<=1e-6 for r in rows),
        zero_after=sum(r['after'][0]<=1e-6 for r in rows),source_unchanged=hashlib.sha256(path.read_bytes()).hexdigest()==digest))
    print(json.dumps(out[-1]),flush=True)
(A/'r56_replay_summary.json').write_text(json.dumps(out,indent=2)+'\n')
