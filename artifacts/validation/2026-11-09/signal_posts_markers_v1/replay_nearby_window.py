"""근접 면의 .8초/9스캔 창을 저장 입력에서만 비교한다. 실행 코드 불변."""
import json, hashlib, types, zipfile
from dataclasses import replace
from contextlib import contextmanager
from pathlib import Path
from replay_tools import build_controller, read_events
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.nearby_face_motion import NearbyFragments
A=Path(__file__).resolve().parent; R=A.parents[3]
# 연결 뒤 재실행해도 baseline에 r57 창을 적용하지 않도록 r56 바이트를 읽는다.
saved=types.ModuleType('saved_r56_nearby')
with zipfile.ZipFile(A/'before_r57.zip') as z:
    exec(compile(z.read('src/arena_autonomy/arena_autonomy/nearby_face_motion.py'),
                 'r56_nearby','exec'),saved.__dict__)
original_init=saved.NearbyFragments.__init__
def longer_init(self,policy):
    original_init(self,replace(policy,window_s=.8,minimum_scans=9))
@contextmanager
def window(longer):
    restore=NearbyFragments.__init__
    NearbyFragments.__init__=longer_init if longer else original_init
    try: yield
    finally: NearbyFragments.__init__=restore
results=[]
for name, path in [('c2', A/'c_stationary_posts/run2/follow_decisions.jsonl'),
                   ('c1', A/'c_stationary_posts/run1/follow_decisions.jsonl'),
                   ('r18', R/'artifacts/validation/2026-11-08/rear_face_motion_connected_v1/persistent_block/run1/follow_decisions.jsonl')]:
    digest=hashlib.sha256(path.read_bytes()).hexdigest(); rows=[]
    for e in read_events(path):
        d=e['data']
        if e['kind']=='header':
            with window(False): baseline,mc=build_controller(d)
            with window(True): candidate,_=build_controller(d)
        elif e['kind']=='reset':
            with window(False): baseline.reset()
            with window(True): candidate.reset()
        elif e['kind']=='call':
            with window(False): before=freeze(replay_call(baseline,d['input'],mc))
            with window(True): after=freeze(replay_call(candidate,d['input'],mc))
            rows.append(dict(t=d['input']['reference_s'], before=before[:2], after=after[:2],
                same=equal_output(before[:2],after[:2]),
                baseline_matches=equal_output(comparable_decision(before),comparable_decision(d['output'])),
                reason=after[2].get('bypass',{}).get('reason')))
    (A/f'nearby_window_{name}_rows.json').write_text(json.dumps(rows,indent=1)+'\n')
    result=dict(name=name, calls=len(rows), command_changes=sum(not r['same'] for r in rows),
        baseline_mismatches=sum(not r['baseline_matches'] for r in rows),
        zero_before=sum(r['before'][0]<=1e-6 for r in rows),zero_after=sum(r['after'][0]<=1e-6 for r in rows),
        zero_after_14=sum(r['after'][0]<=1e-6 and r['t']>14 for r in rows),
        source_unchanged=hashlib.sha256(path.read_bytes()).hexdigest()==digest)
    results.append(result); print(json.dumps(result),flush=True)
(A/'nearby_window_summary.json').write_text(json.dumps(results,indent=2)+'\n')
