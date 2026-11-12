"""r221의 저장 현재 입력에서 두 원호 일괄 계산을 비교. ROS/장치/공통 수정 없음."""
from pathlib import Path
import hashlib, json, statistics, time
import numpy as np
from arena_autonomy.decision_process import make_controller
from arena_autonomy.follow_trace import restore_call, freeze, equal_output
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy import guard_observation_policy as policy, pass_release_trial as ps
from candidate import motion_queries as candidate

A=Path(__file__).resolve().parent; ROOT=A.parents[3]
samples=A.parent/'current_recheck_jetson_r221_v1/samples.jsonl'
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
sources={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src').rglob('*.py')}
input_sha=sha(samples)
rows=[json.loads(line) for line in samples.open()]
owner,motion=make_controller(rows[0]['header'])
original=ps.motion_queries
captured=[]; prepared=[]; outputs=[]; errors=[]
def capture(*args,**kwargs):
    answer=original(*args,**kwargs)
    captured.append((args,kwargs,answer))
    return answer
ps.motion_queries=capture
try:
    for row in rows[1:]:
        data,scan,history=restore_call(row['current'],motion)
        scene=motion_scene(scan,history,data['reference_s'],timing_verified=data['timing_verified'],
            epoch=data['epoch'],rear_half_angle_deg=data['rear_half_angle_deg'])
        args=(row['original']['output'],row['original']['input'],row['current'],scene,history,owner.config)
        prepared.append((row['seq'],args))
        outputs.append(freeze(policy.rebase(*args)))
finally:ps.motion_queries=original

query_errors=[]; max_delta=0.; timing={'baseline':[],'candidate':[]}
for i,(args,kwargs,expected) in enumerate(captured):
    got=candidate(*args,**kwargs)
    if expected[1].shape==got[1].shape and expected[1].size:
        max_delta=max(max_delta,float(np.max(np.abs(expected[1]-got[1]))))
    if expected[0]!=got[0] or not np.array_equal(expected[1],got[1]):query_errors.append(i)
for repeat in range(2):
    for label,fn in ([('baseline',original),('candidate',candidate)] if repeat==0 else
                     [('candidate',candidate),('baseline',original)]):
        ps.motion_queries=fn
        try:
            for (seq,args),expected in zip(prepared,outputs):
                start=time.perf_counter();answer=policy.rebase(*args);elapsed=1000*(time.perf_counter()-start)
                timing[label].append(elapsed)
                if not equal_output(freeze(answer),expected):errors.append(dict(seq=seq,variant=label,repeat=repeat))
        finally:ps.motion_queries=original
def stats(values):
    v=sorted(values)
    return dict(count=len(v),p50_ms=statistics.median(v),p95_ms=v[int(.95*len(v))],max_ms=max(v))
after={p.relative_to(ROOT).as_posix():sha(p) for p in (ROOT/'src').rglob('*.py')}
report=dict(cases=len(prepared),query_cases=len(captured),query_array_mismatches=query_errors,
    max_query_delta_m=max_delta,output_mismatches=errors,timing={k:stats(v) for k,v in timing.items()},
    source_unchanged=sources==after,input_unchanged=sha(samples)==input_sha,
    source_sha256=sources,input_sha256=input_sha,
    scope='preloaded rebase-only CPU; two alternating passes under concurrent simulation; no runtime adoption')
report['passed']=not query_errors and not errors and report['source_unchanged'] and report['input_unchanged']
with (A/'report.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in report.items() if k!='source_sha256'}),flush=True)
raise SystemExit(0 if report['passed'] else 1)
