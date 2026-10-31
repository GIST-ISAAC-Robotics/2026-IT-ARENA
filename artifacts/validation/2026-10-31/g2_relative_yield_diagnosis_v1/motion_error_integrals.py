"""부호 있는 적분 차이의 상쇄와 구분한 속도 차이 절대 적분. 평가 전용."""
import json
from pathlib import Path
from compare_first45 import CASES,sha

BASE=Path(__file__).resolve().parent
results=[]
for label,relative in CASES:
    run=BASE.parent/relative
    paths=[run/'report.json',run/'braking_diagnostics.jsonl']
    hashes={p.name:sha(p) for p in paths}
    green=json.loads(paths[0].read_text(encoding='utf-8'))['green_observed_sim_s']
    previous=None;positive=negative=0.
    with paths[1].open(encoding='utf-8') as stream:
        for line in stream:
            r=json.loads(line);t=r['sim_time_s']
            if not green<=t<=green+45.:continue
            error=r['wheel_surface_speed_mps']-r['truth_longitudinal_mps']
            if previous:
                dt=t-previous[0];assert 0<dt<=.05
                positive+=.5*dt*(max(0.,error)+max(0.,previous[1]))
                negative+=.5*dt*(max(0.,-error)+max(0.,-previous[1]))
            previous=(t,error)
    results.append(dict(label=label,positive_difference_m=positive,negative_difference_m=negative,
        absolute_difference_integral_m=positive+negative,signed_difference_m=positive-negative,
        raw_sha256=hashes,raw_unchanged=all(sha(p)==hashes[p.name] for p in paths)))
result=dict(scope='45s wheel/body speed difference integrals; not localization accuracy or wheel contact forces',
            cases=results,script_sha256=sha(Path(__file__)))
with (BASE/'absolute_motion_difference.json').open('x',encoding='utf-8') as stream:
    json.dump(result,stream,indent=2)
print(json.dumps(result))
