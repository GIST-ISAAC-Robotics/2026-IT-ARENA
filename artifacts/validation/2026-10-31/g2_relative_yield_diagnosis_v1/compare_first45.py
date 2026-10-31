"""저장된 세 실행의 초록 이후45초 운동 차이. 동일궤적 인과 대조는 아니다."""
import hashlib
import json
from pathlib import Path

BASE=Path(__file__).resolve().parent
CASES=(('old_original_step','g2_motion_pause_gazebo_v1/run1'),
       ('old_level_surface','g2_motion_pause_level_v1/run1'),
       ('yield_level_surface','g2_relative_yield_gazebo_v1/run2'))


def sha(p):
    with p.open('rb') as stream:return hashlib.file_digest(stream,'sha256').hexdigest()


def summarize(label,relative):
    run=BASE.parent/relative
    report_path=run/'report.json';data=run/'braking_diagnostics.jsonl'
    hashes={p.name:sha(p) for p in (report_path,data)}
    report=json.loads(report_path.read_text(encoding='utf-8'))
    green=report['green_observed_sim_s'];end=green+45.
    previous=None;first=last=None;samples=0;wheel=body=large=largest=0.
    with data.open(encoding='utf-8') as stream:
        for line in stream:
            r=json.loads(line);t=r['sim_time_s']
            if not green<=t<=end:continue
            samples+=1;first=t if first is None else first;last=t
            error=abs(r['wheel_surface_speed_mps']-r['truth_longitudinal_mps'])
            largest=max(largest,error)
            if previous is not None:
                dt=t-previous['sim_time_s'];assert 0<dt<=.05
                wheel+=.5*dt*(r['wheel_surface_speed_mps']+previous['wheel_surface_speed_mps'])
                body+=.5*dt*(r['truth_longitudinal_mps']+previous['truth_longitudinal_mps'])
                if error>.2 and r['applied_target_mps']>.05:large+=dt
            previous=r
    assert first is not None and first-green<=.05 and end-last<=.05
    return dict(label=label,run=relative,green_s=green,first_s=first,last_s=last,samples=samples,
        wheel_integral_m=wheel,body_longitudinal_integral_m=body,
        commanded_large_disagreement_s=large,maximum_absolute_difference_mps=largest,
        raw_sha256=hashes,raw_unchanged=all(sha(p)==hashes[p.name] for p in (report_path,data)))


if __name__=='__main__':
    result=dict(scope='post-hoc equal45s windows; different trajectories/scan phases; not isolated causal proof',
                cases=[summarize(*case) for case in CASES],script_sha256=sha(Path(__file__)))
    with (BASE/'first45_comparison.json').open('x',encoding='utf-8') as stream:
        json.dump(result,stream,indent=2)
    print(json.dumps(result))
