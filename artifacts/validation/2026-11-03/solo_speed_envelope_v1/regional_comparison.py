"""11/2 같은 월드 두 실제 궤적과 선정 입력의 저장 반사실을 분리한다. 평가 전용."""
from bisect import bisect_left, bisect_right
from collections import Counter
from csv import DictReader
from hashlib import sha256
from pathlib import Path
import json
import math

HERE = Path(__file__).resolve().parent
DAY = HERE.parent.parent / '2026-11-02'
BASE = DAY / 'solo_alternative_speed_v1'
RAY = DAY / 'solo_ray_combination_v1'


def trajectory(case):
    report = json.loads((case / 'run1/report.json').read_bytes())
    rows = json.loads((case / 'run1/trace.json').read_bytes())
    green = report['start_time_sim_s']
    return green, [r for r in rows if r['time'] >= green]


def first_passage(green, trace, mark):
    for a, b in zip(trace, trace[1:]):
        if a['progress_m'] < mark <= b['progress_m']:
            k = (mark-a['progress_m'])/(b['progress_m']-a['progress_m'])
            def value(f):
                return f(a) + k*(f(b)-f(a))
            return dict(elapsed_s=value(lambda r: r['time'])-green,
                        longitudinal_mps=value(lambda r: r['dynamics']['truth_longitudinal_mps']),
                        center_error_m=value(lambda r: r['centerline_error_m']),
                        bracket_s=b['time']-a['time'])
    return None


def main():
    paths = [BASE/'run1/trace.json', BASE/'run1/report.json', RAY/'run1/trace.json',
             RAY/'run1/report.json', RAY/'comparison.csv', RAY/'summary.json',
             RAY/'run1/follow_decisions.jsonl']
    digests = {p.relative_to(DAY).as_posix(): sha256(p.read_bytes()).hexdigest() for p in paths}
    one, two = trajectory(BASE), trajectory(RAY)
    sections = [0, 10, 20, 30, 40, 42, 43, 44, 49]
    with (RAY/'comparison.csv').open(encoding='utf-8') as f:
        compared = list(DictReader(f))
    assert json.loads((RAY/'summary.json').read_bytes())['baseline_mismatches'] == 0
    t = [r['time'] for r in one[1]]
    regions = []
    for lo, hi in zip(sections, sections[1:]):
        rows = [r for r in compared if lo <= one[1][max(0,bisect_right(t,float(r['t']))-1)]['progress_m'] < hi]
        assert rows
        ds = [float(r['candidate_ramp_1.5'])-float(r['base_ramp_1.5']) for r in rows]
        da = [abs(float(r['candidate_steer'])-float(r['base_steer'])) for r in rows]
        regions.append(dict(interval_m=[lo,hi], calls=len(rows), signed_ramped_command_integral_m=sum(ds)*.02,
                            max_ramped_command_difference_mps=max(map(abs,ds)),
                            max_absolute_steering_difference_rad=max(da),
                            nonzero_steering_differences=sum(v>1e-9 for v in da)))
    assert sum(r['calls'] for r in regions)==len(compared)
    latches = Counter()
    with (RAY/'run1/follow_decisions.jsonl').open(encoding='utf8') as f:
        for row in map(json.loads, f):
            if row['kind']!='call':
                continue
            d = row['data']['output'][2]
            part = d.get('partial_observation') or {}
            if part.get('reason')=='known_obstacle_latched':
                latches[(part['anchor_diagnostic']['reason'],part['clearance_diagnostic']['reason'])]+=1
    marks = [6, 14, 30, 40, 42, 42.5, 43, 43.2, 43.4, 44]
    result = dict(input_sha256=digests, compared_calls=len(compared), sample_period_s=.02,
                  fixed_input_regions=regions,
                  actual_first_passage={str(mark):dict(selected=first_passage(*one,mark),ray=first_passage(*two,mark)) for mark in marks},
                  failure_latched_calls=[dict(anchor=k[0],clearance=k[1],calls=v) for k,v in latches.most_common()],
                  scope='11/2 두 실제 주행의 최초 거리 통과 시각은 인접 평가 표본 선형 보간. 별도 표의 조향/속도 차이는 선정 입력 위에서만 비교한 기존 재생이다. 명령 적분은 실제 주행 거리가 아니다. 어느 표도 제어 입력이 아니다.')
    dest=HERE/'regional_comparison.json'
    with dest.open('x',encoding='utf-8') as f:
        json.dump(result,f,ensure_ascii=False,indent=2,allow_nan=False)
        f.write('\n')
    print(len(compared),sum(latches.values()),[(p['interval_m'],round(p['signed_ramped_command_integral_m'],3)) for p in regions])


if __name__=='__main__':
    main()
