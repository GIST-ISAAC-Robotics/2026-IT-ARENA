"""기준 주행과 비교 주행의 같은 정의 지표(평가 전용, 정답 자세는 지표 계산에만 사용).

python -I -B compare.py <run 디렉터리> [<run 디렉터리> ...]  → 표준 출력 JSON. --output 파일을 주면 저장.
정의(사전 등록과 같음):
- 창: 검사기 case_window [녹색, 녹색+14 s]. '첫 3 s 이후' = [녹색+3, 창 끝].
- 자차 속도: ego_poses 보간, 0.05 s 격자에서 ±0.05 s 중앙 차분.
- 0.10 m/s 미만: 첫 3 s 이후 표본 수×0.05 s(시간)과 진입 횟수(연속 구간 수).
- 명령 톱니: follow_decisions 창 안 호출의 최종 속도 명령 중 0.10 m/s 미만 개수, 최소·중앙값.
- 판정: report.json의 11개 관문·run_valid·harness_valid·source_unchanged·STOP 검사, 노면은 surface json.
"""
import bisect
import json
import math
import sys
from pathlib import Path


def jl(path):
    with path.open(encoding='utf-8') as f:
        return [json.loads(l) for l in f if l.strip()]


def at(rows, ts, t):
    i = min(max(bisect.bisect_left(ts, t), 1), len(rows)-1)
    a, b = rows[i-1], rows[i]
    w = (t-a[0])/(b[0]-a[0]) if b[0] > a[0] else 0.
    return a[1]+w*(b[1]-a[1]), a[2]+w*(b[2]-a[2])


def metrics(run, surface=None):
    report = json.loads((run/'report.json').read_text(encoding='utf-8'))
    tg, te = report['case_window']['green_sim_s'], report['case_window']['end_sim_s']
    ego = jl(run/'ego_poses.jsonl'); ts = [r[0] for r in ego]
    speeds, t = [], tg+3.
    while t <= te+1e-9:
        a, b = at(ego, ts, t-.05), at(ego, ts, t+.05)
        speeds.append((round(t, 3), math.hypot(b[0]-a[0], b[1]-a[1])/.1))
        t += .05
    low = [v < .10 for _, v in speeds]
    episodes = sum(1 for i, x in enumerate(low) if x and (i == 0 or not low[i-1]))
    calls = [r['data'] for r in jl(run/'follow_decisions.jsonl') if r['kind'] == 'call']
    win = [c for c in calls if tg <= c['input']['reference_s'] <= te]
    after3 = [c for c in win if c['input']['reference_s'] >= tg+3.]
    cmd = sorted(c['output'][0] for c in after3)
    reasons = {}
    for c in win:
        k = c['output'][2].get('follow_entry_space', {}).get('reason')
        reasons[k] = reasons.get(k, 0)+1
    ev = report['evaluation']
    surf = None
    if surface and Path(surface).is_file():
        s = json.loads(Path(surface).read_text(encoding='utf-8'))
        surf = dict(body_outside_samples=s['body_outside_samples_over_1e-6_m2'], wall_wheel_samples=s['wall_wheel_samples'],
                    max_off_wheels=s['interpretations']['any']['max_off_wheels'])
    vs = [v for _, v in speeds]
    return dict(run=run.as_posix(), passed=report['passed'], run_valid=report['run_valid'],
                harness_valid=report['harness_valid'], source_unchanged=report['source_unchanged'],
                gates=report['gates'], stop_checks=report['stop_checks'],
                min_sampled_gap_m=ev['gap']['min_sampled_gap_m'], min_swept_lower_bound_m=ev['gap']['min_swept_lower_bound_m'],
                contact_messages=ev['gap']['actor_contact_messages_with_ego'],
                final_gap_m=ev['progress']['sampled_gap_at_window_end_m'], progress_m=ev['progress']['at_window_end_m'],
                longest_stationary_s=ev['case_metrics']['longest_stationary_while_lead_moves_s'],
                after3=dict(samples=len(speeds), below_0p10_time_s=round(sum(low)*.05, 3), below_0p10_episodes=episodes,
                            min_speed_mps=min(vs), median_speed_mps=sorted(vs)[len(vs)//2], max_speed_mps=max(vs)),
                command_after3=dict(calls=len(cmd), below_0p10=sum(v < .10 for v in cmd), min=cmd[0], median=cmd[len(cmd)//2]),
                follow_entry_space_reasons_in_window=reasons, surface=surf)


def main(argv):
    out = None
    if '--output' in argv:
        i = argv.index('--output'); out = Path(argv[i+1]); argv = argv[:i]+argv[i+2:]
    result = [metrics(Path(a), Path(a).parent/(Path(a).name+'_surface.json')) for a in argv]
    text = json.dumps(result, indent=1, ensure_ascii=False)+'\n'
    if out:
        out.write_text(text, encoding='utf-8', newline='\n')
    print(text)


if __name__ == '__main__':
    main(sys.argv[1:])
