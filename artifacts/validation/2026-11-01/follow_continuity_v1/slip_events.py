"""모터축 환산 속도와 고밀도 차체 속도가 크게 다른 구간을 사건으로 묶는다(사후 평가 전용).

차체 자세는 평가 기록이며 제어 입력이 아니다. 어떤 물리 원인인지는 이 표만으로 단정하지 않는다.
"""
import json
import sys
from pathlib import Path

import numpy as np


def body_speed(run, green):
    poses = np.array([json.loads(x) for x in (run / 'ego_poses.jsonl').open(encoding='utf-8')])
    t = poses[:, 0] - green
    grid = np.arange(max(0., t[0]), t[-1], .02)
    x = np.interp(grid, t, poses[:, 1])
    y = np.interp(grid, t, poses[:, 2])
    # ±30 ms 중앙 차분. 위치 중복 표본의 계단을 줄인다.
    n = 3
    d = np.hypot(x[2 * n:] - x[:-2 * n], y[2 * n:] - y[:-2 * n]) / (grid[2 * n:] - grid[:-2 * n])
    return grid[n:-n], d, x[n:-n], y[n:-n]


def events(run, threshold=.4, minimum=.1):
    report = json.loads((run / 'report.json').read_bytes())
    green = report['case_window']['green_sim_s']
    duration = report['case_window']['duration_s']
    trace = json.loads((run / 'actuation_trace.json').read_bytes())
    tt = np.array([x['now_us'] / 1e6 - green for x in trace])
    wheel = np.array([x['mean_wheel_mps'] or 0. for x in trace], dtype=float)
    target = np.array([x['target_speed_mps'] or 0. for x in trace], dtype=float)
    t, body, x, y = body_speed(run, green)
    # 20 ms 카운트 양자화를 줄이도록 100 ms 평균한 축 속도를 비교한다.
    keep = np.r_[True, np.diff(tt) > 0]
    tt, wheel, target = tt[keep], wheel[keep], target[keep]
    w = np.interp(t, tt, np.convolve(wheel, np.ones(5) / 5, mode='same'))
    cmd = np.interp(t, tt, target)
    gap = body - w
    active = (t <= duration) & (gap > threshold)
    out = []
    i = 0
    while i < len(t):
        if not active[i]:
            i += 1
            continue
        j = i
        while j + 1 < len(t) and active[j + 1]:
            j += 1
        if t[j] - t[i] >= minimum:
            out.append(dict(start=float(t[i]), end=float(t[j]), body0=float(body[i]), body1=float(body[j]),
                            wheel_mean=float(np.mean(w[i:j + 1])), command_mean=float(np.mean(cmd[i:j + 1])),
                            decel_mps2=float((body[i] - body[j]) / (t[j] - t[i])),
                            xy0=[float(x[i]), float(y[i])], xy1=[float(x[j]), float(y[j])]))
        i = j + 1
    return out


if __name__ == '__main__':
    result = {}
    for name in sys.argv[2:]:
        result[name] = events(Path(name))
        print(name)
        for e in result[name]:
            print(f"  {e['start']:6.2f}-{e['end']:6.2f} body {e['body0']:.2f}->{e['body1']:.2f} wheel~{e['wheel_mean']:.2f} cmd~{e['command_mean']:.2f}"
                  f" decel {e['decel_mps2']:.2f} xy ({e['xy0'][0]:.2f},{e['xy0'][1]:.2f})->({e['xy1'][0]:.2f},{e['xy1'][1]:.2f})")
    Path(sys.argv[1]).write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8')
