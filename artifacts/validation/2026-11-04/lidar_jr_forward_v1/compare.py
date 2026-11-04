"""*_eval.json을 한 표로 모으고 단독 랩은 R 대조와 같은 진행 구간 시간을 비교한다. 평가 전용."""
import json
import math
from pathlib import Path
import sys

import numpy as np

HERE = Path(__file__).resolve().parent


def timing(run):
    rows = [json.loads(line) for line in (run/'trajectory.jsonl').read_text(encoding='utf-8').splitlines()]
    green = [r for r in rows if r['light'] == 'green']
    t = np.array([r['time'] for r in green])-green[0]['time']
    s = np.array([r['progress_m'] for r in green])
    keep = np.r_[True, np.diff(s) > 0]
    return t[keep], s[keep]


def main(control='r_lap1'):
    table = {}
    for path in sorted(HERE.glob('*_eval.json')):
        table[path.name[:-10]] = json.loads(path.read_bytes())
    report = dict(scope='각 1회 실행의 평가 요약. 같은 빌드·월드·속도 프로필·장면이며 센서 장착 YAML만 다르다. '
                        '단회 차이를 장착 효과로 단정하지 않는다.', runs={})
    for name, data in table.items():
        report['runs'][name] = {k: v for k, v in data.items() if k != 'actor_scan_rows'}
    if (HERE/control).exists():
        tc, sc = timing(HERE/control)
        edges = np.arange(0., 47., 4.)
        base = np.interp(edges, sc, tc)
        segments = {}
        for name, data in table.items():
            if name == control or 'lap_s' not in data or not (HERE/name/'trajectory.jsonl').exists():
                continue
            t, s = timing(HERE/name)
            if s[-1] < edges[-1]:
                continue
            now = np.interp(edges, s, t)
            segments[name] = [dict(s0=float(a), s1=float(b), control_s=round(float(base[i+1]-base[i]), 3),
                                   now_s=round(float(now[i+1]-now[i]), 3),
                                   delta_s=round(float((now[i+1]-now[i])-(base[i+1]-base[i])), 3))
                              for i, (a, b) in enumerate(zip(edges, edges[1:]))]
        report['same_progress_vs_control'] = dict(control=control, segments=segments)
    (HERE/'comparison.json').write_text(json.dumps(report, ensure_ascii=False, indent=1, default=str),
                                         encoding='utf-8', newline='\n')
    for name, data in report['runs'].items():
        print(name, {k: data.get(k) for k in ('passed', 'lap_s', 'peak_ground_speed_mps', 'peak_speed_mps', 'progress_m',
                                              'min_gap_m', 'contacts', 'first_move_after_green_s')})


if __name__ == '__main__':
    main(*sys.argv[1:2])
