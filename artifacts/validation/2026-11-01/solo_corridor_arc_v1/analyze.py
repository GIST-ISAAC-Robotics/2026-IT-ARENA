"""주행 뒤 기존 단독 기록과 같은 진행 위치의 시간/속도를 비교한다. 평가 전용."""
from hashlib import sha256
import importlib.util
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent


def summarize(folder):
    source = folder/'run1/trajectory.jsonl'
    raw = source.read_bytes()
    rows = [r for r in map(json.loads, raw.splitlines()) if r['light'] == 'green']
    t = np.array([r['time'] for r in rows]); s = np.array([r['progress_m'] for r in rows])
    v = np.array([math.hypot(r['dynamics']['truth_longitudinal_mps'],
                            r['dynamics']['truth_lateral_mps']) for r in rows])
    if len(t) < 2 or not np.all(np.diff(t) > 0):
        raise ValueError('incomplete or unordered trajectory')
    report = json.loads((folder/'run1/report.json').read_bytes())
    crossing = []
    for distance in [5., 10., 15., 19., 20., 25., 30., 35., 40., report['lap_length_m']]:
        hits = np.flatnonzero((s[:-1] < distance) & (s[1:] >= distance))
        if not len(hits):
            crossing.append(dict(distance_m=distance, elapsed_s=None, ground_mps=None))
            continue
        i = int(hits[0]); fraction = (distance-s[i])/(s[i+1]-s[i])
        crossing.append(dict(distance_m=distance, elapsed_s=float(t[i]+fraction*(t[i+1]-t[i])-t[0]),
                             ground_mps=float(v[i]+fraction*(v[i+1]-v[i]))))
    drops = [(float(v[i]-np.interp(stamp+.3, t, v)), float(stamp-t[0]), float(s[i]))
             for i, stamp in enumerate(t) if stamp+.3 <= t[-1]]
    return dict(case=folder.name, passed=report.get('passed'), source_sha256=sha256(raw).hexdigest(),
                crossings=crossing, maximum_0p3s_speed_drop_mps=sorted(drops, reverse=True)[:5],
                scope='녹색 궤적 표본의 보간. STOP 정리 구간이 포함될 수 있으며 실물 제동 실측이 아님.')


if __name__ == '__main__':
    results = [summarize(HERE.parent/name) for name in ('solo_route_v1', 'solo_headway_v1', HERE.name)]
    with (HERE/'lap_comparison.json').open('x', encoding='utf-8') as stream:
        json.dump(dict(cases=results, script_sha256=sha256(Path(__file__).read_bytes()).hexdigest()),
                  stream, ensure_ascii=False, indent=2)
    # 완료 랩만 종전 상한 분해를 그대로 적용한다. 실패 실행을 완주 창으로 만들지 않는다.
    metrics = json.loads((HERE/'run1_metrics.json').read_bytes())
    if metrics['lap_s_from_first_green_sample'] is not None:
        path = HERE.parent/'solo_route_v1/speed_budget.py'
        spec = importlib.util.spec_from_file_location('prior_solo_budget', path)
        module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
        module.HERE = HERE
        module.main()
    print(json.dumps([dict(case=r['case'], passed=r['passed'], crossings=r['crossings']) for r in results]))
