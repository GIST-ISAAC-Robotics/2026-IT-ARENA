"""release_to_baseline 해제 판단의 합성 반례/긍정 사례. 같은 입력을 수정 전·후 소스에 넣어 출력만 기록한다.

자차는 정지(운동 이력 v=0), 도로 횡위치 −0.09 m(공식 3번 슬롯), 왼쪽 옆차 0.20×0.15 m가 0.45 m/s로 앞서 나간다.
장면은 0.1 s 간격의 서로 다른 스캔이다. 정답 위치는 장면 생성에만 쓰며 제어 입력이 아니다.
"""
import hashlib
import inspect
import json
import math
from pathlib import Path
import sys

import numpy as np

import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import GRASS, car, box
from test_local_follow import history
from test_m3_execution_trial import padding
from test_m3_observed_follow import timed_scene

D0 = -.09
SENSOR = (.06, 0.)


def car_x(t):
    return .30+.45*(t-1.0)


def span_deg(poly, margin=1.5):
    a = [math.degrees(math.atan2(y-SENSOR[1], x-SENSOR[0])) for x, y in poly]
    return min(a)-margin, max(a)+margin


def dropout_for(poly, keep=0):
    lo, hi = span_deg(poly)
    if not keep:
        return lambda deg: (deg >= lo) & (deg <= hi)
    # 물체 방향 광선 중 가운데 keep개만 남긴다(부분 관측).
    step = 360/500

    def fn(deg):
        inside = (deg >= lo) & (deg <= hi)
        idx = np.flatnonzero(inside)
        mid = idx[len(idx)//2-keep//2: len(idx)//2-keep//2+keep]
        out = inside.copy()
        out[mid] = False
        return out
    return fn


def scenario(name):
    """(시각, extra 다각형 목록, dropout) 열."""
    steps = []
    for k in range(13):
        t = round(1.0+.1*k, 3)
        c = car(car_x(t), .13, D0)
        if name == 'clear_ahead' or k == 0:
            steps.append((t, [c], None))
        elif name == 'dropout_ahead':
            steps.append((t, [c], dropout_for(c)))
        elif name == 'occluded_by_board':
            board = box(-.6, 1.6, .100, .110)          # 비compact 긴 반환열, 자차 옆면 2.5 cm 밖
            steps.append((t, [c, board], None))
        elif name == 'partial_two_points':
            steps.append((t, [c], dropout_for(c, keep=2)))
    if name == 'lost_beside':
        steps = [(1.0, [car(0., .13, D0)], None)]
        c = car(0., .13, D0)
        steps += [(round(1.0+.1*k, 3), [c], dropout_for(c)) for k in range(1, 13)]
    return steps


def run(name):
    trial = mx.ExecutionLateralTrial(GRASS, mx.ExecutionConfig(release_to_baseline=True))
    takes_history = 'history' in inspect.signature(trial.step).parameters
    h = history(v=0.)
    rows = []
    for t, extra, dropout in scenario(name):
        scene, path = timed_scene(t, D0, extra=extra, dropout=dropout)
        kwargs = {'history': h} if takes_history else {}
        out = trial.step(scene, [], path, .35, padding(scene), **kwargs)
        present = [k for k, v in out.get('threats', {}).items() if v.get('present')]
        gate = out.get('release_gate')
        rows.append({'t': t, 'true_car_rear_ds_m': round(car_x(t)-.10, 3) if name != 'lost_beside' else -.10,
                     'present': present, 'mode': out['mode'], 'released': bool(out.get('released_to_baseline')),
                     'active_after': trial.active,
                     'gate': None if gate is None else {k: gate.get(k) for k in ('ok', 'reason', 'blocked_cells',
                                                                                    'possible_cells', 'corridor_cells')}})
    first = next((r['t'] for r in rows if r['released']), None)
    return {'rows': rows, 'first_release_s': first}


def main():
    out = Path(sys.argv[1])
    if out.exists():
        raise ValueError('new output required')
    src = Path(mx.__file__).read_bytes()
    result = {'scope': 'synthetic step-level release cases; not driving', 'source': mx.__file__,
              'source_sha256': hashlib.sha256(src).hexdigest(),
              'cases': {n: run(n) for n in ('clear_ahead', 'dropout_ahead', 'occluded_by_board',
                                            'partial_two_points', 'lost_beside')}}
    out.write_text(json.dumps(result, ensure_ascii=False, indent=1)+'\n', encoding='utf-8', newline='\n')
    print(json.dumps({k: v['first_release_s'] for k, v in result['cases'].items()}))


if __name__ == '__main__':
    main()
