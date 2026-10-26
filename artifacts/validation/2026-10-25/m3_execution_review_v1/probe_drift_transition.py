"""합성 직선로에서 해제되지 않은 M3가 직진 편차 22 mm를 넘을 때의 전이(이전각 포함) 탐침. 개루프 다단계 진단.

벽만 있는 직선로(시험 fixture와 같은 광선 투사)를 자차 횡위치 d0·방향 오차 psi로 놓고, 이동 시에는 매 스캔 벽을
속도 v만큼 뒤로 옮겨 운동 이력과 일관되게 만든다. 측방 시험은 활성·만료 위협 기록(복귀 불가)으로 주입한다.
각 단계의 이전 조향은 직전 출력이다. 자세는 단계 사이에 바꾸지 않는다(출력대로 움직인 결과가 아님).
"""
import json
import math
from pathlib import Path
import sys
from dataclasses import replace
from types import SimpleNamespace

import numpy as np

import arena_autonomy.m3_execution_trial as mx
from test_lateral_intent import box, cast
from test_local_follow import history
from test_m3_execution_trial import ctl


def scene_at(now, d0, psi, shift):
    yc = -d0
    walls = [box(-3-shift, 6-shift, yc+.425, yc+.475), box(-3-shift, 6-shift, yc-.475, yc-.425),
             box(6-shift, 6.05-shift, yc-.5, yc+.5), box(-3.05-shift, -3-shift, yc-.5, yc+.5)]
    c, s = math.cos(-psi), math.sin(-psi)            # 도로를 -psi 회전: 자차 방향 오차 +psi
    rot = np.array([[c, -s], [s, c]])
    polys = [p@rot.T for p in walls]
    sc = cast(polys)
    delta = now-sc.t_ref_s
    return replace(sc, t_ref_s=now, source_first_s=sc.source_first_s+delta, source_last_s=sc.source_last_s+delta,
                   rays=tuple(replace(r, acquired_at_s=r.acquired_at_s+delta) for r in sc.rays))


def stale():
    return mx.ThreatRecord('left', SimpleNamespace(t_ref_s=0., source_epoch=-1, source_first_s=0.),
                           np.zeros((1, 2)), np.zeros(1), None, 0.)


def probe(d0, psi, v, small, steps=4):
    execution = mx.ExecutionConfig(command_fallback=True, release_to_baseline=True, small_center_straight=small)
    follow, h = ctl(execution), history(v=v)
    rows, previous = [], 0.
    for k in range(3):                                   # 기본 G1 경계 확인 준비(측방 시험 비활성)
        t = .4+.1*k
        follow.command(scene_at(t, d0, psi, v*(t-.4)), h, v, 'left')
    follow.lateral.active, follow.lateral.clear_fresh = True, 2
    follow.lateral.records = {'left': stale()}
    follow.held, follow.release_count = (v == 0.), 0
    for k in range(steps):
        t = .7+.1*k
        out = follow.command(scene_at(t, d0, psi, v*(t-.4)), h, v, 'left', previous_steering=previous)
        d = out[2]
        lat = d.get('m3_lateral') or {}
        checks = lat.get('checks') or {}
        rows.append({'t': round(t, 2), 'previous': previous, 'speed': out[0], 'steering': out[1], 'reason': d['reason'],
                     'mode': lat.get('mode'), 'small_ok': (lat.get('small_center') or {}).get('ok'),
                     'pred_mm': round(1000*(lat.get('small_center') or {}).get('max_abs_offset_m', float('nan')), 2),
                     'd0_mm': round(1000*(lat.get('state') or {}).get('d0', float('nan')), 2),
                     'plan': {n: c.get('reasons') for n, c in checks.items()},
                     'command': {n: (c.get('command') or {}).get('reason') for n, c in checks.items() if c.get('command')},
                     'gate': (lat.get('release_gate') or {}).get('reason')})
        previous = out[1]
    return rows


def main():
    output = Path(sys.argv[1])
    if output.exists():
        raise FileExistsError(output)
    cases = []
    for d0, psi in ((.0085, -.0056), (.014, -.0056), (.025, -.0056), (.04, 0.), (.025, 0.)):
        for v in (0., .34):
            for small in (False, True):
                cases.append({'d0_m': d0, 'psi_rad': psi, 'v_mps': v, 'small_center_straight': small,
                              'rows': probe(d0, psi, v, small)})
    output.write_text(json.dumps({'scope': 'synthetic straight wall road, injected unreleased M3; open-loop multi-step probe',
                                  'cases': cases}, ensure_ascii=False, indent=1)+'\n', encoding='utf8')
    for c in cases:
        print(c['d0_m'], c['psi_rad'], c['v_mps'], c['small_center_straight'])
        for r in c['rows']:
            print('   ', {k: r[k] for k in ('t', 'previous', 'speed', 'steering', 'reason', 'mode', 'small_ok', 'pred_mm', 'd0_mm')},
                  r['plan'], r['command'])


if __name__ == '__main__':
    main()
