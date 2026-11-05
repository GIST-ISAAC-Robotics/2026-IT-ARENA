"""r4 구조 경로 증속 후보와 v4 1.2 m/s 문턱의 결합 재현(합성 입력, 런타임 무변경).

기존 회귀 tests/test_structure_route_speed.py의 합성 직선 통로(C 지주 구조 각도)를 그대로 쓴다.
같은 장면에서 일반 결측 1광선을 넣고, 열린 경로 증속 후보(open_route)가 있을 때와 없을 때
(open_route_speed_mps 미설정 = r4 이전처럼 기본 속도 후보만)의 최종 명령을 비교한다.
출력: repro_candidate_gate.json
"""
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT / 'tests'), str(ROOT / 'src/arena_autonomy'), str(ROOT / 'src/arena_vehicle_interface'), str(ROOT / 'scripts')]

import numpy as np  # noqa: E402
from arena_autonomy.local_follow import FollowConfig, LocalFollow  # noqa: E402
from arena_autonomy.local_path import mask_scan  # noqa: E402
from arena_autonomy.scene_motion import motion_scene  # noqa: E402
from test_local_follow import scan, history  # noqa: E402
from test_boundary_bridge import CONFIG  # noqa: E402
from test_structure_route_speed import sectors  # noqa: E402


def run(index, cruise, s=.05):
    cfg = dict(CONFIG)
    if not cruise:
        cfg.pop('open_route_speed_mps'); cfg.pop('blend_open_speed_by_length')
    ctl = LocalFollow(FollowConfig(**cfg, boundary_bridge=True, partial_observation_policy='bounded_local_v4',
                                   known_structure_policy='structure_aware_v1',
                                   known_structure_sectors_json=json.dumps(sectors(s))))
    h = history(); h.config = replace(h.config, lidar_x_m=s)
    angles = -math.pi + np.arange(500) * math.tau / 500
    out = None
    for first in [.3, .4, .5, .6, .7, .8]:
        raw = scan(first, np.minimum(8. / np.maximum(abs(np.cos(angles)), 1e-8),
                                     .425 / np.maximum(abs(np.sin(angles)), 1e-8)).tolist())
        raw = mask_scan(raw, {'rear_half_angle_deg': 0., 'sectors_deg': sectors(s)})
        if first == .8 and index is not None:
            raw.ranges[index] = float('nan')
        local = motion_scene(raw, h, first + .1, timing_verified=True, rear_half_angle_deg=0.)
        out = ctl.command(local, h, 1., 'left', raw_scan=raw)
    v, steer, meta = out
    po = meta.get('partial_observation') or {}
    return dict(missing_index=index, missing_deg=None if index is None else round(math.degrees(angles[index]), 2),
                cruise_candidate=cruise, speed_mps=v, reason=meta.get('reason'),
                partial_reason=po.get('reason'), partial_cap_mps=po.get('speed_cap_mps'),
                open_route=meta.get('open_route_speed_enabled'))


rows = [run(i, c) for i in (None, 250, 361) for c in (True, False)]
# 인자 없으면 기존 이름(수정 전 재현). 수정 후 재실행은 이름을 주어 원래 결과를 덮지 않는다.
NAME = sys.argv[1] if len(sys.argv) > 1 else 'repro_candidate_gate'
(HERE / f'{NAME}.json').write_text(json.dumps(rows, indent=1) + '\n', encoding='utf-8')
for r in rows:
    print(r)
