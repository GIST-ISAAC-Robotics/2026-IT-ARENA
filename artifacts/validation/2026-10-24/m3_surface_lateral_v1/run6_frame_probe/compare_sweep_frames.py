"""동일 센서/후보에서 제거한 자차 전진 때문에 달라지는 격자 질의를 비교한다.

현재 코드의 선택/판정은 수정하지 않는다. 전체 미래 자세의 관측 여부와 숨은 도달을 평가한다.
숨은 차량 속도는 현재 자차좌표에 고정된 월드에서 v_ego+상대 상계로 보수적으로 둔다.
이는 센서 잡음/누락/기구 실측을 검증한 안전 증거가 아니다.
"""
from dataclasses import replace
import json
from pathlib import Path
import sys
import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'src/arena_autonomy')]
import test_lateral_intent as fixture
import arena_autonomy.lateral_intent as li


def inspect(scene, path, speed, cfg):
    frame = li.PathFrame(path)
    ctx = li.build_context(scene, frame, speed, cfg)
    state, grid = ctx['state'], ctx['grid']
    out = li.LateralIntentLimiter(cfg).step(scene, path, speed)
    ex, ey = grid.centers()
    results = {}
    for key, check in out['checks'].items():
        candidate = li.offset_candidate(frame, state, check['d_target_m'], cfg, key)
        times = candidate['progress']/max(cfg.planned_speed_floor_mps, abs(speed))
        valid = times <= cfg.commit_horizon_s
        first = li.sweep_cells(grid, candidate['poses'][valid], times[valid], cfg,
                               li.steering_protrusion_m(candidate['max_abs_steering_rad'], cfg))
        query = np.isfinite(first) & (grid.state != li.EGO)
        cells = np.column_stack((ex[query], ey[query]))
        tau = first[query]+ctx['age_s']
        # free-space 도달의 하한: 자차를 피해갈 의무를 가정하지 않고 각 축 이동을 허용한다.
        seeds = ctx['hidden_seeds_xy']
        reached = np.zeros(len(cells), dtype=bool)
        for start in range(0, len(cells), 128):
            p = cells[start:start+128]
            if not len(seeds):
                continue
            dx = np.maximum(0., np.abs(p[:, None, 0]-seeds[None, :, 0])-cfg.hidden_vehicle_length_m/2)
            dy = np.maximum(0., np.abs(p[:, None, 1]-seeds[None, :, 1])-cfg.hidden_vehicle_width_m/2)
            minimum = np.min(np.maximum(dx/(abs(speed)+cfg.hidden_relative_speed_mps),
                                       dy/cfg.hidden_lateral_speed_mps), axis=1)
            reached[start:start+128] = minimum <= tau[start:start+128]
        results[key] = dict(original_mode=out['mode'], original_reasons=check['reasons'],
                            full_pose_cells=int(np.sum(query)),
                            full_pose_unobserved=int(np.sum(query & ~ctx['observed_free'])),
                            full_pose_hidden_reach=int(np.sum(reached)),
                            original_unobserved=check.get('new_cells_unobserved'),
                            original_hidden=check.get('hidden_reach_cells'))
    return results


data = {}
for speed in (.2, .35, .7, 1.):
    for name, d0, actor in [('empty_offset', -.09, None), ('left_actor', -.09, (-.4, .13)),
                            ('right_actor', .09, (-.4, -.13))]:
        extra = [] if actor is None else [fixture.car(*actor, d0)]
        scene, path = fixture.straight(d0, extra=extra)
        data[f'{name}@{speed}'] = inspect(scene, path, speed, fixture.GRASS)
destination = Path(sys.argv[1])
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text(json.dumps(data, indent=2), encoding='utf-8')
for key, result in data.items():
    print(key, json.dumps(result))
