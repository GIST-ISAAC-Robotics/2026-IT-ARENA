"""시간별 같은 숨은 출발점을 유지하는 한계 비교. 위험 수용/주행 허가는 아니다.

추가 공간의 reach 여부만 세면 동일 숨은 차가 이미 기준 경로에도 닿는지 잃는다.
동일 seed·동일 시각에서 기준보다 먼저 새 후보에 닿을 수 있는 seed를 별도로 센다.
"""
import json
from pathlib import Path
import sys
import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'src/arena_autonomy')]
from test_lateral_intent import GRASS, straight, car
from arena_autonomy.lateral_intent import PathFrame, build_context, offset_candidate
from arena_autonomy.lateral_sweep_review import synchronized_sweep


def seed_contacts(grid, sweep, seeds, speed, age, cfg):
    x, y = grid.centers()
    hits = {key: np.full(len(seeds), np.inf) for key in ('candidate', 'reference')}
    # 동일 seed가 현재 차체와 충돌하는 경우는 초기 상태와 모순이므로 별도 제외한다.
    initial = ((abs(seeds[:, 0]) <= cfg.length_m/2+cfg.hidden_vehicle_length_m/2) &
               (abs(seeds[:, 1]) <= cfg.width_m/2+cfg.hidden_vehicle_width_m/2))
    for key in hits:
        for mask, t in zip(sweep[key+'_masks'], sweep['times']):
            cells = np.column_stack((x[mask], y[mask]))
            ids = np.flatnonzero(~initial & ~np.isfinite(hits[key]))
            for start in range(0, len(ids), 128):
                take = ids[start:start+128]
                q = seeds[take]
                dx = np.maximum(0., abs(q[:, None, 0]-cells[None, :, 0])-cfg.hidden_vehicle_length_m/2)
                dy = np.maximum(0., abs(q[:, None, 1]-cells[None, :, 1])-cfg.hidden_vehicle_width_m/2)
                arrival = np.min(np.maximum(dx/(speed+cfg.hidden_relative_speed_mps), dy/cfg.hidden_lateral_speed_mps), axis=1)
                hit = arrival <= t+age
                hits[key][take[hit]] = t
    a, b = hits['candidate'], hits['reference']
    return {'seeds': len(seeds), 'initial_inconsistent': int(initial.sum()),
            'candidate_contact': int(np.isfinite(a).sum()), 'reference_contact': int(np.isfinite(b).sum()),
            'candidate_only': int((np.isfinite(a) & ~np.isfinite(b)).sum()),
            'candidate_earlier': int((a < b-1e-9).sum()),
            'reference_earlier': int((b < a-1e-9).sum()),
            'scope': 'same seed identity but independent attainable trajectories; not an actual trajectory pair'}


result = {}
for speed in (.2, .35, .7):
    for name, d0, actor, target in [('center', -.09, None, 0.), ('yield', -.09, (-.4, .13), -.122)]:
        scene, path = straight(d0, extra=[] if actor is None else [car(*actor, d0)])
        frame = PathFrame(path)
        ctx = build_context(scene, frame, speed, GRASS)
        hold = offset_candidate(frame, ctx['state'], d0, GRASS, 'hold')
        other = offset_candidate(frame, ctx['state'], target, GRASS, 'candidate')
        sweep = synchronized_sweep(ctx['grid'], other, hold, speed, GRASS)
        key = f'{name}@{speed}'
        result[key] = seed_contacts(ctx['grid'], sweep, ctx['hidden_seeds_xy'], speed, ctx['age_s'], GRASS)
        print(key, json.dumps(result[key]), flush=True)
Path(sys.argv[1]).write_text(json.dumps(result, indent=1), encoding='utf-8')
