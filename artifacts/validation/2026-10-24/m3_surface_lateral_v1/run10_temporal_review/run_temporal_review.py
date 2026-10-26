"""현재 좌표·동일 시각 비교의 제한 합성 행렬. 제어 후보를 선택하거나 연결하지 않는다."""
import json
from pathlib import Path
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'src/arena_autonomy')]
from test_lateral_intent import GRASS, straight, car
from arena_autonomy.lateral_intent import PathFrame, build_context, offset_candidate
from arena_autonomy.lateral_sweep_review import review_sweep

result = {}
for speed in (.2, .35, .7):
    for name, d0, actor, target in [('empty_offset', -.09, None, 0.),
                                   ('left_actor', -.09, (-.4, .13), -.122),
                                   ('right_actor', .09, (-.4, -.13), .122)]:
        extra = [] if actor is None else [car(*actor, d0)]
        scene, path = straight(d0, extra=extra)
        frame = PathFrame(path)
        ctx = build_context(scene, frame, speed, GRASS)
        hold = offset_candidate(frame, ctx['state'], d0, GRASS, 'hold')
        candidate = offset_candidate(frame, ctx['state'], target, GRASS, 'candidate')
        for horizon in (.1, .2, .5):
            key = f'{name}@{speed}/horizon={horizon}'
            result[key] = review_sweep(scene, ctx['grid'], candidate, hold, ctx['hidden_seeds_xy'], speed, GRASS, horizon_s=horizon)
            print(key, json.dumps(result[key]), flush=True)
Path(sys.argv[1]).write_text(json.dumps(result, indent=1), encoding='utf-8')
