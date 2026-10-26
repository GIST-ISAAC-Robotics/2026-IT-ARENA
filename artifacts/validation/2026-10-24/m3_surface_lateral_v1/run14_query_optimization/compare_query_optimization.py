"""run13 고정판과 새 묶음 계산의 질의·교차 개수/비용 대조. 센서·판정 변경 없음."""
import importlib.util
import json
from pathlib import Path
import sys
import time
import numpy as np

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'src/arena_autonomy')]
from test_lateral_intent import GRASS, straight, car
from arena_autonomy.lateral_intent import PathFrame, ego_state, offset_candidate
from arena_autonomy.observed_lateral_trial import path_observation_queries

original = Path(sys.argv[1])
spec = importlib.util.spec_from_file_location('old_observed_trial', original)
old = importlib.util.module_from_spec(spec)
spec.loader.exec_module(old)
results = []
for d0 in (-.09, 0., .09):
    for target in (-.12, 0., .12):
        for dropout in (None, lambda a: (a > 75) & (a < 120)):
            scene, path = straight(d0, extra=[car(-.4, .13, d0)], dropout=dropout)
            frame = PathFrame(path)
            candidate = offset_candidate(frame, ego_state(frame, GRASS), target, GRASS)
            poses = candidate['poses']
            before = time.perf_counter()
            a, ac = old.path_observation_queries(scene, poses, GRASS)
            mid = time.perf_counter()
            b, bc = path_observation_queries(scene, poses, GRASS)
            after = time.perf_counter()
            same = a.shape == b.shape and np.array_equal(a, b) and ac == bc
            results.append({'d0': d0, 'target': target, 'dropout': dropout is not None, 'equal': same,
                            'queries': len(a), 'crossings': ac, 'old_ms': (mid-before)*1e3, 'new_ms': (after-mid)*1e3})
            if not same:
                raise AssertionError(results[-1])
Path(sys.argv[2]).write_text(json.dumps(results, indent=1), encoding='utf-8')
print('identical_cases', len(results), 'old_ms_p50_p95_max', np.percentile([r['old_ms'] for r in results], [50,95,100]).tolist(),
      'new_ms_p50_p95_max', np.percentile([r['new_ms'] for r in results], [50,95,100]).tolist())
