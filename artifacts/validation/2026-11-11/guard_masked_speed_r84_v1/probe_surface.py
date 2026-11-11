"""저장 표면 적합 실패: 가까운 끝점과 유효한 내부 현의 선택 차이만 분리."""
import hashlib
import json
import sys
from pathlib import Path
import numpy as np

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path.insert(0, str(ROOT/'src/arena_autonomy'))
from arena_autonomy.partial_surface_alignment import surface_translation

d = json.loads((A/'latch_sensor_fixture.json').read_bytes())['context']
old = np.array(d['previous_points_now_xy'])
cur = np.array(d['current_return_points_xy'])
ids = np.array(d['current_return_source_indices'])
source = ROOT/'src/arena_autonomy/arena_autonomy/partial_surface_alignment.py'
before = hashlib.sha256(source.read_bytes()).hexdigest()
code = source.read_text(encoding='utf-8')
original = 'nearest=np.argmin(distance,axis=1);rows=np.arange(len(points))'
replacement = ('distance=np.where((fraction>=-1e-9)&(fraction<=1+1e-9),distance,np.inf)\n'
               '        nearest=np.argmin(distance,axis=1);rows=np.arange(len(points))')
assert code.count(original) == 1
namespace = {}
exec(compile(code.replace(original, replacement), '<interior_segment_probe>', 'exec'), namespace)
out = dict(scope='saved frame fit only; runtime unmodified; no controller permission or driving',
    baseline=surface_translation(old, cur, ids),
    constrained_nearest=namespace['surface_translation'](old, cur, ids),
    source_sha256=before, source_unchanged=hashlib.sha256(source.read_bytes()).hexdigest() == before)
(A/'surface_fit_probe.json').write_text(json.dumps(out, indent=2)+'\n', encoding='utf-8')
print(json.dumps(out))
