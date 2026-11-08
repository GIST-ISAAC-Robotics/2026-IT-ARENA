"""합성 직각 벽에서 수정 전(before.zip)/후 local_path 출력을 비교한다. 제어기 실행 없음."""
import importlib.util
import json
import sys
import zipfile
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'src/arena_autonomy'))
from arena_autonomy.local_path import local_path, _path_ahead  # noqa: E402

source = zipfile.ZipFile(HERE/'before.zip').read('src/arena_autonomy/arena_autonomy/local_path.py')
spec = importlib.util.spec_from_loader('before_local_path', loader=None)
before = importlib.util.module_from_spec(spec)
exec(compile(source, 'before/local_path.py', 'exec'), before.__dict__)


def polyline(*v):
    return np.vstack([np.linspace(a, b, max(6, int(np.linalg.norm(np.array(b)-a)/.02)+1))[:-1]
                      for a, b in zip(v, v[1:])] + [np.array(v[-1:])])


def ahead(path):
    return None if path is None else _path_ahead(path)


CASES = {
    'outer_corner_long_return': [[-.4, -.505], [1.1, -.505], [1.1, 2.0]],
    'outer_corner_observed_end_short': [[-.4, -.505], [1.1, -.505], [1.1, .395]],
    'outer_corner_second_corner': [[-.4, -.505], [1.1, -.505], [1.1, .3], [.5, .3]],
    'outer_corner_far_corner': [[-.4, -.505], [1.51, -.505], [1.51, 2.0]],
    'straight': [[-.4, -.505], [2.5, -.505]],
}
rows = []
for name, vertices in CASES.items():
    for mirror in (1, -1):
        side = 'right' if mirror == 1 else 'left'
        cloud = np.vstack([polyline(*vertices), polyline([-.4, .505], [.1, .505])])*[1, mirror]
        for piecewise in (False, True):
            kwargs = dict(piecewise_corner=True, minimum_radius=.347) if piecewise else {}
            new, new_info = local_path(cloud.copy(), side, .505, **kwargs)
            old, old_info = before.local_path(cloud.copy(), side, .505, **kwargs)
            same = ((new is None and old is None) or (new is not None and old is not None and np.array_equal(new, old))) \
                and new_info == old_info
            rows.append(dict(case=name, mirror=mirror, piecewise=piecewise, identical_to_before=bool(same),
                             before_ahead=ahead(old), after_ahead=ahead(new),
                             before_wall_ahead=old_info.get('boundary_observed_ahead_m'),
                             after_wall_ahead=new_info.get('boundary_observed_ahead_m'),
                             budget=new_info.get('corner_path_budget'),
                             after_end=None if new is None else new[-1].tolist()))
            print(json.dumps(rows[-1]))
(HERE/'synthetic_check.json').write_text(json.dumps(rows, indent=1)+'\n', encoding='utf-8')
