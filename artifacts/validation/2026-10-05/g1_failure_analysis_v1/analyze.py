"""보존 정지 스캔의 형상 진단. 운동 이력이 없어 정지 상태 근사로만 해석한다."""
import json
from pathlib import Path
import sys
import numpy as np

root = Path(__file__).resolve().parents[4]
sys.path[:0] = [str(root/'src/arena_autonomy'), str(root/'src/arena_vehicle_interface')]
from arena_autonomy.local_path import local_path
from arena_autonomy.local_follow import path_agrees

run_name = sys.argv[1] if len(sys.argv) > 1 else 'g1_gazebo_solo_v1'
if run_name not in ('g1_gazebo_solo_v1', 'g1_gazebo_solo_v2'):
    raise ValueError('unsupported saved run')
source = root/'artifacts/validation/2026-10-05'/run_name/'last_scan.json'
scan = json.loads(source.read_text())
r = np.asarray(scan['ranges'])
a = scan['angle_min']+np.arange(len(r))*scan['angle_increment']
valid = np.isfinite(r)&(r >= scan['range_min'])&(r <= scan['range_max'])&(np.abs(a) <= np.deg2rad(150))
points = np.column_stack((r[valid]*np.cos(a[valid])+.06, r[valid]*np.sin(a[valid])))
rows, paths = {}, {}
for side in ('left', 'right'):
    path, info = local_path(points, side)
    paths[side] = path
    rows[side] = {'info': {k:v for k,v in info.items() if k != 'path_points'},
        'path_x_extent': None if path is None else [float(path[:, 0].min()), float(path[:, 0].max())]}
    sign = 1 if side == 'left' else -1
    ordered = points[np.argsort(-sign*np.arctan2(points[:,1], points[:,0]))]
    gaps = np.linalg.norm(np.diff(ordered, axis=0), axis=1)
    limits = .10+.035*np.minimum(np.linalg.norm(ordered[:-1],axis=1),np.linalg.norm(ordered[1:],axis=1))
    rows[side]['segments'] = []
    for segment in np.split(ordered, np.flatnonzero(gaps > limits)+1):
        if len(segment) < 12: continue
        p, result = local_path(segment, side)
        rows[side]['segments'].append({'count':len(segment),'bbox':[segment.min(axis=0).tolist(), segment.max(axis=0).tolist()],
            'result':{k:v for k,v in result.items() if k != 'path_points'}})
rows['path_agrees'] = bool(all(p is not None for p in paths.values()) and path_agrees(paths['left'],paths['right'],.08))
report = json.loads((source.parent/'report.json').read_text())
rows['shutdown'] = {key:report.get(key) for key in ('shutdown_audit','gazebo_stop_service','process_errors','process_exit_codes','reader_forced_cleanup')}
text = json.dumps(rows, ensure_ascii=False, indent=2)
destination = Path(__file__).parent/('analysis.json' if run_name.endswith('_v1') else 'corner_analysis.json')
if destination.exists():
    raise FileExistsError(destination)
destination.write_text(text)
print(text)
