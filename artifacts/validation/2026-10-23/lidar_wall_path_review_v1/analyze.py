"""저장 정지 장면에서 전방 각도 양자화만 바꾼 반사실 경로 계산. 구동 없음."""
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path

import numpy as np
from arena_autonomy.local_path import local_path, pursuit_command

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
FIXTURE = OUT.parent/'lidar_wall_raster_v1'
paths = [FIXTURE/'run_fixture.py', FIXTURE/'result/scan_frames.json', FIXTURE/'result/report.json',
         ROOT/'src/arena_autonomy/arena_autonomy/local_path.py', Path(__file__)]
hashes = {str(p.relative_to(ROOT)): hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
spec = importlib.util.spec_from_file_location('wall_fixture_review', paths[0])
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
capture = json.loads(paths[2].read_text())
assert capture['capture_complete'] and capture['source_unchanged']
scans = json.loads(paths[1].read_text())


def endpoints(row):
    angles = row['angle_min']+np.arange(500)*row['angle_increment']
    ranges = np.array([math.inf if value is None else value for value in row['ranges']])
    selected = (np.isfinite(ranges) & (ranges >= .05) & (ranges < 12.)
                & (np.abs(angles) <= math.radians(150)))
    return np.column_stack((ranges[selected]*np.cos(angles[selected])+.06,
                            ranges[selected]*np.sin(angles[selected])))


def summarize(points, side):
    # fixture 양쪽 벽은 ±0.225 m. 운영 0.425 m 설정을 바꾸는 작업이 아니다.
    path, details = local_path(points, side, offset=.225)
    speed, steering, command = pursuit_command(points, side, speed=.5, wheelbase=.135,
        max_steering=.37, max_speed=1., lateral_acceleration=3.5, offset=.225, scan_age=0.)
    return dict(path=None if path is None else path.tolist(), path_reason=details['reason'],
        command_reason=command['reason'], target_speed_mps=speed, steering_rad=steering,
        selected_wall=command.get('path_wall'), observed_points=len(points))


rows = []
for case, frames in scans.items():
    assert len(frames) == 3
    for index, raw in enumerate(frames):
        nominal = copy.deepcopy(raw)
        changed = []
        for ray_index, measured in enumerate(raw['ranges']):
            angle = raw['angle_min']+ray_index*raw['angle_increment']
            if abs(angle) > math.radians(44):
                continue
            exact, _ = fixture.expected(case, angle)
            assert (measured is None or measured >= 12.) == (exact is None)
            # 이 장면에서 hit/no-hit는 같았다. 가려진/미관측 공간을 새로 채우지 않는다.
            if exact is not None:
                nominal['ranges'][ray_index] = exact
                changed.append(ray_index)
        for ray_index in set(range(500))-set(changed):
            assert nominal['ranges'][ray_index] == raw['ranges'][ray_index]
        for side in ('left', 'right'):
            before, after = summarize(endpoints(raw), side), summarize(endpoints(nominal), side)
            assert before['observed_points'] == after['observed_points']
            difference = None
            if before['path'] is not None and after['path'] is not None:
                difference = float(np.max(np.linalg.norm(np.array(before['path'])-after['path'], axis=1)))
            rows.append(dict(scene=case, frame=index, preferred_side=side,
                changed_front_range_count=len(changed), raw=before, counterfactual=after,
                maximum_corresponding_path_point_difference_m=difference,
                command_speed_difference_mps=after['target_speed_mps']-before['target_speed_mps'],
                command_steering_difference_rad=after['steering_rad']-before['steering_rad'],
                changed_path_reason=before['path_reason'] != after['path_reason'],
                changed_command_reason=before['command_reason'] != after['command_reason']))
assert all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
summary = {}
for case in scans:
    selected = [r for r in rows if r['scene'] == case]
    summary[case] = dict(comparisons=len(selected),
        path_reason_changes=sum(r['changed_path_reason'] for r in selected),
        command_reason_changes=sum(r['changed_command_reason'] for r in selected),
        max_path_point_difference_m=max((r['maximum_corresponding_path_point_difference_m'] for r in selected
            if r['maximum_corresponding_path_point_difference_m'] is not None), default=None),
        max_command_speed_difference_mps=max(abs(r['command_speed_difference_mps']) for r in selected),
        max_command_steering_difference_deg=math.degrees(max(abs(r['command_steering_difference_rad']) for r in selected)))
report = dict(scope='offline_counterfactual_front_ranges_only_not_controller_replay',
    source_hashes=hashes, source_unchanged=True, summary=summary, rows=rows,
    fixture_only_wall_offset_m=.225, production_wall_offset_modified=False,
    ground_truth_used_for_counterfactual=True, unknown_filled=False, actual_control_output=False,
    g1_state_machine_exercised=False, independent_guard_exercised=False,
    limits=['only the stored static scenes and +/-44 degree front sector',
            'three repeated frames do not provide independent scenario coverage',
            'nominal geometry is evaluation-only, not a sensor correction supplied to the running controller',
            'no real C1 noise, moving acquisition, tracking, braking, or whole-course claim'])
with (OUT/'report.json').open('x') as handle:
    json.dump(report, handle, indent=2, allow_nan=False)
print(json.dumps(summary))
