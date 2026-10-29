"""교통 방지턱 방문·자세·속도의 사후 분석. 각본 actor 정답은 제어에 사용하지 않는다."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
DYNAMIC = BASE.parent/'bump_dynamic_v1'
sys.path.insert(0, str(DYNAMIC))
# 기존 검증된 방문/기하 함수만 재사용한다. 새 제어기를 추가하지 않는다.
spec = importlib.util.spec_from_file_location('bump_visits_base', DYNAMIC/'analyze_laps.py')
visits_module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(visits_module)


def digest(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def analyze(run):
    paths = [run/name for name in ('bump_pose_trace.json','trajectory.jsonl','report.json','trial_overlay.json')]
    before = {path.name: digest(path) for path in paths}
    poses = json.loads(paths[0].read_text(encoding='utf-8'))
    with paths[1].open(encoding='utf-8') as stream:
        trace = [json.loads(line) for line in stream]
    report = json.loads(paths[2].read_text(encoding='utf-8'))
    overlay = json.loads(paths[3].read_text(encoding='utf-8'))
    scene_path = ROOT/'artifacts/validation/2026-10-27/official_departure_v2/fixture/scene.json'
    bump = json.loads(scene_path.read_text(encoding='utf-8'))['speed_bumps']['bumps'][0]
    groups = visits_module.visits(poses, bump)
    windows = []
    for index, group in enumerate(groups, 1):
        crossing = visits_module.pose_metrics(group, bump)
        attitude = crossing.pop('near_attitude')
        if not attitude:
            windows.append(dict(visit=index, crossing=crossing, near_samples=0))
            continue
        first, last = attitude[0]['time'], attitude[-1]['time']
        local = [row for row in trace if first <= row['t'] <= last]
        times = [row['time'] for row in group]
        windows.append(dict(visit=index, crossing=crossing, near_range_s=[first,last],
            maximum_pose_gap_s=max((b-a for a,b in zip(times,times[1:])), default=None),
            sampled_body_speed_min_mps=min((row['ground_speed_mps'] for row in local), default=None),
            sampled_body_speed_max_mps=max((row['ground_speed_mps'] for row in local), default=None),
            sampled_safety_reasons=sorted({str(row['safety_reason']) for row in local}),
            safe_zero_samples=sum(row['safe_speed_mps']==0 for row in local),
            nearby_trace_samples=len(local)))
    checks = dict(geometry_visit=bool(windows) and any(row['crossing']['all_wheels_passed'] for row in windows),
        original_traffic_passed=report.get('passed') is True,
        runtime_contract=overlay.get('runtime_contract',{}).get('passed') is True,
        source_unchanged=overlay.get('source_unchanged') is True)
    return dict(scope='simulated four-wheel geometry/pose and sampled traffic; not force/contact/hardware proof',
        checks=checks, limited_passed=all(checks.values()), visits=windows,
        original_failed_gates=[key for key,ok in report.get('evaluation',{}).get('gates',{}).items() if not ok],
        input_sha256=before, input_unchanged=all(digest(path)==before[path.name] for path in paths),
        scene_sha256=digest(scene_path), helper_sha256={name:digest(DYNAMIC/name) for name in ('analyze.py','analyze_laps.py')})


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('run', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = analyze(args.run)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps(result, ensure_ascii=False))
