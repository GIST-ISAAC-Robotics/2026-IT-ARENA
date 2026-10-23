#!/usr/bin/env python3
"""기록한 G1 판단의 단계별 PC/Jetson 비용. ROS·센서 실시간 마감 시험은 아니다."""
import argparse
import gc
from collections import defaultdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import platform
import time
import zipfile

import numpy as np

from arena_autonomy.follow_trace import equal_output, freeze, replay_events, restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene

ROOT = Path(__file__).resolve().parents[1]
STAGES = ('restore_ms', 'projection_ms', 'decision_ms', 'total_ms', 'thread_cpu_ms', 'gc_elapsed_ms')
SNAPSHOT_MODULES = {
    'arena_autonomy': ('follow_trace', 'local_follow', 'boundary_evidence', 'corner_evidence', 'corner_distance', 'corner_corridor', 'corridor_observer', 'occupancy_timing',
                      'observation_support', 'observation_lateral', 'linear_observation',
                      'local_scene', 'scene_motion', 'lidar_motion', 'lidar_observation', 'local_path'),
    'arena_vehicle_interface': ('queued_log',),
}


class GCObservation:
    """계측만 수행한다. GC 설정·enable/disable·수동 collection을 바꾸지 않는다."""
    def __init__(self):
        self.start = None
        self.elapsed_ns = 0
        self.collections = 0
        self.generations = [0, 0, 0]

    def callback(self, phase, info):
        if phase == 'start':
            self.start = time.perf_counter_ns()
        elif phase == 'stop' and self.start is not None:
            self.elapsed_ns += time.perf_counter_ns()-self.start
            self.collections += 1
            self.generations[info['generation']] += 1
            self.start = None


def timed_call(follow, inputs, motion, clock=time.perf_counter_ns, gc_observer=None):
    gc_start = (gc_observer.elapsed_ns, gc_observer.collections) if gc_observer else (0, 0)
    generations_before = list(gc_observer.generations) if gc_observer else [0, 0, 0]
    cpu_begin = time.thread_time_ns()
    begin = clock()
    data, scan, history = restore_call(inputs, motion)
    restored = clock()
    projected = restored
    stage = 'projection'
    failed_stage = None
    try:
        scene = motion_scene(scan, history, data['reference_s'], timing_verified=data['timing_verified'],
                             epoch=data['epoch'], rear_half_angle_deg=data['rear_half_angle_deg'])
        projected = clock()
        stage = 'decision'
        output = follow.command(scene, history, data['speed_mps'], data['side'],
                                data['previous_steering_rad'], data['dt_s'])
    except (MotionUnavailable, ValueError) as error:
        if stage == 'projection':
            projected = clock()
        failed_stage = stage
        follow.reset()
        output = 0., 0., {'reason': 'follow_input_stop', 'behavior': 'HOLD',
                         'input_error': str(error), 'mode': 'g1_follow'}
    end = clock()
    cpu_end = time.thread_time_ns()
    # freeze/출력 비교·JSON 파싱은 아래 total 측정 밖이다. restore는 포함한다.
    return output, dict(restore_ms=(restored-begin)/1e6, projection_ms=(projected-restored)/1e6,
                        decision_ms=(end-projected)/1e6, total_ms=(end-begin)/1e6,
                        thread_cpu_ms=(cpu_end-cpu_begin)/1e6,
                        gc_elapsed_ms=(gc_observer.elapsed_ns-gc_start[0])/1e6 if gc_observer else 0.,
                        gc_collections=gc_observer.collections-gc_start[1] if gc_observer else 0,
                        gc_generations=[n-p for n, p in zip(gc_observer.generations, generations_before)]
                            if gc_observer else [0, 0, 0],
                        failed_stage=failed_stage)


def distribution(values):
    return dict(count=len(values), p50=float(np.percentile(values, 50)),
                p95=float(np.percentile(values, 95)), p99=float(np.percentile(values, 99)),
                maximum=max(values), over_20ms=sum(value > 20. for value in values))


def profile(events, repeats=3, warmups=1, clock=time.perf_counter_ns):
    return profile_source(lambda: iter(events), repeats, warmups, clock)


def profile_source(event_source, repeats=3, warmups=1, clock=time.perf_counter_ns):
    observation = GCObservation()
    callback = observation.callback
    gc.callbacks.append(callback)
    try:
        return _profile(event_source, repeats, warmups, clock, observation)
    finally:
        gc.callbacks.remove(callback)


def _profile(event_source, repeats, warmups, clock, observation):
    if type(repeats) is not int or not 1 <= repeats <= 10 or type(warmups) is not int or not 0 <= warmups <= 3:
        raise ValueError('bounded repeats/warmups required')
    # 실행 전 footer/순서/손실과 전체 출력 일치를 검증한다. 잘못된 기록의 빠른 실행은 성과가 아니다.
    baseline = replay_events(event_source())
    if not baseline['passed']:
        raise ValueError('recorded outputs do not match current source')
    matched = baseline['matched']
    del baseline  # 전체 원래 출력/경로 목록을 계측 동안 붙잡아 두지 않는다.
    records, mismatches = [], []
    for cycle in range(warmups+repeats):
        iterator = iter(event_source())
        header = next(iterator)['data']
        follow = LocalFollow(FollowConfig(**header['follow_config']))
        motion = MotionConfig(**header['motion_config'])
        previous_source = None
        for event in iterator:
            if event['kind'] == 'reset':
                follow.reset()
                previous_source = None
            elif event['kind'] == 'call':
                output, costs = timed_call(follow, event['data']['input'], motion, clock, observation)
                if not equal_output(freeze(output), event['data']['output']):
                    mismatches.append(dict(cycle=cycle, seq=event['seq']))
                raw = event['data']['input']['scan']
                source = (raw['sec'], raw['nanosec'])
                new_scan = source != previous_source
                previous_source = source
                if cycle >= warmups:
                    records.append(dict(repeat=cycle-warmups, seq=event['seq'],
                                        new_source_stamp=new_scan,
                                        reason=output[2]['reason'],
                                        selection=output[2].get('boundary_selection'), **costs))
    groups = defaultdict(list)
    for row in records:
        groups['all'].append(row)
        groups['new_scan' if row['new_source_stamp'] else 'reused_scan'].append(row)
        groups['gc_observed' if row['gc_collections'] else 'no_gc_observed'].append(row)
        if row['selection'] == 'bilateral_observed_fragments':
            groups['fragment_candidate_selected'].append(row)
        if row['selection'] == 'bilateral_observed_corner_anchor':
            groups['corner_anchor_selected'].append(row)
        if row['selection'] == 'bilateral_observed_corner_distance':
            groups['corner_distance_selected'].append(row)
    stats = {name: {stage: distribution([r[stage] for r in rows]) for stage in STAGES}
             for name, rows in groups.items()}
    return dict(passed=not mismatches, baseline_matched=matched, repeats=repeats, warmups=warmups,
                timed_calls=len(records), output_mismatches=mismatches, groups=stats, rows=records,
                scope='decision_stage_offline_replay_cost_not_ros_deadline_or_closed_loop',
                latency_limit_ms_reference_only=20., online_deadline_verified=False,
                timing_includes=['captured_history_restore', 'motion_scene_projection', 'local_follow_command'],
                timing_excludes=['file_read', 'json_parse', 'output_compare', 'ros_dds_callbacks',
                                 'vision', 'safety_node', 'actuator', 'real_sensor_arrival_and_clock_mapping'],
                clock_note='wall includes scheduling; thread_time excludes waiting and other worker threads; serialized warm-cache replay',
                gc_note='native GC observed only, never disabled or forced; overlap is correlation not proof of causality',
                grouped_scan_note='new_source_stamp is evaluator grouping, not runtime freshness authorization')


def digest(path):
    checksum = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b''):
            checksum.update(chunk)
    return checksum.hexdigest()


def read_events(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--warmups', type=int, default=1)
    parser.add_argument('--input-mode', choices=('stream', 'preload'), default='stream')
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    paths = [Path(__file__).resolve(), ROOT/'tests/test_follow_profile.py']
    paths += [ROOT/'src'/package/package/f'{name}.py'
              for package, names in SNAPSHOT_MODULES.items() for name in names]
    hashes = {str(p.relative_to(ROOT)): digest(p) for p in paths}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in paths:
            bundle.write(path, path.relative_to(ROOT))
    before = digest(args.input)
    try:
        if args.input_mode == 'preload':
            result = profile(list(read_events(args.input)), args.repeats, args.warmups)
        else:
            result = profile_source(lambda: read_events(args.input), args.repeats, args.warmups)
    except (ValueError, KeyError, TypeError) as error:
        result = dict(passed=False, error=str(error), online_deadline_verified=False)
    result.update(created_at=datetime.now(timezone.utc).isoformat(), input_sha256=before,
                  input_mode=args.input_mode,
                  input_unchanged=before == digest(args.input), source_sha256=hashes,
                  source_unchanged=all(digest(ROOT/p) == h for p, h in hashes.items()),
                  environment=dict(system=platform.system(), release=platform.release(), machine=platform.machine(),
                                   python=platform.python_version(), numpy=np.__version__,
                                   timer_resolution_s=time.get_clock_info('perf_counter').resolution))
    result['passed'] = result['passed'] and result['input_unchanged'] and result['source_unchanged']
    (output/'report.json').write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k: result.get(k) for k in ('passed', 'baseline_matched', 'timed_calls', 'groups', 'error')}, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
