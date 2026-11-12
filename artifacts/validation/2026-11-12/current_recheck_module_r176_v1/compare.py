"""원 r174 함수와 공통 함수의 저장 입력/현재 검사 결과 대조. ROS·구동 없음."""
import copy
import hashlib
import importlib.util
import json
from collections import Counter
from pathlib import Path

from arena_autonomy import current_path_recheck as candidate
from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
TRACES = {
    'r152': 'adjacent_live_r152_v1/two_actor_run2',
    'r168': 'released_runtime_r168_v1/two_actor_run1',
    'r171': 'present_guard_runtime_r171_v1/two_actor_run1',
    'r174': 'released_parallel_runtime_r174_v1/two_actor_run1',
}


def main():
    path = A.parent/'released_parallel_runtime_r174_v1/rebase.py'
    spec = importlib.util.spec_from_file_location('r176_original_r174', path)
    original = importlib.util.module_from_spec(spec); spec.loader.exec_module(original)
    counts, rejected, scopes = Counter(), Counter(), Counter()
    mismatches, input_errors, rows, hashes = [], [], [], {}
    for label, folder in TRACES.items():
        trace = A.parent/folder/'run1/follow_decisions.jsonl'
        hashes[label] = hashlib.sha256(trace.read_bytes()).hexdigest()
        with trace.open() as stream:
            header = json.loads(next(stream))['data']
            motion = MotionConfig(**header['motion_config'])
            cfg = FollowConfig(**header['follow_config'])
            for event in map(json.loads, stream):
                if event['kind'] != 'call':
                    continue
                item = event['data']; out = item['output']; submitted = item['input']
                selected = candidate.scope(out)
                counts['all_calls'] += 1
                if original.supported(out) != candidate.supported(out):
                    mismatches.append([label, event['seq'], 'scope'])
                if selected is None:
                    assert original.rebase(out, None, None, None, None, None) is None
                    assert candidate.rebase(out, None, None, None, None, None) is None
                    counts['unsupported'] += 1
                    continue
                scopes[selected] += 1
                cases = [('same_time', submitted, None)]
                if 'current_rebase' in item:
                    cases.append(('recorded_current', item['current_rebase']['input'],
                                  item['current_rebase'].get('output') if label == 'r174' else None))
                if label == 'r171':
                    for offset in (.02, .04):
                        now = copy.deepcopy(submitted); now['reference_s'] += offset
                        cases.append((f'offset_{offset}', now, None))
                for kind, now, recorded in cases:
                    identity = [label, event['seq'], kind]
                    before = copy.deepcopy((out, submitted, now))
                    try:
                        _, scan, history = restore_call(now, motion)
                        scene = motion_scene(scan, history, now['reference_s'],
                            timing_verified=now['timing_verified'], epoch=now['epoch'],
                            rear_half_angle_deg=now['rear_half_angle_deg'])
                    except MotionUnavailable as error:
                        input_errors.append(dict(case=identity, error=str(error)))
                        continue
                    old = original.rebase(out, submitted, now, scene, history, cfg)
                    new = candidate.rebase(out, submitted, now, scene, history, cfg)
                    if not equal_output(freeze(old), freeze(new)):
                        mismatches.append(identity+['full_return'])
                    assert equal_output(freeze((out, submitted, now)), freeze(before))
                    assert new is not None and 0 <= new['speed_mps'] <= out[0]
                    assert new['path_source_s'] == out[2]['normal']['path_source_s']
                    assert new['independent_guard_required'] is True
                    if recorded is not None:
                        counts['recorded_current'] += 1
                        if not equal_output(freeze(new), recorded):
                            mismatches.append(identity+['recorded_current'])
                    counts['compared_returns'] += 1
                    counts['positive_returns'] += new['speed_mps'] > 0
                    if new['speed_mps'] <= 0: rejected[new['reason']] += 1
                    rows.append(dict(case=identity, scope=selected, reason=new['reason'],
                                     speed_mps=new['speed_mps'], steering_rad=new['steering_rad']))
    assert hashes == {label: hashlib.sha256((A.parent/folder/'run1/follow_decisions.jsonl').read_bytes()).hexdigest()
                      for label, folder in TRACES.items()}
    report = dict(counts=dict(counts), scopes=dict(scopes), rejected=dict(rejected),
                  mismatches=mismatches, unavailable_input_cases=input_errors,
                  input_sha256=hashes, inputs_unchanged=True,
                  default_or_ros_connection=False, physical_drive_test=False)
    with (A/'comparison.json').open('x', encoding='utf8') as stream:
        json.dump(report, stream, indent=2); stream.write('\n')
    with (A/'comparison_rows.json').open('x', encoding='utf8') as stream:
        json.dump(rows, stream, indent=2); stream.write('\n')
    print(json.dumps({k:v for k,v in report.items() if k != 'unavailable_input_cases'}))
    assert not mismatches
    assert set(scopes) == {'solo_baseline', 'solo_or_guard_parallel', 'absent_guard_baseline',
                          'released_baseline', 'present_guard_baseline', 'released_parallel'}


if __name__ == '__main__':
    main()
