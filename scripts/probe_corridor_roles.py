#!/usr/bin/env python3
"""저장 입력에서만 회랑 기하 역할을 대조한다. ROS·실행 파일·원래 결과 미변경."""
import argparse
from collections import Counter
import json
from pathlib import Path
import sys
import zipfile

from arena_autonomy import corridor_observer, observation_support, observation_lateral
from arena_autonomy.corridor_roles import role_aware_evidence
from arena_autonomy.follow_trace import equal_output, freeze, replay_events, restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.scene_motion import motion_scene
from audit_corridor_runtime import digest

ROOT = Path(__file__).resolve().parents[1]


def events(path):
    with path.open(encoding='utf-8') as stream:
        for line in stream:
            yield json.loads(line)


def compare(capture):
    baseline = replay_events(events(capture))
    if not baseline['passed']:
        raise ValueError('original output replay must match before role comparison')
    matched = baseline['matched']
    del baseline
    original = corridor_observer.corridor_evidence
    rows, reasons, calls, changed_commands, changed_details = [], Counter(), 0, 0, 0
    # 이 독립 평가 프로세스의 함수 참조만 바꾼다. 생성자/상태/보호 조건은 그대로다.
    # 이후 가져오는 다른 실행 코드에 전파하지 않도록 예외에서도 원상 복귀한다.
    corridor_observer.corridor_evidence = role_aware_evidence
    try:
        for event in events(capture):
            if event['kind'] == 'header':
                data = event['data']
                cfg = FollowConfig(**data['follow_config'])
                motion = MotionConfig(**data['motion_config'])
                follow = LocalFollow(cfg)
            elif event['kind'] == 'reset':
                follow.reset()
            elif event['kind'] == 'call':
                data, scan, history = restore_call(event['data']['input'], motion)
                scene = motion_scene(scan, history, data['reference_s'],
                    timing_verified=data['timing_verified'], epoch=data['epoch'],
                    rear_half_angle_deg=data['rear_half_angle_deg'])
                output = follow.command(scene, history, data['speed_mps'], data['side'],
                    data['previous_steering_rad'], data['dt_s'])
                old = event['data']['output']
                changed = not equal_output(freeze(output[:2]), old[:2])
                changed_detail = not equal_output(freeze(output), old)
                changed_commands += changed
                changed_details += changed_detail
                calls += 1
                reasons[output[2]['reason']] += 1
                if changed or 3141 <= event['seq'] <= 3181 or event['seq'] in (3201, 3301, 3401, 3501):
                    rows.append(dict(seq=event['seq'], time_s=data['reference_s'],
                        measured_motor_speed_mps=data['speed_mps'], preferred_side=data['side'],
                        old_command=old[:2], new_command=freeze(output[:2]),
                        old_reason=old[2]['reason'], new_reason=output[2]['reason'],
                        command_changed=changed, detail=freeze(output[2])))
    finally:
        corridor_observer.corridor_evidence = original
    return dict(baseline_matched=matched, calls=calls, changed_commands=changed_commands,
        changed_details=changed_details, reasons=dict(reasons), rows=rows,
        function_restored=corridor_observer.corridor_evidence is original,
        new_closed_loop_verified=False, independent_safety_verified=False,
        branch_semantics_verified=False, baseline_wall_memory_policy_changed=False,
        runtime_files_changed=False, old_failure_changed=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output == ROOT/'artifacts':
        parser.error('fresh artifact subdirectory required')
    output.mkdir(parents=True, exist_ok=False)
    before = digest(args.capture)
    paths = {Path(__file__).resolve(), ROOT/'tests/test_corridor_roles.py'}
    for module in tuple(sys.modules.values()):
        name = getattr(module, '__file__', None)
        if name:
            path = Path(name).resolve()
            if path.is_relative_to(ROOT) and path.suffix == '.py':
                paths.add(path)
    sources = {p.relative_to(ROOT).as_posix(): digest(p) for p in sorted(paths)}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in sources:
            archive.write(ROOT/name, name)
    result = compare(args.capture)
    result.update(scope='offline_role_assignment_counterfactual_only', source_sha256=sources,
        source_unchanged=all(digest(ROOT/n) == h for n, h in sources.items()),
        input_sha256=before, input_unchanged=digest(args.capture) == before)
    (output/'report.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'source_sha256')}))
    return 0 if result['function_restored'] and result['input_unchanged'] and result['source_unchanged'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
