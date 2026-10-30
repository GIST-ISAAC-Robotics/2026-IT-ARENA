"""기본OFF 과거 재생과 명시ON의 미연결 후보 일치를 같은 입력으로 대조한다."""
from dataclasses import replace
import hashlib
import json
from pathlib import Path
import sys
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT / 'src/arena_autonomy'), str(ROOT / 'src/arena_vehicle_interface'), str(ROOT / 'scripts')]
from arena_autonomy.bypass_handoff import BypassHandoffFollow, HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call, freeze, equal_output, comparable_decision
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.lidar_motion import MotionConfig


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def compare(path, end=None, expected_path=None):
    digest = sha(path)
    changes = {row['seq']: row for row in map(json.loads, expected_path.open())} if expected_path else {}
    calls = off_equal = on_equal = normal_equal = hold_equal = 0
    mismatches = []
    footer = None
    for seq, line in enumerate(path.open()):
        event = json.loads(line)
        assert event['seq'] == seq and footer is None
        kind, data = event['kind'], event['data']
        if seq == 0:
            assert kind == 'header' and data['schema'] == 5
            execution = HandoffExecutionConfig.from_mapping(data['execution_config'])
            assert execution.preparation_hold_policy == 'off'
            config = FollowConfig(**data['follow_config'])
            off = BypassHandoffFollow(config, execution, allow_simulation_trial=True)
            on = BypassHandoffFollow(config, replace(execution, preparation_hold_policy='bounded_hold_v1'),
                allow_simulation_trial=True) if expected_path else None
            motion = MotionConfig(**data['motion_config'])
        elif kind == 'reset':
            off.reset()
            if on is not None:
                on.reset()
        elif kind == 'call':
            if end is not None and data['input']['reference_s'] > end:
                break
            old = data['output']
            actual = freeze(replay_call(off, data['input'], motion))
            matched = equal_output(comparable_decision(actual), comparable_decision(old))
            calls += 1
            off_equal += matched
            candidate_ok = True
            if on is not None:
                candidate = freeze(replay_call(on, data['input'], motion))
                expected = changes.get(seq)
                expected_command = expected['candidate_command'] if expected else old[:2]
                command_ok = equal_output(candidate[:2], expected_command)
                normal_ok = equal_output(comparable_decision([0, 0, candidate[2].get('normal')]),
                    comparable_decision([0, 0, old[2].get('normal')]))
                reserve = candidate[2].get('follow_entry_space') or {}
                memory_ok = (equal_output(reserve, expected['reserve']) if expected else
                    reserve.get('reason') != 'bounded_preparation_hold_candidate')
                on_equal += command_ok
                normal_equal += normal_ok
                hold_equal += memory_ok
                candidate_ok = command_ok and normal_ok and memory_ok
            if (not matched or not candidate_ok) and len(mismatches) < 4:
                mismatches.append(dict(seq=seq, time_s=data['input']['reference_s'], off=matched, on=candidate_ok))
        elif kind == 'footer':
            footer = data
        else:
            raise ValueError('unrecognized trace event')
    if end is None:
        assert footer and footer['accepted'] == footer['written'] == seq
        assert footer['worker_alive'] is False and all(footer[k] == 0 for k in ('failed', 'dropped', 'pending', 'capture_errors'))
    return dict(input=str(path.relative_to(ROOT)), sha256=digest, input_unchanged=sha(path) == digest,
        prefix_end_s=end, footer_checked=end is None, calls=calls, off_matched=off_equal,
        on_command_matched=on_equal if on else None, on_normal_matched=normal_equal if on else None,
        on_memory_detail_matched=hold_equal if on else None, first_mismatches=mismatches,
        passed=bool(calls) and off_equal == calls and (on is None or on_equal == normal_equal == hold_equal == calls)
            and sha(path) == digest)


if __name__ == '__main__':
    output = BASE / 'run1'
    output.mkdir(exist_ok=False)
    files = [*BASE.glob('*.py'), *BASE.glob('*.sh'), BASE / 'PLAN.md']
    files.extend(ROOT / path for path in ('tests/test_bypass_preparation_hold.py',
        'tests/test_g2_preparation_hold_provenance.py', 'tests/test_source_provenance_r0_plus_b3.py',
        'config/tests/g2_preparation_hold_r1_sha256.json', 'config/tests/g2_handoff_corridor_r1_sha256.json'))
    for directory in ('src/arena_autonomy/arena_autonomy', 'src/arena_vehicle_interface/arena_vehicle_interface'):
        files.extend((ROOT / directory).glob('*.py'))
    hashes = {p.relative_to(ROOT).as_posix(): sha(p) for p in files}
    with zipfile.ZipFile(output / 'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name in hashes:
            archive.write(ROOT / name, name)
    evidence = BASE.parent / 'g2_preparation_hold_v1/run1'
    cases = [('2026-10-31/g2_blocked_course_v3/run1/follow_decisions.jsonl', 35., evidence / 'g2_blocked_course_v3_changes.jsonl'),
        ('2026-10-31/g2_course_trial_v1/run1/follow_decisions.jsonl', None, evidence / 'g2_course_trial_v1_changes.jsonl'),
        ('2026-10-30/g2_handoff_corridor_gazebo_v1/run1/g2_decisions.jsonl', None, None)]
    results = [compare(ROOT / 'artifacts/validation' / case, end, expected)
        for case, end, expected in cases]
    unchanged = all(sha(ROOT / p) == digest for p, digest in hashes.items())
    result = dict(results=results, source_sha256=hashes, source_unchanged=unchanged,
        passed=unchanged and all(r['passed'] for r in results), scope='recorded inputs and explicit policy; not closed-loop proof')
    (output / 'summary.json').write_text(json.dumps(result, indent=2) + '\n')
    print(json.dumps({k: result[k] for k in ('results', 'source_unchanged', 'passed')}))
    raise SystemExit(0 if result['passed'] else 1)
