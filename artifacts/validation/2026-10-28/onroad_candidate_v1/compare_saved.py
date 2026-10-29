"""과거 좌우 피추월 입력에 최신 G1 구성을 적용하는 제한 개루프 비교.

측정 궤적/직전 조향은 당시 M3가 만든 값이다. 현재 G1의 실제 주행·보호층
통과·접촉/간격 보장이 아니며, 다른 제어기를 동일 재생했다고 표현하지 않는다.
"""
from collections import Counter
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import sys
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
for name in ('src/arena_autonomy', 'src/arena_vehicle_interface'):
    sys.path.insert(0, str(ROOT / name))

from arena_autonomy.follow_trace import replay_call, freeze, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow, FollowConfig

OLD = ROOT / 'artifacts/validation/2026-10-25/m3_execution_review_v1'
INPUTS = {
    'left': OLD / 'run17_observed_release_closed_loop/run/follow_decisions.jsonl',
    'right': OLD / 'run21_right_encounter/run/follow_decisions.jsonl',
}
CONFIG_SOURCE = ROOT / 'artifacts/validation/2026-10-28/traffic_side_scope_v1/run1_course/follow_decisions.jsonl'


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def preflight(path):
    """순서·footer·누락을 계산 전에 검사한다. 소스 재현성 판정과는 별개다."""
    header = footer = None
    calls = count = 0
    with path.open(encoding='utf-8') as stream:
        for count, line in enumerate(stream, 1):
            event = json.loads(line)
            if footer is not None or event['seq'] != count - 1:
                raise ValueError('trace order or trailing event')
            kind, data = event['kind'], event['data']
            if count == 1:
                if kind != 'header' or data['schema'] not in (1, 3):
                    raise ValueError('unexpected trace header')
                header = data
            elif kind == 'call':
                calls += 1
            elif kind == 'reset':
                pass
            elif kind == 'footer':
                footer = data
            else:
                raise ValueError('unexpected event')
    if not header or not footer or not calls:
        raise ValueError('incomplete capture')
    expected = dict(capture_errors=0, failed=0, dropped=0, pending=0,
                    worker_alive=False, written=count - 1, accepted=count - 1)
    if any(footer.get(k) != v for k, v in expected.items()):
        raise ValueError('capture queue incomplete')
    return header, calls


def compare(path, config, output):
    before = sha(path)
    header, expected_calls = preflight(path)
    if header.get('controller_mode') != 'm3_execution_trial':
        raise ValueError('expected historical M3 input')
    controller = LocalFollow(config)
    motion = MotionConfig(**header['motion_config'])
    rows, transitions, last_mode = [], [], object()
    with path.open(encoding='utf-8') as stream, output.open('x', encoding='utf-8') as sink:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'reset':
                controller.reset()
                last_mode = object()
                continue
            if event['kind'] != 'call':
                continue
            data = event['data']
            speed, steering, details = replay_call(controller, data['input'], motion)
            old = thaw(data['output'])
            mode = details.get('selector_mode', details.get('selector_mode_before'))
            row = dict(seq=event['seq'], t=data['input']['reference_s'],
                       recorded_speed=old[0], recorded_steering=old[1],
                       candidate_speed=speed, candidate_steering=steering,
                       command_equal=abs(speed-old[0]) <= 1e-10 and abs(steering-old[1]) <= 1e-10,
                       mode=mode, reason=details.get('reason'),
                       selector_reason=details.get('selector_reason'),
                       choice=details.get('selector_choice'))
            rows.append(row)
            sink.write(json.dumps(freeze(dict(**row, details=details)), ensure_ascii=False, allow_nan=False) + '\n')
            if mode != last_mode:
                transitions.append({k: row[k] for k in ('seq', 't', 'mode', 'choice', 'selector_reason')})
                last_mode = mode
    if len(rows) != expected_calls or sha(path) != before:
        raise ValueError('input changed or count mismatch')
    first = next((r for r in rows if not r['command_equal']), None)
    stop_streaks, start = [], None
    for index, row in enumerate(rows):
        stopped = row['candidate_speed'] <= 0
        if stopped and start is None:
            start = row['t']
        if start is not None and (not stopped or index == len(rows)-1):
            stop_streaks.append(dict(from_s=start, to_s=row['t'], duration_s=row['t']-start))
            start = None
    return dict(input=str(path.relative_to(ROOT)), sha256=before, input_unchanged=True,
                old_controller=header['controller_mode'], motion_config=header['motion_config'],
                calls=len(rows), recorded_positive=sum(r['recorded_speed'] > 0 for r in rows),
                candidate_positive=sum(r['candidate_speed'] > 0 for r in rows),
                command_equal=sum(r['command_equal'] for r in rows), first_difference=first,
                modes=dict(Counter(str(r['mode']) for r in rows)),
                reasons=dict(Counter(str(r['reason']) for r in rows)),
                selector_reasons=dict(Counter(str(r['selector_reason']) for r in rows)),
                mode_transitions=transitions,
                longest_candidate_zero_windows=sorted(stop_streaks, key=lambda v:v['duration_s'], reverse=True)[:5])


def main():
    output = BASE / 'run1'
    output.mkdir(exist_ok=False)
    with CONFIG_SOURCE.open(encoding='utf-8') as stream:
        event = json.loads(stream.readline())
    if event['kind'] != 'header' or event['data']['schema'] != 1:
        raise ValueError('expected G1 configuration header')
    config = FollowConfig(**event['data']['follow_config'])
    if not all(getattr(config, key) for key in ('parallel_hold_trial', 'pass_release_trial',
                                               'guard_curve_trial', 'traffic_recovery_trial',
                                               'traffic_path_recovery_trial')):
        raise ValueError('missing current selected trial')
    files = [p for directory in ('src/arena_autonomy', 'src/arena_vehicle_interface')
             for p in (ROOT / directory).rglob('*.py') if '__pycache__' not in p.parts]
    files.append(Path(__file__))
    hashes = {str(p.relative_to(ROOT)).replace('\\', '/'): sha(p) for p in files}
    with zipfile.ZipFile(output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in files:
            archive.write(path, path.relative_to(ROOT).as_posix())
    config_header = event['data']
    (output/'config_header.json').write_text(json.dumps(config_header, ensure_ascii=False, indent=2), encoding='utf-8')
    summary = dict(scope='historical M3 sensor trajectory to current G1 open-loop; no physical or independent guard validation',
                   config_source=str(CONFIG_SOURCE.relative_to(ROOT)),
                   config_header_sha256=hashlib.sha256(json.dumps(config_header, sort_keys=True).encode()).hexdigest(),
                   actual_follow_config=asdict(config), source_hashes=hashes,
                   inputs={name:compare(path, config, output/(name+'.jsonl')) for name,path in INPUTS.items()})
    summary['source_unchanged'] = all(sha(ROOT/p) == digest for p,digest in hashes.items())
    if not summary['source_unchanged']:
        raise ValueError('sources changed')
    (output/'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False), encoding='utf-8')
    print(json.dumps({k:{field:value[field] for field in ('calls','recorded_positive','candidate_positive','command_equal','modes')}
                      for k,value in summary['inputs'].items()}, ensure_ascii=False))


if __name__ == '__main__':
    main()
