"""공식 실패 입력의 상태를 이어 투영 원점 후보를 비교한다. 새 주행은 아니다."""
from collections import Counter
import hashlib
import json
from pathlib import Path
import sys
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT / 'src/arena_autonomy'), str(ROOT / 'src/arena_vehicle_interface'),
               str(BASE.parent / 'onroad_candidate_v1')]
from compare_saved import preflight, sha
from arena_autonomy import corridor_observer
from arena_autonomy.follow_trace import replay_call, freeze, thaw
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from origin_candidate import short_prefix_evidence


def main():
    output = BASE / 'trace_candidate_v1'
    output.mkdir(exist_ok=False)
    trace = BASE.parent / 'official_traffic_v1/run1_course/follow_decisions.jsonl'
    before = sha(trace)
    header, expected = preflight(trace)
    assert header['schema'] == 1 and header['scope'] == 'g1_decision_stage_only'
    config = FollowConfig(**header['follow_config'])
    assert config.short_boundary_prefix_enabled and config.traffic_path_recovery_trial
    files = [p for d in ('src/arena_autonomy', 'src/arena_vehicle_interface')
             for p in (ROOT / d).rglob('*.py') if '__pycache__' not in p.parts]
    files += [Path(__file__), BASE / 'origin_candidate.py', BASE.parent / 'onroad_candidate_v1/compare_saved.py']
    hashes = {p.relative_to(ROOT).as_posix(): sha(p) for p in files}
    with zipfile.ZipFile(output / 'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in files:
            archive.write(path, path.relative_to(ROOT).as_posix())
    (output / 'header.json').write_text(json.dumps(header, ensure_ascii=False, indent=2), encoding='utf-8')
    # 단일 격리 프로세스에서만 교체한다. 원본 소스와 경로/보호 정책은 그대로다.
    corridor_observer.short_prefix_evidence = short_prefix_evidence
    controller = LocalFollow(config)
    motion = MotionConfig(**header['motion_config'])
    rows = []
    with trace.open(encoding='utf-8') as stream, (output / 'decisions.jsonl').open('x', encoding='utf-8') as sink:
        for line in stream:
            event = json.loads(line)
            if event['kind'] == 'reset':
                controller.reset()
            if event['kind'] != 'call':
                continue
            data = event['data']
            speed, steering, detail = replay_call(controller, data['input'], motion)
            old = thaw(data['output'])
            row = dict(seq=event['seq'], t=data['input']['reference_s'],
                       recorded_speed=old[0], recorded_steering=old[1],
                       candidate_speed=speed, candidate_steering=steering,
                       command_equal=abs(speed-old[0]) <= 1e-10 and abs(steering-old[1]) <= 1e-10,
                       reason=detail.get('reason'), behavior=detail.get('behavior'),
                       selector_mode=detail.get('selector_mode'), selector_reason=detail.get('selector_reason'))
            sink.write(json.dumps(freeze(dict(**row, detail=detail)), ensure_ascii=False, allow_nan=False)+'\n')
            rows.append(row)
    assert len(rows) == expected and sha(trace) == before
    assert all(sha(ROOT / path) == digest for path, digest in hashes.items())
    changed = [row for row in rows if not row['command_equal']]
    stalled = [row for row in rows if 118. <= row['t'] < 162.6]
    summary = dict(scope='saved sensor trajectory; candidate commands do not move the recorded vehicle; independent guard not evaluated',
                   trace=trace.relative_to(ROOT).as_posix(), trace_sha256=before,
                   source_hashes=hashes, source_unchanged=True, input_unchanged=True,
                   calls=len(rows), changed_commands=len(changed), first_difference=next(iter(changed), None),
                   recorded_positive=sum(r['recorded_speed'] > 0 for r in rows),
                   candidate_positive=sum(r['candidate_speed'] > 0 for r in rows),
                   stall_window=dict(calls=len(stalled), positive=sum(r['candidate_speed'] > 0 for r in stalled),
                                     reasons=dict(Counter(str(r['reason']) for r in stalled)),
                                     first_positive=next((r for r in stalled if r['candidate_speed'] > 0), None)),
                   reasons=dict(Counter(str(r['reason']) for r in rows)))
    (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k != 'source_hashes'}, ensure_ascii=False))


if __name__ == '__main__':
    main()
