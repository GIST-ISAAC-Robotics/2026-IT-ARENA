"""선택지 추가 전 schema5 판단 기록을 현재 코드로 재생한다. 새 주행 아님.

header의 execution_config에 preparation_policy가 없으면 당시 즉시 준비 감속
(front_patch_v0)으로 복원되어야 한다. 원본 기록은 읽기만 한다.
"""
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_events

DAY = ROOT/'artifacts/validation/2026-10-29'
TRACES = [DAY/'g2_handoff_v1/integration3'/(name+'.jsonl')
          for name in ('right_ideal', 'left_combined', 'right_combined', 'moving_combined')]
TRACES += [DAY/'g2_handoff_gazebo_v1'/run/'g2_decisions.jsonl' for run in ('run1', 'run2')]


def main():
    sources = [p for p in (ROOT/'src').rglob('*.py') if '__pycache__' not in p.parts]+[Path(__file__)]
    hashes = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    results = []
    for path in TRACES:
        with path.open(encoding='utf-8') as stream:
            events = [json.loads(line) for line in stream]
        header = events[0]['data']
        replay = replay_events(events)
        results.append(dict(trace=path.relative_to(ROOT).as_posix(),
            trace_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
            header_has_preparation_policy='preparation_policy' in header.get('execution_config', {}),
            calls=replay['calls'], matched=replay['matched'], passed=replay['passed'],
            first_mismatch_seq=[m['seq'] for m in replay['mismatches'][:5]]))
        print(json.dumps(results[-1]), flush=True)
    report = dict(results=results, source_sha256=hashes,
        source_unchanged=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == v for p, v in hashes.items()),
        scope='Backward replay of preserved schema5 decision traces; not new driving.')
    (BASE/'old_schema5_replay.json').write_text(json.dumps(report, indent=2)+'\n', encoding='utf-8')
    if not all(r['passed'] for r in results) or not report['source_unchanged']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
