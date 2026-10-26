"""선별 입력 확인: 현재 소스가 세 저장 기록의 원래 판단을 그대로 재생하는지만 본다.

analyze.py가 쓰는 장면 재구성·당시 경로점이 원래 제어기의 것과 같은지 확인하는
관련 회귀다. 새 판단·주행·판정이 아니다.
"""
import hashlib
import json
from pathlib import Path
import sys

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_events  # noqa: E402

TRACES = ['artifacts/validation/2026-10-24/e3_onroad_v1/g1/follow_decisions.jsonl',
          'artifacts/validation/2026-10-24/e3_onroad_v1/r0plus/follow_decisions.jsonl',
          'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl']


def main():
    result = {}
    for name in TRACES:
        path = ROOT/name
        with path.open() as stream:
            outcome = replay_events(json.loads(line) for line in stream)
        result[name] = {'sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'scope': outcome['scope'],
                        'calls': outcome['calls'], 'matched': outcome['matched'], 'passed': outcome['passed'],
                        'first_mismatch_seq': outcome['mismatches'][0]['seq'] if outcome['mismatches'] else None}
    (OUT/'replay_check.json').write_text(json.dumps(result, indent=1), encoding='utf-8', newline='\n')
    print(json.dumps(result, indent=1))
    if not all(v['passed'] for v in result.values()):
        raise SystemExit(1)


if __name__ == '__main__':
    main()
