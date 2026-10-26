"""선택형 연결 후 보존 G1/R0+ 판단 재생. 새 주행 동등성을 주장하지 않음."""
import json
from pathlib import Path
import sys
import hashlib

OUT = Path(__file__).resolve().parent
ROOT = OUT.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_events


if __name__ == '__main__':
    paths = [OUT.parent/'e3_harness_v1/run/follow_decisions.jsonl',
             OUT.parent/'e3_stop_restart_v1/run/follow_decisions.jsonl',
             OUT/'stopped_center/follow_decisions.jsonl']
    results = []
    for path in paths:
        with path.open() as stream:
            result = replay_events(json.loads(line) for line in stream)
        result.pop('outcomes')
        result.update(input=path.relative_to(ROOT).as_posix(),
                      sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        results.append(result)
    with (OUT/'completed_replay.json').open('x') as stream:
        json.dump(results, stream, indent=2)
    print(json.dumps(results, indent=2))
    raise SystemExit(0 if all(r['passed'] for r in results) else 1)
