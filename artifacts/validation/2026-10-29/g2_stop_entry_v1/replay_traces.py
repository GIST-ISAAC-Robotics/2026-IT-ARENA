"""G2 비교 함수에 여유 증가 인자(기본 0)를 더한 뒤 기존 schema5 기록이 그대로 재생되는지 확인한다.

closed1의 v1 판단 기록7개, legacy_stop1의 v0 기록, g2_handoff_v1 integration3 4개, 이번 closed_space1 6개를 재생한다.
원본 기록은 읽기만 한다. 새 주행 아님.
"""
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from arena_autonomy.follow_trace import replay_events

DAY = BASE.parent
traces = sorted((DAY/'g2_prepare_cap_v1/closed1').glob('*.jsonl'))+sorted((BASE/'legacy_stop1').glob('*.jsonl'))
traces += sorted((DAY/'g2_handoff_v1/integration3').glob('*.jsonl'))+sorted((BASE/'closed_space1').glob('*.jsonl'))
results = []
for path in traces:
    with path.open(encoding='utf-8') as stream:
        replay = replay_events(json.loads(line) for line in stream)
    results.append(dict(trace=path.relative_to(ROOT).as_posix(), calls=replay['calls'], matched=replay['matched'], passed=replay['passed']))
    print(json.dumps(results[-1]), flush=True)
sources = {p.relative_to(ROOT).as_posix(): hashlib.sha256(p.read_bytes()).hexdigest()
           for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('bypass_*.py')}
(BASE/'trace_replay.json').write_text(json.dumps(dict(results=results, source_sha256=sources,
    scope='Schema5 decision replay with current code; not new driving.'), indent=2)+'\n', encoding='utf-8')
if not all(r['passed'] for r in results):
    raise SystemExit(1)
