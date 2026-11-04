"""shadow 변동 하나의 HOLD 사유·시각 분포(개루프). 사용: 실행 jr|c 변동명... → stdout."""
import importlib.util, json, sys
from collections import Counter
from pathlib import Path
from arena_autonomy.follow_trace import replay_events
HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('sr', HERE/'shadow_replay.py'); sr = importlib.util.module_from_spec(spec); spec.loader.exec_module(sr)
run, layout, *names = sys.argv[1:]
sr.load(HERE/run/'follow_decisions.jsonl')
table = dict(sr.variants(sr.LAYOUTS[layout]))
for name in names:
    out = replay_events(sr.transform(sr.EVENTS, **table[name]) if table[name] else sr.EVENTS)['outcomes']
    holds = [o for o in out if o['speed_mps'] <= .001]
    t0 = out[0]['reference_s']
    print(name, Counter(o['details'].get('reason') for o in holds).most_common(6),
          'spans_s', sorted({round(o['reference_s']-t0) for o in holds}))
