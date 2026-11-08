import json
from pathlib import Path
HERE=Path(__file__).resolve().parent
before=[json.loads(x) for x in (HERE.parent/'side_corner_observation_v1/replay_fragment_on_rows.jsonl').read_text().splitlines()]
after=[json.loads(x) for x in (HERE/'replay_connected_on_rows.jsonl').read_text().splitlines()]
assert len(before)==len(after)
differences=[i for i,(a,b) in enumerate(zip(before,after)) if a!=b]
result={'rows':len(before),'differences':differences,'same':not differences}
(HERE/'connected_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
print(result)
raise SystemExit(0 if not differences else 1)
