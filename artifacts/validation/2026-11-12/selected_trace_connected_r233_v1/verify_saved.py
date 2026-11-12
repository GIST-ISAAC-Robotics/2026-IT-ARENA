"""공통 재생기와 계측 반복의632개 판단을 원 기록과 대조한다."""
import hashlib
import json
from pathlib import Path
from profile_follow_trace import profile_source, read_events

A = Path(__file__).resolve().parent
trace = A.parent/'traffic_solo_gap_runtime_r225_v1/two_actor_run1/run1/follow_decisions.jsonl'
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = sha(trace)
result = profile_source(lambda: read_events(trace), repeats=1, warmups=0)
result.pop('rows', None)
result.update(input_sha256=before, input_unchanged=sha(trace)==before,
              scope='common replay and profiler decision equivalence; not runtime or speed evidence')
with (A/'saved_result.json').open('x') as f:
    json.dump(result, f, indent=2)
    f.write('\n')
print(json.dumps({k:result.get(k) for k in ('passed','baseline_matched','timed_calls','input_unchanged')}))
assert result['passed'] and result['input_unchanged'] and result['baseline_matched']==632
