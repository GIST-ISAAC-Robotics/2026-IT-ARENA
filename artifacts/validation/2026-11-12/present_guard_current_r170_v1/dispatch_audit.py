"""기록된 모든 양수 상태의 현재 재계산 연결 범위. 지원 여부는 안전/주행 판정이 아니다."""
import collections
import importlib.util
import json
from pathlib import Path
A = Path(__file__).resolve().parent
p = A.parent/'present_guard_runtime_r171_v1/rebase.py'
spec = importlib.util.spec_from_file_location('r170_dispatch_audit', p)
candidate = importlib.util.module_from_spec(spec); spec.loader.exec_module(candidate)
trace = A.parent/'released_runtime_r168_v1/two_actor_run1/run1/follow_decisions.jsonl'
groups = collections.Counter(); samples = {}
for row in map(json.loads, trace.open()):
    if row['kind'] != 'call': continue
    output = row['data']['output']; normal = output[2].get('normal') or {}
    if output[0] <= 0: continue
    key = (output[2].get('handoff_state'), normal.get('selector_mode'), normal.get('selector_choice'),
           normal.get('reason'), candidate.supported(output))
    groups[key] += 1
    if not key[-1]: samples.setdefault(str(key), dict(seq=row['seq'], normal=normal))
report = dict(groups=[dict(state=k[0], selector_mode=k[1], choice=k[2], reason=k[3], supported=k[4], count=v)
                      for k,v in groups.most_common()], unsupported_samples=samples)
(A/'dispatch_coverage.json').write_text(json.dumps(report, indent=2)+'\n')
print(json.dumps(report['groups']))
