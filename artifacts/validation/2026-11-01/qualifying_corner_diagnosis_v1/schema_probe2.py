"""follow_decisions의 종류·필드 구조와 actuation_trace 시간 범위를 확인한다."""
import collections
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'qualifying_profile_v1/run1'


def shape(value, depth=0, limit=4):
    if isinstance(value, dict):
        if depth >= limit:
            return f'dict[{len(value)}]'
        return {k: shape(v, depth + 1, limit) for k, v in value.items()}
    if isinstance(value, list):
        return f'list[{len(value)}]'
    if isinstance(value, str):
        return f'str:{value[:50]}'
    return type(value).__name__ + f':{value}'


kinds = collections.Counter()
samples = {}
late = None
with (RUN / 'follow_decisions.jsonl').open(encoding='utf-8') as f:
    for line in f:
        row = json.loads(line)
        k = row.get('kind')
        kinds[k] += 1
        samples.setdefault(k, row)
        if k not in ('reset',):
            late = row
print(kinds)
for k, row in samples.items():
    print('==', k)
    print(json.dumps(shape(row), ensure_ascii=False, indent=1)[:6000])
print('== late')
print(json.dumps(shape(late), ensure_ascii=False, indent=1)[:9000])
with (RUN / 'actuation_trace.json').open(encoding='utf-8') as f:
    act = json.load(f)
print(len(act), act[-1])
st = collections.Counter((a['state'], a['reason']) for a in act)
print(st.most_common(12))
