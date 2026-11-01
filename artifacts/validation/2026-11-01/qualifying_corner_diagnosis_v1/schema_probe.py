"""저장 원시 파일의 필드 구조만 확인한다. 값을 근거로 쓰지 않는다."""
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT / 'qualifying_profile_v1/run1'


def shape(value, depth=0, limit=3):
    if isinstance(value, dict):
        if depth >= limit:
            return f'dict[{len(value)}]'
        return {k: shape(v, depth + 1, limit) for k, v in value.items()}
    if isinstance(value, list):
        return f'list[{len(value)}]' + (f' of {shape(value[0], depth + 1, limit)}' if value else '')
    if isinstance(value, str):
        return f'str:{value[:40]}'
    return type(value).__name__ + f':{value}'


for name in ('trajectory.jsonl', 'follow_decisions.jsonl'):
    with (RUN / name).open(encoding='utf-8') as f:
        lines = [f.readline() for _ in range(2)]
    print('==', name, len(lines[1]))
    print(json.dumps(shape(json.loads(lines[1])), ensure_ascii=False, indent=1)[:9000])
for name in ('trace.json', 'actuation_trace.json', 'acquisition.json', 'progress.json'):
    with (RUN / name).open(encoding='utf-8') as f:
        data = json.load(f)
    print('==', name, type(data).__name__, len(data))
    print(json.dumps(shape(data, limit=3), ensure_ascii=False, indent=1)[:5000])
