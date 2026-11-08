"""현재(r42 작업트리) 코드로 r15 저장 판단을 1회 재생한다. 고정 입력 재생이며 새 주행 증명이 아니다.

판단 단계(속도·조향 출력) 일치만 본다. r15 당시와 전체 소스/메타데이터 동일성은 별도(code_delta.json)이다.
"""
import hashlib
import json
from pathlib import Path

from arena_autonomy.follow_trace import replay_events

HERE = Path(__file__).resolve().parent
SOURCE = HERE.parent/'medium_pass_connected_v1/straight_moving_lead/run1/follow_decisions.jsonl'
with SOURCE.open(encoding='utf-8') as stream:
    result = replay_events(json.loads(line) for line in stream if line.strip())
summary = {k: v for k, v in result.items() if k not in ('outcomes', 'mismatches')}
summary.update(source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(), mismatch_count=len(result['mismatches']))
(HERE/'replay_r15_result.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n', encoding='utf-8', newline='\n')
if result['mismatches']:
    (HERE/'replay_r15_mismatches.json').write_text(json.dumps(result['mismatches'][:20], ensure_ascii=False, indent=1, default=str)+'\n',
                                                 encoding='utf-8', newline='\n')
print(json.dumps(summary, ensure_ascii=False))
