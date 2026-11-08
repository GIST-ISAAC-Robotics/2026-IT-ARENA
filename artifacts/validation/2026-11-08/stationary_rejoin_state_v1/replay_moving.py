"""기본 저속 r13의 실제 판단 재현. 고정 입력이며 중속 주행 증명이 아니다."""
import hashlib,json
from pathlib import Path
from arena_autonomy.follow_trace import replay_events
HERE=Path(__file__).resolve().parent
SOURCE=HERE.parents[3]/'artifacts/validation/2026-11-07/traffic_piecewise_v1/straight_moving_lead/run1/follow_decisions.jsonl'
with SOURCE.open() as f: result=replay_events(json.loads(line) for line in f if line.strip())
summary={k:v for k,v in result.items() if k not in ('outcomes','mismatches')}
summary.update(source_sha256=hashlib.sha256(SOURCE.read_bytes()).hexdigest(),mismatch_count=len(result['mismatches']))
(HERE/'moving_replay_result.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2)+'\n')
if result['mismatches']:(HERE/'moving_replay_mismatches.json').write_text(json.dumps(result['mismatches'][:3],ensure_ascii=False,indent=2)+'\n')
print(json.dumps(summary,ensure_ascii=False));assert result['passed']
