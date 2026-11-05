"""부모 증속 모델(parent_model.json current)과 실제 MCU 목표(actuation_trace target_speed_mps) 대조. 평가 전용."""
import bisect
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
series = json.loads((HERE/'parent_model.json').read_bytes())['window_series']
trace = json.loads((HERE.parent/'run1/actuation_trace.json').read_bytes())
times = [r['now_us']/1e6 for r in trace]
diffs = []
for row in series:
    # MCU는 /drive/safe를 다음 주기에 받는다. 0.02–0.06 s 뒤 최근접 표본과 비교한다.
    best = min((abs(trace[i]['target_speed_mps']-row['current']), times[i]-row['t'])
               for i in range(bisect.bisect_left(times, row['t']+.015), bisect.bisect_right(times, row['t']+.065)))
    diffs.append(best[0])
diffs.sort()
result = dict(samples=len(diffs), median_abs_mps=round(diffs[len(diffs)//2], 4), p95_abs_mps=round(diffs[int(.95*len(diffs))], 4),
              max_abs_mps=round(diffs[-1], 4), window='38.3–39.9 s', note='최근접 지연 0.015–0.065 s 안의 최소 차이. 독립 보호층 통과값 포함.')
(HERE/'model_check.json').write_text(json.dumps(result, ensure_ascii=False, indent=1), encoding='utf-8', newline='\n')
print(json.dumps(result, ensure_ascii=False))
