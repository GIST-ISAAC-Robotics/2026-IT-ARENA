"""저장 원시 스캔에서 센서 방위 90~150° 무응답 광선 수만 센다. 판단·주행과 무관."""
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
FILES = ['artifacts/validation/2026-10-24/e3_onroad_v1/g1/follow_decisions.jsonl',
         'artifacts/validation/2026-10-24/e3_onroad_v1/r0plus/follow_decisions.jsonl',
         'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run/follow_decisions.jsonl']
result = {}
for name in FILES:
    calls = rays = with_any = 0
    for line in (ROOT/name).open():
        event = json.loads(line)
        if event['kind'] != 'call':
            continue
        scan = event['data']['input']['scan']
        calls += 1
        count = 0
        for i, value in enumerate(scan['ranges']):
            bearing = math.degrees(math.remainder(scan['angle_min']+i*scan['angle_increment'], math.tau))
            valid = (isinstance(value, (int, float)) and math.isfinite(value)
                     and scan['range_min'] <= value <= scan['range_max'])
            if 90 <= abs(bearing) <= 150 and not valid:
                count += 1
        rays += count
        with_any += count > 0
    result[name] = {'calls': calls, 'side_rear_nonreturn_rays': rays, 'calls_with_any': with_any}
(Path(__file__).resolve().parent/'unknown_check.json').write_text(json.dumps(result, indent=1), encoding='utf-8', newline='\n')
print(json.dumps(result, indent=1))
