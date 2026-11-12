"""원 첫 실행의 camelCase JSON을 재해석한다. Gazebo 재실행/원 보고서 변경 없음."""
import hashlib
import json
import math
from pathlib import Path

A = Path(__file__).resolve().parent
source = A/'gpu_probe/scan512.jsonl'
data = json.loads(source.read_text().splitlines()[0])
raw = json.loads((A/'gpu_probe/report.json').read_text())
values = [float(x) for x in data['ranges']]
assert len(values) == data['count'] == 512
angles = [float(data['angleMin'])+i*float(data['angleStep']) for i in range(len(values))]
hits = {}
for outline, azimuth, yaw in raw['actors']:
    selected = [(i, a, r) for i, (a, r) in enumerate(zip(angles, values))
                if math.isfinite(r) and .65 < r < .95
                and abs(math.atan2(math.sin(a-azimuth), math.cos(a-azimuth))) < .2]
    hits[outline] = dict(count=len(selected), rays=selected)
result = dict(scope='saved stationary GPU scan decoded; 3600 stream not collected',
    initial_failure_preserved=raw.get('error'), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    hits=hits, stamp=data['header']['stamp'], cleanup_verified=raw['server_stop']['verified']
    and not raw['remaining_pids'] and not raw['launch_cleanup'])
assert not (A/'gpu_probe/decoded.json').exists()
(A/'gpu_probe/decoded.json').write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n')
print(json.dumps({k:v['count'] for k,v in hits.items()}))
