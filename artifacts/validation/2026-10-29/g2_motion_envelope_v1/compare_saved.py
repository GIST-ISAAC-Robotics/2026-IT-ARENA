"""같은 저장 장면/수치를 유지한 팽창 도형 비교. 명령/재생 원본은 바꾸지 않는다."""
import hashlib
import json
from pathlib import Path
from rounded_sweep import rounded_swept_limit

BASE=Path(__file__).resolve().parent
source=BASE.parent/'g2_tracking_preview_v1/prototype2_stop_diagnosis.json'
raw=json.loads(source.read_text())
rows=[]
for item in raw['samples']:
    geometry=item['geometry']
    for old in geometry['first_interference']:
        clearance=rounded_swept_limit(geometry['points'],old['steering'],wheelbase=.135,
                                     horizon=2.5,margin=geometry['padding_m'])
        rows.append(dict(time_s=item['t'],steering_rad=old['steering'],
                         original_clearance_m=old['clearance_m'],rounded_clearance_m=clearance))
output=BASE/'saved_comparison.json'
if output.exists():raise RuntimeError('preserve prior diagnostic')
output.write_text(json.dumps(dict(source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    code_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in BASE.glob('*.py')},
    scope='fixed observed scene; changed displacement norm geometry, no new control/observation approval',cases=rows),indent=2)+'\n')
print(json.dumps(rows,indent=2))
