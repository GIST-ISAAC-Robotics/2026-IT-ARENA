"""Git 제외 상세 상태에서 첫 래치 진단만 별도 보존한다. 재주행/정답 제어 입력 없음."""
import hashlib
import json
from pathlib import Path

OUT=Path(__file__).resolve().parent
source=OUT/'run2_lap/trajectory.jsonl'
for line in source.open(encoding='utf-8'):
    row=json.loads(line)
    policy=((row.get('safety') or {}).get('partial_observation') or {})
    if policy.get('latch_evidence'):
        result=dict(scope='first_recorded_latch_sensor_point_evidence_not_object_identity',
            record_time_s=row['time'],progress_m=row['progress_m'],
            latch_reason=policy['latch_reason'],latch_time_s=policy['latch_time_s'],
            evidence=policy['latch_evidence'],
            source_sha256={p.relative_to(OUT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in (source,Path(__file__),OUT/'run2_lap/report.json')})
        with (OUT/'run2_latch_evidence.json').open('x',encoding='utf-8') as stream:
            json.dump(result,stream,indent=2)
        print('saved first latch with source hashes')
        break
else:
    raise RuntimeError('no recorded diagnostic latch')
