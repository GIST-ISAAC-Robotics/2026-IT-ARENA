"""Git 제외 상세 상태에서 첫 실패와 해시를 보존한다. 원시 전체 로그는 복사하지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('run',type=Path)
parser.add_argument('--output',type=Path,required=True)
args=parser.parse_args()
source=args.run/'trajectory.jsonl'
for line in source.open(encoding='utf-8'):
    row=json.loads(line)
    policy=(row.get('safety') or {}).get('partial_observation') or {}
    if policy.get('latch_evidence'):
        result=dict(scope='first_recorded_latch_sensor_point_evidence_not_object_identity',
            record_time_s=row['time'],progress_m=row['progress_m'],
            latch_reason=policy['latch_reason'],latch_time_s=policy['latch_time_s'],
            evidence=policy['latch_evidence'],
            source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest()
                           for p in (source,Path(__file__),args.run/'report.json')})
        with args.output.open('x',encoding='utf-8') as stream:json.dump(result,stream,indent=2)
        print('saved first latch context with source hashes')
        break
else:raise RuntimeError('no recorded diagnostic latch')
