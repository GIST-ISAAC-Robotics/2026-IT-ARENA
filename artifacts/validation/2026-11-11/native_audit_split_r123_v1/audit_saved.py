"""r114 원 입력을 바꾸지 않고 기존 실패와 새 분해 진단을 함께 보존한다."""
import hashlib
import json
from pathlib import Path
from native_traffic_contract import NativeSamplingAudit

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
RUN=A.parent/'fresh_submission_r114_v1/two_actor_run1/run1'
files=[RUN/'acquisition.json',RUN/'scans_received.jsonl',RUN/'report.json']
digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before={p.name:digest(p) for p in files}
records=json.loads(files[0].read_bytes())
received=[json.loads(s) for s in files[1].open()]
profile=json.loads((ROOT/'config/tests/c1_standard10_native_angles_v1.json').read_bytes())
result=NativeSamplingAudit(profile,10.).audit(records,received)
assert result['verified'] is False
assert result['completed_payload']['all_completed_sources_received']
assert result['completed_payload']['acquisition_gap_count']==2
out=dict(audit=result,source_sha256=before,source_unchanged=before=={p.name:digest(p) for p in files},
         original_report_unchanged=True,scope='Diagnostic addition only; original failure remains')
with (A/'comparison.json').open('x') as f:json.dump(out,f,indent=2)
print(json.dumps(out))
