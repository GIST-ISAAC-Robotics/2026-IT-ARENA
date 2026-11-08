"""r42 소스 계약: follow-profile 묶음 도구의 정적 import 폐포 전환. 제어기·보호층·기본값 변경 없음."""
import hashlib
import json
import zipfile
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sha = lambda data: hashlib.sha256(data).hexdigest()
PARENT = 'config/tests/current_corner_support_r41_sha256.json'
CONTRACT = ROOT/'config/tests/follow_bundle_closure_r42_sha256.json'
REASONS = {
    'scripts/prepare_jetson_bundle.py': 'follow-profile bundle collects explicit entry points by static import closure',
    'scripts/profile_follow_trace.py': 'source snapshot module list derived from static import closure',
    'scripts/verify_follow_profile_bundle.py': 'verifier recloses imports from archive members only',
}
NEW = ('scripts/python_dependency_closure.py',)
before = A/'before.zip'
with zipfile.ZipFile(before) as archive:
    changes = {p: dict(before=sha(archive.read(p)), after=sha((ROOT/p).read_bytes()), reason=r) for p, r in REASONS.items()}
assert all(row['before'] != row['after'] for row in changes.values())
contract = dict(schema_version=1,
    scope='Jetson follow-profile 묶음·계측 소스 스냅샷의 내부 의존성을 정적 import 폐포로 수집. 주행 제어기·보호층·기본값·물성 변경 없음.',
    parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
    before_snapshot=before.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(before.read_bytes()),
    approved_changes={}, approved_evaluation_scripts=changes,
    new_dependencies={p: sha((ROOT/p).read_bytes()) for p in NEW},
    revision=('r42 완료 전 수정: profile_follow_trace CLI가 묶음에 없는 tests/test_follow_profile.py를 선택 기록으로 처리. '
              '최초 r42 after(ddc6b2ec...)는 v1 묶음과 v2/r42_contract_initial.json에 보존.'))
with CONTRACT.open('x', encoding='utf-8', newline='\n') as stream:
    stream.write(json.dumps(contract, ensure_ascii=False, indent=2)+'\n')
print(CONTRACT.relative_to(ROOT).as_posix(), sha(CONTRACT.read_bytes()))
