"""r43 소스 계약: 의미 보존 비용 절감(C3 blind_spec 1회 파싱, C2 같은 threat_view 호출 안 compact 재사용, 관측 격자 자유 표본 범위 제한).

주행 전략·임계값·여유·설정·센서·모델 변경 없음. 이전 계약은 고치지 않는다.
"""
import hashlib
import json
import zipfile
from pathlib import Path

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sha = lambda data: hashlib.sha256(data).hexdigest()
PARENT = 'config/tests/follow_bundle_closure_r42_sha256.json'
CONTRACT = ROOT/'config/tests/grid_cost_r43_sha256.json'
P = 'src/arena_autonomy/arena_autonomy/'
REASONS = {
    P+'local_scene.py': 'scene_from_scan parses the blind spec once per scan with the same formula and first-ray error order',
    P+'lateral_intent.py': 'ObservationGrid builds only free-sample indices that can reach the grid and may copy same-call compact returns',
    P+'pass_release_trial.py': 'threat_view passes its own same-call ReturnGrid to ObservationGrid for compact reuse',
}
before = A/'before.zip'
with zipfile.ZipFile(before) as archive:
    changes = {p: dict(before=sha(archive.read(p)), after=sha((ROOT/p).read_bytes()), reason=r) for p, r in REASONS.items()}
assert all(row['before'] != row['after'] for row in changes.values())
contract = dict(schema_version=1,
    scope=('관측 격자·장면 구성의 의미 보존 비용 절감(같은 격자 상태·장면·명령). 주행 전략·임계값·여유·설정·'
           '센서·모델·보호층 변경 없음.'),
    parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
    before_snapshot=before.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(before.read_bytes()),
    approved_changes=changes, approved_evaluation_scripts={}, new_dependencies={})
with CONTRACT.open('x', encoding='utf-8', newline='\n') as stream:
    stream.write(json.dumps(contract, ensure_ascii=False, indent=2)+'\n')
print(CONTRACT.relative_to(ROOT).as_posix(), sha(CONTRACT.read_bytes()))
