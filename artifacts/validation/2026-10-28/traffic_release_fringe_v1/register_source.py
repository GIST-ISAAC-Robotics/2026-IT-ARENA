"""r0를 덮어쓰지 않고 후속 RELEASED 연결의 이전/현재 bytes를 등록한다."""
import hashlib
import json
from pathlib import Path
import zipfile

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
PARENT = 'config/tests/traffic_recovery_trial_r0_sha256.json'
TARGET = 'src/arena_autonomy/arena_autonomy/pass_release_trial.py'
NAME = 'config/tests/traffic_recovery_release_fringe_r1_sha256.json'


def sha(data):
    return hashlib.sha256(data).hexdigest()


if (ROOT/NAME).exists():
    raise FileExistsError(NAME)
parent = json.loads((ROOT/PARENT).read_text(encoding='utf-8'))
snapshot = BASE.parent/'traffic_recovery_connection_v1/run1_source_snapshot.zip'
manifest = json.loads((snapshot.parent/'run1_source_manifest.json').read_text())
expected = next(row['sha256'] for row in manifest if row['path'] == TARGET)
with zipfile.ZipFile(snapshot) as archive:
    previous = archive.read(TARGET)
assert sha(previous) == expected == parent['approved_changes'][TARGET]['after']
current = (ROOT/TARGET).read_bytes()
assert sha(current) != expected
before = BASE/'before.zip'
with zipfile.ZipFile(before, 'x', zipfile.ZIP_DEFLATED) as archive:
    archive.writestr(TARGET, previous)
contract = dict(schema_version=1,
    scope='기존 후반부 제한을 선택형 RELEASED에 연결한 출처. 기본 활성화/실물 안전 승인 아님',
    parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
    before_snapshot=before.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(before.read_bytes()),
    approved_changes={TARGET: dict(before=expected, after=sha(current),
        reason='회복 ON·유지 통과 이력·0.35 m/s 이하에 기존 후반1 mm 검사를 연결. 동일 경로/후반 분류 보고를 구분하며 OFF 유지.')},
    approved_evaluation_scripts={}, new_dependencies={})
(ROOT/NAME).write_text(json.dumps(contract, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps(dict(contract=NAME, before=expected, after=sha(current))))
