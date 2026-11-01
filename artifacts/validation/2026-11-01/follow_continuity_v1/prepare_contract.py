"""이번 소스 변경을 이전 계약과 수정 전 사본에 잇는 후속 계약을 만든다. 소스가 바뀌면 다시 생성한다."""
from hashlib import sha256
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
PARENT = 'config/tests/recent_forward_r1_sha256.json'
OUTPUT = ROOT / 'config/tests/follow_continuity_r1_sha256.json'
SNAPSHOT = BASE / 'before.zip'
REASON = '전진율을 못 얻은 움직이는 조각에 정지 대비 고정 여유·준비 정지를 걸지 않는 선택형 비교; 기본값과 기존 정책은 그대로'


def digest(data):
    return sha256(data).hexdigest()


with zipfile.ZipFile(SNAPSHOT) as archive:
    def row(path):
        before = digest(archive.read(path))
        after = digest((ROOT / path).read_bytes())
        if before == after:
            raise ValueError(path + ': 변경 없음')
        return dict(before=before, after=after, reason=REASON)

    contract = dict(
        schema_version=1,
        scope='기존 1.5 m/s 교통 기준에서 고정 우회 여유를 연속 낮은 이동 조각에만 적용하는 선택형 비교',
        parent_contract=PARENT,
        parent_contract_sha256=digest((ROOT / PARENT).read_bytes()),
        before_snapshot=SNAPSHOT.relative_to(ROOT).as_posix(),
        before_snapshot_sha256=digest(SNAPSHOT.read_bytes()),
        approved_changes={p: row(p) for p in ('src/arena_autonomy/arena_autonomy/bypass_handoff.py',
                                              'src/arena_autonomy/arena_autonomy/g1_g2_node.py',
                                              'src/arena_bringup/launch/simulation.launch.py')},
        approved_evaluation_scripts={p: row(p) for p in ('scripts/g1_handoff_eval.py', 'scripts/validate_g1_traffic.py')},
        new_dependencies={})
with OUTPUT.open('w', encoding='utf-8', newline='\n') as f:
    json.dump(contract, f, ensure_ascii=False, indent=2)
    f.write('\n')
print(OUTPUT.relative_to(ROOT), digest(OUTPUT.read_bytes()))
