"""구조 경로 증속 전제 수정을 r6 뒤 후속 출처 계약 r7로 기록한다. 이전 계약은 바꾸지 않는다.

before 바이트는 structure_gate_before.zip의 local_follow.py이며 r4(lidar_mount_speed)의 after와 같아야 한다.
"""
import hashlib
import json
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PATH = 'src/arena_autonomy/arena_autonomy/local_follow.py'
PARENT = 'config/tests/lidar_mount_traffic_marker_r6_sha256.json'
NAME = 'config/tests/lidar_mount_structure_gate_r7_sha256.json'
PREVIOUS = 'config/tests/lidar_mount_speed_r4_sha256.json'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    snapshot = HERE/'structure_gate_before.zip'
    with zipfile.ZipFile(snapshot) as archive:
        before = archive.read(PATH)
    previous = json.loads((ROOT/PREVIOUS).read_text(encoding='utf-8'))
    assert sha(before) == previous['approved_changes'][PATH]['after']
    after = (ROOT/PATH).read_bytes()
    assert sha(after) != sha(before)
    contract = dict(
        schema_version=1,
        scope='JR60P20 구조 적응 경로의 증속 후보 전제 수정. 선택형 structure_aware_v1에서만 동작 변경, off·기본 경로 불변. 주행 결과·실차 승인 아님',
        parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
        approved_changes={PATH: dict(before=sha(before), after=sha(after),
            reason='이번 스캔의 설명된 구조 광선만 ±100° 전방 증속 전제에서 제외. 일반 결측이 있으면 1.2 m/s 초과 후보를 내지 않고 '
                   '기본 목표가 v4 예산·감속 판단을 받는다(후보 속도로 인한 불허→정지 방지). 같은 순수 classify를 쓰며 정책 evaluate는 1회')},
        approved_evaluation_scripts={},
        new_dependencies={})
    text = json.dumps(contract, ensure_ascii=False, indent=2) + '\n'
    target = ROOT/NAME
    if target.exists():
        assert target.read_text(encoding='utf-8') == text, 'existing r7 differs; do not rewrite silently'
    else:
        target.write_text(text, encoding='utf-8', newline='\n')
    print(NAME, contract['approved_changes'][PATH]['after'])


if __name__ == '__main__':
    main()
