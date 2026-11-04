"""교통 검사기의 opt-in 후면 마커 높이 인자를 r5 뒤 후속 출처 계약 r6로 기록한다. 이전 계약은 바꾸지 않는다.

before 바이트는 HEAD의 scripts/validate_g1_traffic.py이며 r1(lidar_mount_trial)의 after와 같아야 한다.
"""
import hashlib
import json
import subprocess
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PATH = 'scripts/validate_g1_traffic.py'
PARENT = 'config/tests/lidar_mount_launch_record_r5_sha256.json'
NAME = 'config/tests/lidar_mount_traffic_marker_r6_sha256.json'
PREVIOUS = 'config/tests/lidar_mount_trial_r1_sha256.json'


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    before = subprocess.run(['git', 'show', 'HEAD:' + PATH], cwd=ROOT, capture_output=True, check=True).stdout
    previous = json.loads((ROOT/PREVIOUS).read_text(encoding='utf-8'))
    assert sha(before) == previous['approved_evaluation_scripts'][PATH]['after']
    after = (ROOT/PATH).read_bytes()
    assert sha(after) != sha(before)
    snapshot = HERE/'traffic_marker_before.zip'
    if not snapshot.exists():
        with zipfile.ZipFile(snapshot, 'x', compression=zipfile.ZIP_DEFLATED) as archive:
            archive.writestr(PATH, before)
    with zipfile.ZipFile(snapshot) as archive:
        assert archive.read(PATH) == before
    contract = dict(
        schema_version=1,
        scope='JR60P20 임시 채택 후 교통 검사기의 opt-in 후면 ID49 높이 전달. 기본 인자·판정 불변, 주행 결과·실차 승인 아님',
        parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
        approved_changes={},
        approved_evaluation_scripts={PATH: dict(before=sha(before), after=sha(after),
            reason='단독 검사기와 같은 --pilot-marker-z(0.115..0.30 m, G1 계열) 전달과 센서 YAML/마커 높이 보고 기록. 미지정 시 기존 launch 인자 동일')},
        new_dependencies={})
    text = json.dumps(contract, ensure_ascii=False, indent=2) + '\n'
    target = ROOT/NAME
    if target.exists():
        assert target.read_text(encoding='utf-8') == text, 'existing r6 differs; do not rewrite silently'
    else:
        target.write_text(text, encoding='utf-8', newline='\n')
    print(NAME, contract['approved_evaluation_scripts'][PATH]['after'])


if __name__ == '__main__':
    main()
