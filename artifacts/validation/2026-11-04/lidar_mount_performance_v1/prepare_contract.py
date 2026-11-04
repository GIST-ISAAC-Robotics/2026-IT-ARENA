"""시험 전용 C1 장착 구조 변경의 후속 출처 계약. 이전 계약은 다시 쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PARENT = 'config/tests/current_lane_preference_r1_sha256.json'
TARGET = ROOT/'config/tests/lidar_mount_trial_r1_sha256.json'
REASONS = {
    'src/arena_autonomy/arena_autonomy/local_scene.py': '후방 반각 실수 외에 시험 전용 장착 구조 각도(sectors_deg)를 masked로 처리. 실수 입력 동작 동일',
    'src/arena_autonomy/arena_autonomy/local_path.py': 'mask_scan이 같은 구조 각도를 NaN(unknown)으로 둔다. 실수 입력 동작 동일',
    'src/arena_autonomy/arena_autonomy/local_pursuit.py': "lidar_blind_sectors_json 파라미터(기본 '')로 구조 각도 선택",
    'src/arena_autonomy/arena_autonomy/lidar_safety.py': "독립 보호층도 같은 lidar_blind_sectors_json(기본 '')을 받는다",
    'src/arena_bringup/launch/simulation.launch.py': '센서 YAML에 시험 키가 있을 때만 지주 표시 형상·차체 표시 상자 높이·구조 각도 계산. 기본 YAML 불변',
}
SCRIPTS = {
    'scripts/validate_basic_autonomy.py': '--local-sensor-config 명시 선택 전달. 기본 미지정',
    'scripts/validate_g1_traffic.py': '--local-sensor-config 명시 선택 전달. 기본 미지정',
}


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    snapshot = HERE/'runtime_before.zip'
    rows = {}
    with zipfile.ZipFile(snapshot) as archive:
        for group in (REASONS, SCRIPTS):
            for path, reason in group.items():
                rows[path] = dict(before=sha(archive.read(path)), after=sha((ROOT/path).read_bytes()), reason=reason)
    contract = dict(schema_version=1,
        scope='C1 장착(JR/C) 주행 비교의 시험 전용 지주 자기반환·알려진 구조 unknown 선택. 기본 채택/실차 승인 아님.',
        parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
        approved_changes={p: rows[p] for p in REASONS}, approved_evaluation_scripts={p: rows[p] for p in SCRIPTS},
        new_dependencies={})
    text = json.dumps(contract, ensure_ascii=False, indent=2) + '\n'
    if TARGET.exists():
        assert TARGET.read_text(encoding='utf-8') == text, '등록 후 소스가 바뀌었다'
    else:
        TARGET.write_bytes(text.encode('utf-8'))
    print(TARGET.relative_to(ROOT).as_posix(), sha(TARGET.read_bytes()))


if __name__ == '__main__':
    main()
