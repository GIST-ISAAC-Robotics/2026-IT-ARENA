"""공식 시설 연결의 소스 계약. 수정 전 바이트는 HEAD(59c3c3b0)에서 묶고 기존 계약은 다시 쓰지 않는다."""
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[4]
HERE = Path(__file__).resolve().parent
BASE = '59c3c3b0'
PARENT = 'config/tests/boundary_bridge_r1_sha256.json'
OUT = ROOT/'config/tests/official_facility_r1_sha256.json'
REASON = '공식 v2026.11.01 파생 월드와 2×5 광원 재질 제어 연결. 실제 RGB 출발 경계, 보호층, 물성, 주행 판단은 그대로다.'
CHANGES = ('src/arena_bringup/launch/simulation.launch.py', 'src/arena_gazebo/scripts/traffic_light_controller.py')
SCRIPTS = ('scripts/build_official_track.py', 'scripts/prelap_signal_eval.py', 'scripts/validate_basic_autonomy.py')
NEW = ('scripts/official_2x5_facilities.py', 'src/arena_gazebo/src/visual_material_client.cpp')


def digest(data):
    return sha256(data).hexdigest()


def old(path):
    return subprocess.run(['git', 'show', f'{BASE}:{path}'], cwd=ROOT, check=True, stdout=subprocess.PIPE).stdout


def row(path):
    return dict(before=digest(old(path)), after=digest((ROOT/path).read_bytes()), reason=REASON)


def main():
    before = HERE/'before.zip'
    with zipfile.ZipFile(before, 'w', zipfile.ZIP_DEFLATED) as archive:
        for path in (*CHANGES, *SCRIPTS):
            info = zipfile.ZipInfo(path, (2026, 11, 2, 0, 0, 0))
            archive.writestr(info, old(path))
    contract = dict(schema_version=1, scope=REASON, parent_contract=PARENT,
                    parent_contract_sha256=digest((ROOT/PARENT).read_bytes()),
                    before_snapshot=before.relative_to(ROOT).as_posix(),
                    before_snapshot_sha256=digest(before.read_bytes()),
                    approved_changes={path: row(path) for path in CHANGES},
                    approved_evaluation_scripts={path: row(path) for path in SCRIPTS},
                    new_dependencies={path: digest((ROOT/path).read_bytes()) for path in NEW})
    OUT.write_bytes((json.dumps(contract, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))


if __name__ == '__main__':
    main()
