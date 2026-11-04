"""공식 교통 경로와 관측 이동 후보의 소스 계약. 이전 계약과 원시 실행은 다시 쓰지 않는다."""
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PARENT = 'config/tests/official_start_region_r1_sha256.json'
OUT = ROOT/'config/tests/official_forward_r1_sha256.json'
REASON = '공식 월드 교통 경로 명시 선택과 기본 OFF 관측 이동 후보. 세계 고정 측방 반환 해제와 전진 시작 조각의 우회 진입 차단. 점유·보호층·물성·판정선 유지.'
CHANGES = ('src/arena_autonomy/arena_autonomy/bypass_handoff.py', 'src/arena_autonomy/arena_autonomy/g1_g2_node.py',
           'src/arena_autonomy/arena_autonomy/local_follow.py', 'src/arena_autonomy/arena_autonomy/pass_release_trial.py',
           'src/arena_bringup/launch/simulation.launch.py')
SCRIPTS = ('scripts/g1_handoff_eval.py', 'scripts/g1_traffic_eval.py', 'scripts/traffic_merge.py', 'scripts/validate_g1_traffic.py')
NEW = ('scripts/official_traffic_course.py', 'src/arena_autonomy/arena_autonomy/static_side_return.py')
ROI = HERE.parent/'official_start_roi_v1/before.zip'


def digest(data):
    return sha256(data).hexdigest()


def before_bytes(path):
    """직전 후속 계약의 after 바이트. launch는 그 계약 때 복원한 CRLF, 나머지는 HEAD와 같다."""
    if path.endswith('simulation.launch.py'):
        with zipfile.ZipFile(ROI) as archive:
            return archive.read(path).replace(b'\n', b'\r\n')
    return subprocess.run(['git', 'show', f'HEAD:{path}'], cwd=ROOT, check=True, stdout=subprocess.PIPE).stdout


def main():
    before = HERE/'before.zip'
    if not before.exists():
        with zipfile.ZipFile(before, 'x', zipfile.ZIP_DEFLATED) as archive:
            for path in (*CHANGES, *SCRIPTS):
                archive.writestr(zipfile.ZipInfo(path, (2026, 11, 2, 0, 0, 0)), before_bytes(path))
    with zipfile.ZipFile(before) as archive:
        row = lambda path: dict(before=digest(archive.read(path)), after=digest((ROOT/path).read_bytes()), reason=REASON)
        contract = dict(schema_version=1, scope=REASON, parent_contract=PARENT,
                        parent_contract_sha256=digest((ROOT/PARENT).read_bytes()),
                        before_snapshot=before.relative_to(ROOT).as_posix(), before_snapshot_sha256=digest(before.read_bytes()),
                        approved_changes={p: row(p) for p in CHANGES}, approved_evaluation_scripts={p: row(p) for p in SCRIPTS},
                        new_dependencies={p: digest((ROOT/p).read_bytes()) for p in NEW})
    OUT.write_bytes((json.dumps(contract, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))


if __name__ == '__main__':
    main()
