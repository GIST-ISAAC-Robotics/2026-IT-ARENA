"""운영 묶음 소스 계약. 편집 전 바이트를 before.zip에 보존하고 현재 바이트와 연결한다. 재실행 금지."""
from hashlib import sha256
import json
from pathlib import Path
import zipfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BEFORE = HERE/'before'
PARENT = 'config/tests/official_traffic_eval_r2_sha256.json'
OUT = ROOT/'config/tests/official_ops_r1_sha256.json'
SNAPSHOT = HERE/'before.zip'
PKG = 'src/arena_autonomy/arena_autonomy/'
REASONS = {
    PKG+'race_session.py': '같은 경기 recovery_reset/resume_start/cancel_start/restore_race와 START 준비 상태. 기본 legacy 경로 불변.',
    PKG+'local_pursuit.py': 'B3 후보의 입력 경계, 정지 그림자 경로 확인, 마커 동결, 재획득 옵션 전달. 기본 legacy 경로 불변.',
    PKG+'wall_follow.py': 'B3 후보 상태에 입력 나이 표시 추가.',
    PKG+'vision_process.py': '같은 경기 입력 경계. 경계 전 취득 영상 폐기, 출발 판정 유지.',
    PKG+'lidar_motion_ros.py': '입력 경계에서 운동 이력 초기화와 경계 전 표본 거절.',
    PKG+'lidar_safety.py': 'B3 후보에서 입력 경계 수신 시 관측 기억만 초기화. 허가를 늘리지 않음.',
    PKG+'local_follow.py': '기본 OFF standing_wall_reacquire. 기억 없는 낮은 속도의 선호 벽 재획득.',
    'src/arena_autonomy/setup.py': 'race_ops 실행 파일 등록.',
    'src/arena_bringup/launch/g1_follow.launch.py': '기본 OFF g1_standing_wall_reacquire 전달.',
}
SCRIPTS = {'scripts/race_operator.py': 'CLI에 recovery-reset/resume-start 추가. 기존 명령 판정 불변.'}
NEW = ('race_ops_core.py', 'race_ops_export.py', 'race_ops_node.py', 'race_ops_page.py')


def digest(data):
    return sha256(data).hexdigest()


def before_bytes(path):
    backup = BEFORE/Path(path).name
    if backup.is_file():
        return backup.read_bytes()
    current = (ROOT/path).read_bytes()
    if path.endswith('local_follow.py'):
        text = current.replace(b'\r\n', b'\n').decode('utf-8')
        start = text.index('    standing_wall_reacquire: bool = False')
        text = text[:start]+text[text.index('\n', start)+1:]
        text = text.replace("""        if type(self.standing_wall_reacquire) is not bool:
            raise ValueError('standing wall reacquire must be bool')
""", '').replace("if k not in ('standing_wall_reacquire', 'static_side_release_trial',", "if k not in ('static_side_release_trial',")
        start = text.index('                    if not eligible and cfg.standing_wall_reacquire')
        end = text.index("                        detail['bilateral_reference'] = 'standing_preferred_wall'\n")
        end = text.index('\n', end)+1
        return (text[:start]+text[end:]).encode('utf-8').replace(b'\n', b'\r\n')
    if path.endswith('g1_follow.launch.py'):
        text = current.decode('utf-8')
        start = text.index("        DeclareLaunchArgument('g1_standing_wall_reacquire'")
        end = text.index("        SetParameter(name='g1_standing_wall_reacquire'")
        end = text.index('\n', end)+1
        return (text[:start]+text[end:]).encode('utf-8')
    raise FileNotFoundError(path)


def main():
    if OUT.exists() or SNAPSHOT.exists():
        raise FileExistsError('기존 계약')
    rows = {}
    with zipfile.ZipFile(SNAPSHOT, 'x', zipfile.ZIP_DEFLATED) as bundle:
        for path in [*REASONS, *SCRIPTS]:
            old = before_bytes(path)
            bundle.writestr(path, old)
            rows[path] = dict(before=digest(old), after=digest((ROOT/path).read_bytes()),
                              reason=REASONS.get(path) or SCRIPTS[path])
    contract = dict(schema_version=1, scope='C 경기 운영 최소 흐름(11/2). 기본 OFF 후보이며 실차·공용망 검증 아님.',
                    parent_contract=PARENT, parent_contract_sha256=digest((ROOT/PARENT).read_bytes()),
                    before_snapshot=SNAPSHOT.relative_to(ROOT).as_posix(), before_snapshot_sha256=digest(SNAPSHOT.read_bytes()),
                    approved_changes={k: v for k, v in rows.items() if k in REASONS},
                    approved_evaluation_scripts={k: v for k, v in rows.items() if k in SCRIPTS},
                    new_dependencies={PKG+name: digest((ROOT/PKG/name).read_bytes()) for name in NEW})
    OUT.write_bytes((json.dumps(contract, ensure_ascii=False, indent=2)+'\n').encode('utf-8'))
    print(OUT.name, len(rows))


if __name__ == '__main__':
    main()
