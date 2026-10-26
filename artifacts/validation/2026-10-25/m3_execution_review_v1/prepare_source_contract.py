"""새 M3 선택형 연결의 출처 기록. 시험을 실행하거나 과거 계약을 수정하지 않는다."""
import hashlib
import json
from pathlib import Path
import tarfile

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
TARGET = ROOT/'config/tests/m3_execution_r0_sha256.json'
PARENT = 'config/tests/b3_race_session_r0_sha256.json'
BEFORE = 'artifacts/validation/2026-10-24/b3_race_session_ros_v2/source_snapshot_before.tar.gz'
CHANGES = {
    'src/arena_autonomy/arena_autonomy/local_pursuit.py': '기본 false M3 선택·시뮬 시간/기록 필수·타입별 오류 모드. 독립 보호 출력 경계 유지',
    'src/arena_autonomy/arena_autonomy/follow_trace.py': 'M3 schema 3 및 실행 설정 기록/동일 재생. 기존 G1/R0+ schema 유지',
    'src/arena_bringup/launch/g1_follow.launch.py': '기본 false M3 시뮬레이션 파라미터만 추가',
    'scripts/validate_g1_traffic.py': '명시 m3 평가 모드·실제 소스 보존·기존 행동/정지 판정 유지',
    'scripts/g1_traffic_eval.py': 'M3 모드 일치 및 행동 기준 적용, R0 비교용 무결성 통과와 분리',
}
DEPENDENCIES = ('m3_execution_trial', 'm3_observed_follow', 'lateral_intent', 'observed_lateral_trial')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    with tarfile.open(ROOT/BEFORE) as archive:
        changes = {p: {'before': sha(archive.extractfile(p).read()), 'after': sha((ROOT/p).read_bytes()),
                       'reason': reason} for p, reason in CHANGES.items()}
    parent = json.loads((ROOT/PARENT).read_text())
    assert changes['src/arena_autonomy/arena_autonomy/local_pursuit.py']['before'] == parent['approved_changes'][
        'src/arena_autonomy/arena_autonomy/local_pursuit.py']['after']
    result = dict(schema_version=1,
        scope='2026-10-25 M3 기본 false 시뮬레이션 연결의 출처 기록. 기본 채택·주행 안전 승인 아님',
        parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=BEFORE, before_snapshot_sha256=sha((ROOT/BEFORE).read_bytes()), changes=changes,
        approved_changes={p: row for p, row in changes.items() if p.startswith('src/')},
        approved_evaluation_scripts={p: row for p, row in changes.items() if p.startswith('scripts/')},
        dependencies={f'src/arena_autonomy/arena_autonomy/{n}.py': sha((ROOT/f'src/arena_autonomy/arena_autonomy/{n}.py').read_bytes())
                      for n in DEPENDENCIES})
    TARGET.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
    print('prepared', TARGET.relative_to(ROOT), len(changes), 'changed files')


if __name__ == '__main__':
    main()
