"""observed_measured_v1 연결의 후속 출처 계약을 새 파일로 만든다. 이전 계약/스냅샷은 덮어쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
TARGET = ROOT/'config/tests/measured_lane_preference_r1_sha256.json'
PARENT = 'config/tests/observed_lane_preference_r1_sha256.json'
SNAPSHOT = BASE/'before_connection.zip'
V4 = 'artifacts/validation/2026-10-29/rear_lane_preference_shadow_v4'
REASON_SRC = '명시 rear_lane_keep_policy=observed_measured_v1 연결(기본 timed/OFF, 기존 observed 의미 불변)'
REASON_TOOL = '새 선택기·평행 후보·속도 제안 모듈의 보존/이관/재생 의존성 추가'
CHANGES = {'src/arena_autonomy/arena_autonomy/local_follow.py': REASON_SRC}
TOOLS = {f'scripts/{name}.py': REASON_TOOL for name in
         ('validate_pipeline_faults_ros', 'prepare_jetson_bundle', 'profile_follow_trace', 'validate_g1_traffic')}
NEW = ('measured_lane_preference_trial', 'measured_parallel_speed_trial', 'measured_parallel_trial')


def sha(data):
    return hashlib.sha256(data).hexdigest()


def main():
    with zipfile.ZipFile(SNAPSHOT) as archive:
        def rows(table):
            result = {}
            for path, reason in table.items():
                before, after = sha(archive.read(path)), sha((ROOT/path).read_bytes())
                assert before != after, path
                result[path] = dict(before=before, after=after, reason=reason)
            return result
        changes, tools = rows(CHANGES), rows(TOOLS)
    contract = dict(
        schema_version=1,
        scope=('10/29 오프라인 v4(측정속도 비클리핑·자체 정지공간/관측 영역 검사)를 목표 상한 경계 수정 후 '
               '별도 명시 정책 observed_measured_v1로 연결. 기본값/기존 timed·observed 불변. 운영 채택·실차 안전 승인 아님.'),
        parent_contract=PARENT,
        parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
        before_snapshot=SNAPSHOT.relative_to(ROOT).as_posix(),
        before_snapshot_sha256=sha(SNAPSHOT.read_bytes()),
        before_source=dict(scope='v3 observed 연결·Gazebo v1 이후 미커밋 상태의 연결 직전 바이트'),
        approved_changes=changes,
        approved_evaluation_scripts=tools,
        new_dependencies={f'src/arena_autonomy/arena_autonomy/{name}.py':
                          sha((ROOT/f'src/arena_autonomy/arena_autonomy/{name}.py').read_bytes()) for name in NEW},
        offline_origin={f'{V4}/{name}': sha((ROOT/V4/name).read_bytes())
                        for name in ('candidate.py', 'parallel_speed.py', 'parallel_state.py')},
        offline_origin_archive=dict(path=f'{V4}/offline_sources2.zip',
                                    sha256=sha((ROOT/V4/'offline_sources2.zip').read_bytes())),
        successor_fix='measured_parallel_speed_trial.proposal: target_cap 유효성(유한 양수)과 설정 최대속도 min 결합. '
                      'v4 원본은 max_speed_mps<.35에서 기본 .35를 invalid_target_cap으로 거부했다.')
    with TARGET.open('x', encoding='utf-8', newline='\n') as stream:
        stream.write(json.dumps(contract, ensure_ascii=False, indent=2)+'\n')
    print(json.dumps(dict(target=TARGET.relative_to(ROOT).as_posix(), changes=len(changes), tools=len(tools))))


if __name__ == '__main__':
    main()
