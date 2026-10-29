"""원래 실패 주행의 소스를 기준으로 후속 변경 사슬만 추가한다."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
parent = ROOT / 'config/tests/short_prefix_origin_r1_sha256.json'
snapshot = BASE / 'run1_course_sources.zip'
target = ROOT / 'config/tests/rotation_reobservation_r1_sha256.json'
sha = lambda data: hashlib.sha256(data).hexdigest()
changes = {
    'corner_corridor': '실제 안쪽 지지가 짧은 코너는 관측된 첫 경로만 후보로 반환. 원래 실패와 속도/정지 예산을 보존.',
    'corridor_observer': '기존 short_boundary_prefix 옵션의 짧은 안쪽 지지를 연결하고 실제 관측 길이로 정지 예산을 제한.',
    'surface_anchor_trial': '기본 false의 extended_surface 선택을 추가. 정합 승인과 두 새 스캔 및 모든 점 확인은 유지.',
    'traffic_recovery_trial': '명시 교통 회복 옵션에서 네 연속 반환 표면 지지를 사용. 현재 입력 재검사·독립 상태 유지.',
}
scripts = ('prepare_jetson_bundle', 'profile_follow_trace', 'validate_g1_traffic',
           'validate_pipeline_faults_ros', 'validate_sensor_input_ros')
result = dict(schema_version=1,
    scope='회전 오차 run1 원본 이후 짧은 관측 경로와 선택형 재관측 보완의 소스 출처. 과거 판정/기본 설정/실차 승인 변경 아님.',
    parent_contract=parent.relative_to(ROOT).as_posix(), parent_contract_sha256=sha(parent.read_bytes()),
    before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
    approved_changes={}, approved_evaluation_scripts={}, new_dependencies={})
with zipfile.ZipFile(snapshot) as archive:
    for name, reason in changes.items():
        path = 'src/arena_autonomy/arena_autonomy/' + name + '.py'
        before, after = sha(archive.read(path)), sha((ROOT/path).read_bytes())
        assert before != after
        result['approved_changes'][path] = dict(before=before, after=after, reason=reason)
    for name in scripts:
        path = 'scripts/' + name + '.py'
        before, after = sha(archive.read(path)), sha((ROOT/path).read_bytes())
        assert before != after
        result['approved_evaluation_scripts'][path] = dict(before=before, after=after,
            reason='surface_reobservation 내부 의존성을 배포/소스 보존 목록에 추가. 제어·판정 변경 없음.')
path = 'src/arena_autonomy/arena_autonomy/surface_reobservation.py'
result['new_dependencies'][path] = sha((ROOT/path).read_bytes())
with target.open('x', encoding='utf-8', newline='\n') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2)
    stream.write('\n')
print(target.name)
