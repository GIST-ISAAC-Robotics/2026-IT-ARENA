"""소스 계약의 누적 변경을 별도 후속으로 연결. 과거 계약/실행 결과는 수정하지 않는다."""
from pathlib import Path
import hashlib
import json
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
PARENT = 'config/tests/m2_motion_config_r0_sha256.json'
NEW = 'config/tests/m5_publication_r0_sha256.json'
sha = lambda b: hashlib.sha256(b).hexdigest()
changes = {
    'src/arena_autonomy/arena_autonomy/observed_lateral_trial.py': ('df27b78b67a51265f25b4587a5147398632dd23356b60eae06e9ecb0831ca7d5', '출력 보존 배열 질의 계산을 사용하도록 연결; 판정 기준 유지'),
    'scripts/validate_g1_traffic.py': ('04e6ac920ab7c0c2a8bbb60284eb6571de404517189cf77676573b6f1367695b', 'M5 기본 OFF 선택형 인자·금지 조합·출처를 물리 시험기에 연결'),
    'src/arena_autonomy/arena_autonomy/local_pursuit.py': ('404819816110c38ba51bd93dc222e50b5ce8b8dd568dd0e11616c292ad4bb9fe', 'M5 기본 OFF 원판 점유·가능한 양보 우선·중앙 복귀 보류를 노드 인자와 기록에 연결'),
    'src/arena_autonomy/arena_autonomy/m3_execution_trial.py': ('6cab5551ae61cb625d736798067001b3d90d20b97bc5d860fe28b24850a4105e', '원판 점유와 직선 비교 후보·부재 확인 중 직진을 명시 선택형으로 추가; 양보 우선 유지'),
    'src/arena_bringup/launch/g1_follow.launch.py': ('1a3782deaf67819930b43bba3241b48897a52c73a39d5af69471d9d71fdb2163', '새 시험 옵션의 기본 false 전달과 인자 유효성 연결'),
}
reasons = {
    'scripts/profile_follow_trace.py': '새 기록 모드와 정적 import 폐쇄에 필요한 7개 모듈 보존',
    'scripts/prepare_jetson_bundle.py': '명시 이관 파일 목록의 새 M3/스캔/배열 의존성 누락 보충',
    'scripts/validate_pipeline_faults_ros.py': '누적 운영 시험과 새 의존성·공식 10/24 입력 출처 연결',
}
contents, origins = {}, {}
# 이미 등록된 네 변경의 before는 실제 보관된 당시 바이트와 일치해야 한다.
for archive in sorted((ROOT/'artifacts/validation/2026-10-26').rglob('*.zip')):
    if archive.parent == OUT or not set(changes)-set(contents):
        continue
    with zipfile.ZipFile(archive) as z:
        for path in set(changes)-set(contents):
            if path in z.namelist() and sha(z.read(path)) == changes[path][0]:
                contents[path] = z.read(path)
                origins[path] = archive.relative_to(ROOT).as_posix()
assert set(changes) <= set(contents), sorted(set(changes)-set(contents))
for path, reason in reasons.items():
    data = subprocess.run(['git','show',f'4096314:{path}'],cwd=ROOT,capture_output=True,check=True).stdout
    contents[path] = data
    origins[path] = f'git:4096314:{path}'
    changes[path] = (sha(data), reason)
snapshot = OUT/'m5_publication_before.zip'
assert not snapshot.exists() and not (ROOT/NEW).exists()
with zipfile.ZipFile(snapshot,'w',zipfile.ZIP_DEFLATED) as z:
    for path in sorted(contents):
        z.writestr(path,contents[path])
contract = dict(schema_version=1,
    scope='10/26 M5 선택형 구현·기록/이관 의존성·공식 입력 갱신의 출처 연결. 기본 채택 및 주행 안전 승인 아님. before_snapshot은 파일별 보존 바이트를 모은 합성 자료로 단일 실행 시점이 아님',
    parent_contract=PARENT, parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
    before_snapshot=snapshot.relative_to(ROOT).as_posix(), before_snapshot_sha256=sha(snapshot.read_bytes()),
    before_origins=origins, approved_changes={},approved_evaluation_scripts={},new_dependencies={})
for path,(before,reason) in changes.items():
    after=sha((ROOT/path).read_bytes())
    assert before!=after
    group='approved_evaluation_scripts' if path.startswith('scripts/') else 'approved_changes'
    contract[group][path]=dict(before=before,after=after,reason=reason)
for name in ('polygon_batch','scan_topology','rounded_occupancy_candidate','startup_straight_candidate'):
    path=f'src/arena_autonomy/arena_autonomy/{name}.py'
    contract['new_dependencies'][path]=sha((ROOT/path).read_bytes())
(ROOT/NEW).write_text(json.dumps(contract,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
for path in ('tests/test_corner_follow.py','tests/test_g1_dense_stop.py','tests/test_physics_step_option.py','tests/test_source_provenance_r0_plus_b3.py'):
    p=ROOT/path;text=p.read_text(encoding='utf-8')
    old="'m2_motion_config_r0_sha256.json')"
    new="'m2_motion_config_r0_sha256.json', 'm5_publication_r0_sha256.json')"
    if path.endswith('test_source_provenance_r0_plus_b3.py'):
        old="'config/tests/m2_motion_config_r0_sha256.json')"
        new="'config/tests/m2_motion_config_r0_sha256.json', 'config/tests/m5_publication_r0_sha256.json')"
        text=text.replace('[10, 11, 12, 13, 14, 15, 16]', '[10, 11, 12, 13, 14, 15, 16, 17]')
    assert old in text,path
    p.write_text(text.replace(old,new),encoding='utf-8',newline='\n')
print(json.dumps(dict(registered=len(changes),before_origins=origins),ensure_ascii=False))
