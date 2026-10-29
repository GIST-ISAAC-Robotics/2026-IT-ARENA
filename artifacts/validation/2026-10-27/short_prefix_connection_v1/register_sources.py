"""변경 전 사본을 확인하고 선택형 연결의 출처 사슬을 한 단계 추가한다."""
import hashlib
import json
from pathlib import Path
import zipfile

ROOT=Path(__file__).resolve().parents[4]
BASE=Path(__file__).resolve().parent
def sha(data):return hashlib.sha256(data).hexdigest()

parent='config/tests/g1_course_evaluation_r0_sha256.json'
target=ROOT/'config/tests/short_boundary_prefix_r0_sha256.json'
before=json.loads((BASE/'before.json').read_text())
reasons={
 'src/arena_autonomy/arena_autonomy/local_follow.py':'기본 false 선택형 설정·제약과 짧은 관측 경로 선택 기록. 기존 기본 분기와 기억 갱신 정책 유지.',
 'src/arena_autonomy/arena_autonomy/corner_corridor.py':'양측 관측 경로가 일치하는 첫 연속 구간까지만 선호 경로를 자르는 기하 후보. 외삽·벽 정체 인증 없음.',
 'src/arena_autonomy/arena_autonomy/corridor_observer.py':'기존 시간 확인·점유·출력 제한에 짧은 경로 후보를 연결하고 현재 속도의 정지 예산도 검사.',
 'src/arena_autonomy/arena_autonomy/local_pursuit.py':'시뮬레이션·G1·판단 기록이 있을 때만 새 옵션 허용. 보호층 우회 없음.',
 'src/arena_bringup/launch/g1_follow.launch.py':'기본 false ROS 파라미터 전달.',
 'scripts/validate_g1_traffic.py':'명시 --short-boundary-prefix 실행 옵션만 추가. 판정·물성·기존 실행 기본값 유지.',
}
with zipfile.ZipFile(BASE/'before.zip') as archive:
    rows={}
    for path,reason in reasons.items():
        assert sha(archive.read(path))==before[path]
        rows[path]={'before':before[path],'after':sha((ROOT/path).read_bytes()),'reason':reason}
        assert rows[path]['before']!=rows[path]['after']
contract=dict(schema_version=1,scope='기본 false 짧은 관측 경로 연결의 소스 출처. 기본 활성화·실물 안전 승인 아님',
    parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
    before_snapshot=(BASE/'before.zip').relative_to(ROOT).as_posix(),
    before_snapshot_sha256=sha((BASE/'before.zip').read_bytes()),
    approved_changes={p:v for p,v in rows.items() if p.startswith('src/')},
    approved_evaluation_scripts={p:v for p,v in rows.items() if p.startswith('scripts/')},new_dependencies={})
with target.open('x',encoding='utf-8',newline='\n') as f:json.dump(contract,f,ensure_ascii=False,indent=2);f.write('\n')
for name in ('test_corner_follow.py','test_g1_dense_stop.py','test_physics_step_option.py','test_source_provenance_r0_plus_b3.py'):
    path=ROOT/'tests'/name
    text=path.read_text(encoding='utf-8')
    old="'g1_course_evaluation_r0_sha256.json')"
    if name=='test_source_provenance_r0_plus_b3.py':
        old="'config/tests/g1_course_evaluation_r0_sha256.json')"
        new="'config/tests/g1_course_evaluation_r0_sha256.json', 'config/tests/short_boundary_prefix_r0_sha256.json')"
    else:new="'g1_course_evaluation_r0_sha256.json', 'short_boundary_prefix_r0_sha256.json')"
    assert old in text,name
    path.write_text(text.replace(old,new),encoding='utf-8',newline='\n')
print('registered six modified files against preserved snapshot; prior contracts unchanged')
