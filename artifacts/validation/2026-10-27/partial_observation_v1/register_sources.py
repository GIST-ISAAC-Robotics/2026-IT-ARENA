"""10/27 새 선택형 정책의 출처를 이전 main과 연결한다. 과거 계약은 수정하지 않는다."""
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).parent
def sha(data):return hashlib.sha256(data).hexdigest()
files={
 'src/arena_autonomy/arena_autonomy/lidar_safety.py':'기본 strict 보존, bounded_side_v1 독립 보호 선택형 정책',
 'src/arena_bringup/launch/demo.launch.py':'기본 strict 정책 인자 전달',
 'src/arena_bringup/launch/local_pursuit.launch.py':'단독 실행 진입점의 정책 인자 전달',
 'scripts/validate_basic_autonomy.py':'후보 명시·소스 기록·R0 밀집 STOP 및 실패 시 STOP 회수',
 'scripts/validate_sensor_input_ros.py':'선택형 정책 ROS 입출력 고장 대조',
}
snapshot=OUT/'before_main.zip'
rows={}
with zipfile.ZipFile(snapshot,'x',zipfile.ZIP_DEFLATED) as archive:
    for path,reason in files.items():
        original=subprocess.run(['git','show','0e6c28a:'+path],cwd=ROOT,capture_output=True,check=True).stdout
        archive.writestr(path,original)
        rows[path]=dict(before=sha(original),after=sha((ROOT/path).read_bytes()),reason=reason)
parent='config/tests/m5_publication_r0_sha256.json'
result=dict(schema_version=1,scope='10/27 ADR0027 선택형 부분 관측 정책 출처. 기본 채택/실차 안전 승인 아님',
 parent_contract=parent,parent_contract_sha256=sha((ROOT/parent).read_bytes()),
 before_snapshot=snapshot.relative_to(ROOT).as_posix(),before_snapshot_sha256=sha(snapshot.read_bytes()),
 before_source='git:0e6c28a',approved_changes={p:r for p,r in rows.items() if p.startswith('src/')},
 approved_evaluation_scripts={p:r for p,r in rows.items() if p.startswith('scripts/')},
 dependencies={p:sha((ROOT/p).read_bytes()) for p in ('src/arena_autonomy/arena_autonomy/partial_observation.py',)})
destination=ROOT/'config/tests/partial_observation_r0_sha256.json'
with destination.open('x',encoding='utf-8') as stream:stream.write(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
for name in ('test_corner_follow.py','test_g1_dense_stop.py','test_physics_step_option.py'):
    path=ROOT/'tests'/name
    content=path.read_text()
    content=content.replace("'m5_publication_r0_sha256.json')", "'m5_publication_r0_sha256.json', 'partial_observation_r0_sha256.json')")
    path.write_text(content)
path=ROOT/'tests/test_source_provenance_r0_plus_b3.py'
content=path.read_text().replace("'config/tests/m5_publication_r0_sha256.json')",
 "'config/tests/m5_publication_r0_sha256.json', 'config/tests/partial_observation_r0_sha256.json')")
path.write_text(content)
print(json.dumps({'registered':list(rows),'before_snapshot':str(snapshot)}))
