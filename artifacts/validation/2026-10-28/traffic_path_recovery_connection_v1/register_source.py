"""선택형 경로 회복 연결의 부모 계약과 변경 전후 바이트를 등록한다."""
import hashlib
import json
from pathlib import Path
import zipfile
BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
PARENT='config/tests/traffic_recovery_release_fringe_r1_sha256.json'
TARGET='config/tests/traffic_path_recovery_r0_sha256.json'
sha=lambda data:hashlib.sha256(data).hexdigest()
if (ROOT/TARGET).exists():raise FileExistsError(TARGET)
previous=json.loads((BASE/'before_sha256.json').read_text())
groups={'approved_changes':{},'approved_evaluation_scripts':{}}
with zipfile.ZipFile(BASE/'before.zip') as archive:
    for path,before in previous.items():
        assert sha(archive.read(path))==before
        after=sha((ROOT/path).read_bytes())
        assert before!=after,path
        group='approved_changes' if path.startswith('src/') else 'approved_evaluation_scripts'
        groups[group][path]=dict(before=before,after=after,reason=(
            '기본false 경로 회복 옵션·역할 기하·통과 이력 선택기 연결. 기존 경계 기억/보호/물성 유지.'
            if group=='approved_changes' else
            '명시 시험 인자·선행 검증·소스 의존성 보존. 기존 실행/판정 기준 유지.'))
new={f'src/arena_autonomy/arena_autonomy/{n}.py':sha((ROOT/f'src/arena_autonomy/arena_autonomy/{n}.py').read_bytes())
     for n in ('traffic_path_recovery_trial','corridor_roles')}
report=dict(schema_version=1,
    scope='경로 재획득과 완료 통과 이력을 분리한 선택형 연결 출처. 기본 채택/실물 안전 승인 아님',
    parent_contract=PARENT,parent_contract_sha256=sha((ROOT/PARENT).read_bytes()),
    before_snapshot=(BASE/'before.zip').relative_to(ROOT).as_posix(),
    before_snapshot_sha256=sha((BASE/'before.zip').read_bytes()),**groups,new_dependencies=new)
(ROOT/TARGET).write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
print(json.dumps(dict(contract=TARGET,changed=len(previous),dependencies=len(new))))

for name in ('test_source_provenance_r0_plus_b3.py','test_corner_follow.py',
             'test_g1_dense_stop.py','test_physics_step_option.py'):
    path=ROOT/'tests'/name
    content=path.read_text(encoding='utf-8')
    prefix='config/tests/' if name=='test_source_provenance_r0_plus_b3.py' else ''
    old="'"+prefix+"traffic_recovery_release_fringe_r1_sha256.json')"
    new="'"+prefix+"traffic_recovery_release_fringe_r1_sha256.json', '"+prefix+"traffic_path_recovery_r0_sha256.json')"
    expected=2 if name=='test_physics_step_option.py' else 1
    assert content.count(old)==expected,(name,content.count(old))
    path.write_text(content.replace(old,new),encoding='utf-8')
path=ROOT/'tests/source_contract_helpers.py'
content=path.read_text(encoding='utf-8')
old="for name in ('traffic_recovery_release_fringe_r1_sha256.json',"
assert content.count(old)==1
path.write_text(content.replace(old,"for name in ('traffic_path_recovery_r0_sha256.json', 'traffic_recovery_release_fringe_r1_sha256.json',"),encoding='utf-8')
