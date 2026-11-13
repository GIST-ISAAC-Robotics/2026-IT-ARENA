"""r437 준비 도구를 새 경로에 복사한다. 빌드/ROS/주행은 실행하지 않는다."""
from pathlib import Path
import difflib
import hashlib
import importlib.util
import json
import shutil
import zipfile

A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'applied_limit_drive_r437_v1'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_bytes())
def write(path,value):path.write_bytes((json.dumps(value,ensure_ascii=False,indent=2)+'\n').encode())
def source(name):return (BASE/name).read_text(encoding='utf-8')
def save(name,text):(A/name).write_bytes(text.encode('utf-8'))
assert not (A/'prepare.py').exists()
for name in ('actors.json','expected_header.json','facility_geometry.py','header_comparison.py','launch_adapter.py',
    'course.launch.py','test_preflight.py','test_launch_selection.py','.gitignore'):
    shutil.copyfile(BASE/name,A/name)
shutil.copytree(BASE/'launch',A/'launch')
config=read(BASE/'config.json');config['scope']='r441 동일 조건에서 crop proof 성공 후 자체 후측방 mask의 단일0.1m/s current 재검사 주행1회'
config['launch']['entry_g1']=str((A/'course.launch.py').relative_to(ROOT)).replace('\\','/')
write(A/'config.json',config)
prepare=source('prepare.py')
prepare=prepare.replace('r431 대비 승인된 r429/r430/r436 공통14파일과 새 빌드를 확인해 동결한다.','r437 대비 승인된 r434/r440 공통5파일과 새 빌드를 확인해 동결한다.')
prepare=prepare.replace("BASE=A.parent/'crop_reach_drive_r431_v1'","BASE=A.parent/'applied_limit_drive_r437_v1'")
prepare=prepare.replace('assert len(files)==14','assert len(files)==5')
prepare=prepare.replace("scope['r436_ros_constructor_result']","scope['r440_validation_result']")
prepare=prepare.replace('only exact r431-relative fourteen approved common files permitted','only exact r437-relative five approved common files permitted')
prepare=prepare.replace('r431 대비 r430 네 파일/r429 여덟 파일/r436 두 파일만 허용.','r437 대비 r434 네 파일/r440 한 파일만 허용.')
prepare=prepare.replace(".replace('applied_limit_drive_r437_v1/run.py','crop_reach_drive_r431_v1/run.py').replace(\"Node('r437_owned_stop')\",\"Node('r431_owned_stop')\")",
    ".replace('crop_crawl_drive_r441_v1/run.py','applied_limit_drive_r437_v1/run.py').replace(\"Node('r441_owned_stop')\",\"Node('r437_owned_stop')\")")
prepare=prepare.replace('r431_source_archive_sha256','r437_source_archive_sha256')
save('prepare.py',prepare)
run=source('run.py').replace('r431 조건·선택을 유지하고 검토된 r429/r430/r436 통합 뒤 조향 연결을 확인하는 단1회.',
    'r437 조건·선택을 유지하고 crop 단일0.1 current 후속의 실제 발행/진행을 확인하는 단1회.')
save('run.py',run.replace("A.parent/'crop_reach_drive_r431_v1'","A.parent/'applied_limit_drive_r437_v1'"))
save('stop_owned.py',source('stop_owned.py').replace('applied_limit_drive_r437_v1/run.py','crop_crawl_drive_r441_v1/run.py').replace("Node('r437_owned_stop')","Node('r441_owned_stop')"))
audit=source('audit_runtime.py').replace('def reach_summary(rows):','def reach_summary(rows, *, crawl_only=False):')
audit=audit.replace('trial=current.get(REACH_KEY)\n            if trial:',"trial=current.get(REACH_KEY)\n            if crawl_only:trial=(trial or {}).get('crawl_retry')\n            if trial:")
audit=audit.replace("tried+=1;reasons[trial.get('reason','missing')]+=1\n                proof_reasons[(trial.get('proof') or {}).get('reason','not_reached')]+=1",
    "detail=trial.get('result',trial)\n                tried+=1;reasons[detail.get('reason','missing')]+=1\n                proof_reasons[(detail.get('proof') or {}).get('reason','not_reached')]+=1")
audit=audit.replace('current_crop_reach=reach_summary(rows),','current_crop_reach=reach_summary(rows),\n        current_crop_crawl=reach_summary(rows,crawl_only=True),')
save('audit_runtime.py',audit)
with (A/'test_preflight.py').open('a',encoding='utf-8') as stream:stream.write('''
    assert reach_summary(rows,crawl_only=True)['calculations_accepted']==0
    current[REACH_KEY]['crawl_retry']=dict(applied=True,result=dict(reason='single_current_crop_reach_accepted',proof=dict(reason='bounded_same_crop_current_reach_risk')))
    assert reach_summary(rows,crawl_only=True)['calculations_accepted']==1
    assert reach_summary(rows,crawl_only=True)['adjacent_matching_parent_publications']==0
    assert reach_summary(rows+[publication],crawl_only=True)['adjacent_matching_parent_publications']==1
    assert reach_summary(rows+[dict(kind='reset',data={}),publication],crawl_only=True)['adjacent_matching_parent_publications']==0
''')
spec=importlib.util.spec_from_file_location('r437_prepare_inventory',BASE/'prepare.py');m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)
current=m.common_sources();old={p:h for p,h in read(BASE/'prepared.json')['frozen_inputs'].items() if p.startswith(('src/','scripts/','tests/'))}
r434=A.parent/'operator_preview_r434_v1/integration.json';r440=A.parent/'crop_current_crawl_r440_v1/handoff_audit.json'
expected={row['path']:row['staged_sha256'] for row in read(r434)['files']}
expected['src/arena_autonomy/arena_autonomy/race_runtime_policy/crop_current_candidate.py']='886f4875a11cc5f7cc0af4924a99717a323464542ffb4fa48e45f218fe01d9e5'
actual={p:current.get(p) for p in old.keys()|current.keys() if old.get(p)!=current.get(p)}
assert actual==expected,(actual,expected)
files={p:dict(before_sha256=old.get(p),after_sha256=h,kind='changed' if p in old else 'added_to_snapshot',
    group='r440' if 'crop_current_candidate' in p else 'r434',
    reason='검증된 단일0.1 후속. r440 35검사/409 저장 비교 통과.' if 'crop_current_candidate' in p else '기본OFF 선택형 ROS 미리보기. 이번 주행에서 패널/preview를 활성화하지 않는다.') for p,h in sorted(expected.items())}
scope=dict(status='root_authorized_scope_agent_hash_verified',baseline=BASE.relative_to(ROOT).as_posix(),
    baseline_archive_sha256=sha(BASE/'sources.zip'),integrations=dict(r434=dict(path=r434.relative_to(ROOT).as_posix(),sha256=sha(r434))),
    r440_validation_result=dict(path=r440.relative_to(ROOT).as_posix(),sha256=sha(r440)),files=files,
    r439_not_integrated=True,preview_default_off=True,execution_not_authorized_yet=True)
write(A/'source_scope.json',scope)
patch=[]
with zipfile.ZipFile(BASE/'sources.zip') as z:
    for name in sorted(expected):
        before=z.read(name).decode('utf-8-sig') if name in z.namelist() else ''
        after=(ROOT/name).read_text(encoding='utf-8-sig')
        patch.extend(difflib.unified_diff(before.splitlines(True),after.splitlines(True),fromfile='r437/'+name,tofile='r441/'+name))
save('source_delta.patch',''.join(patch))
(A/'build_v1').mkdir()
for name in ('build.sh','import_paths.py'):
    save('build_v1/'+name,source('build_v1/'+name).replace('applied_limit_drive_r437_v1','crop_crawl_drive_r441_v1'))
with (A/'build_v1/import_paths.py').open('a',encoding='utf-8') as stream:stream.write("\nfrom arena_autonomy.race_ops_preview import Preview\np=Preview()\nassert p.worker is None and not p.subscriptions and not p.desired['lidar'] and not p.desired['camera']\n")
(A/'preflight_v1').mkdir()
save('preflight_v1/preflight.sh',source('preflight_v1/preflight.sh').replace('applied_limit_drive_r437_v1','crop_crawl_drive_r441_v1'))
print(json.dumps(dict(common_files=len(current),exact_delta=len(actual),source_scope_sha256=sha(A/'source_scope.json'))))
