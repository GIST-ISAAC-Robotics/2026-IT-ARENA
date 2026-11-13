"""r441 입력/평가 기준을 새 경로에 보존한다. 빌드/주행은 실행하지 않는다."""
from pathlib import Path
import json,shutil
A=Path(__file__).resolve().parent;ROOT=A.parents[3];BASE=A.parent/'crop_crawl_drive_r441_v1'
def source(name):return (BASE/name).read_text(encoding='utf8')
def save(name,text):
    p=A/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text(text,encoding='utf8',newline='\n')
assert not (A/'prepare.py').exists()
for name in ('actors.json','expected_header.json','facility_geometry.py','header_comparison.py','course.launch.py',
             'test_preflight.py','test_launch_selection.py','audit_runtime.py','.gitignore'):
    shutil.copyfile(BASE/name,A/name)
shutil.copytree(BASE/'launch',A/'launch')
config=json.loads((BASE/'config.json').read_bytes())
config['scope']='r452 r441 동일 조건 GUARD 단일0.1 재검사 제한 주행1회; 완전 plan trace 진단 ON'
config['launch']['entry_g1']=(A/'course.launch.py').relative_to(ROOT).as_posix()
save('config.json',json.dumps(config,ensure_ascii=False,indent=2)+'\n')
save('launch_adapter.py',source('launch_adapter.py').replace("    'g1_crop_edge_component_trial:=true',",
    "    'g1_crop_edge_component_trial:=true',\n    'g1_complete_plan_trace:=true',"))
run=source('run.py').replace('r437 조건·선택을 유지하고 crop 단일0.1 current 후속의 실제 발행/진행을 확인하는 단1회.',
    'r441 조건을 유지하고 GUARD 단일0.1 재검사와 완전 plan 기록을 확인하는 단1회.')
run=run.replace("A.parent/'applied_limit_drive_r437_v1'","A.parent/'crop_crawl_drive_r441_v1'")
run=run.replace('crop_edge_component_trial=True,','crop_edge_component_trial=True, complete_plan_trace=True,')
save('run.py',run)
save('stop_owned.py',source('stop_owned.py').replace('crop_crawl_drive_r441_v1/run.py','guard_crawl_drive_r452_v1/run.py')
    .replace("Node('r441_owned_stop')","Node('r452_owned_stop')"))
p=source('prepare.py')
p=p.replace('r437 대비 승인된 r434/r440 공통5파일과 새 빌드를 확인해 동결한다.','r441 대비 실제 전체 source delta와 새 빌드/root 검토를 확인해 동결한다.')
p=p.replace("BASE=A.parent/'applied_limit_drive_r437_v1'","BASE=A.parent/'crop_crawl_drive_r441_v1'")
start=p.index('def checked_delta(');end=p.index('\n\ndef draft_review',start)
p=p[:start]+'''def checked_delta(current,integration):
    assert integration.resolve()==SCOPE
    scope=read(integration);files=scope['files']
    assert scope['baseline_archive_sha256']==sha(BASE/'sources.zip')
    for receipt in scope['integrations'].values():
        assert sha(ROOT/receipt['path'])==receipt['sha256']
    expected=[dict(path=p,**{k:v[k] for k in ('before_sha256','after_sha256','kind')}) for p,v in sorted(files.items())]
    actual=source_delta(current)
    assert actual==expected,('exact reviewed r441-relative actual source delta required',actual,expected)
    return actual
'''+p[end:]
p=p.replace("selection_difference=False,","selection_difference=['g1_complete_plan_trace:=true'],")
p=p.replace('r437 대비 r434 네 파일/r440 한 파일만 허용. 새 3패키지 빌드 필요; 동일소스 빌드재사용 아님.',
    'r441 대비 실제 전체 SHA 목록만 허용. 새 3패키지 build/import/native 확인; 같은 source 재사용 주장이 아님.')
p=p.replace("'header_comparison.py','launch_adapter.py'):","'header_comparison.py'):")
p=p.replace("    assert '--pilot-marker-z' not in arguments()",'''    adapter=(A/'launch_adapter.py').read_text(encoding='utf8').replace("    'g1_complete_plan_trace:=true',\\n",'')
    assert adapter==(BASE/'launch_adapter.py').read_text(encoding='utf8')
    assert '--pilot-marker-z' not in arguments()''')
p=p.replace(".replace('crop_crawl_drive_r441_v1/run.py','applied_limit_drive_r437_v1/run.py').replace(\"Node('r441_owned_stop')\",\"Node('r437_owned_stop')\")",
    ".replace('guard_crawl_drive_r452_v1/run.py','crop_crawl_drive_r441_v1/run.py').replace(\"Node('r452_owned_stop')\",\"Node('r441_owned_stop')\")")
p=p.replace('launch_selection_byte_equal=True','launch_selection_equal_except_trace=True')
p=p.replace("assert build['compiled_again'] is True and build['build_exitcode']==build['import_exitcode']==0",
    "assert build['build_executed'] is True and build['build_exitcode']==build['import_exitcode']==0\n    assert build['native_import_verified'] is True")
p=p.replace("assert bare==expected and review.get('status')=='reviewed' and all(row.get('reason') for row in review['changes'])",
    "assert bare==expected and review.get('status')=='reviewed' and review.get('root_freeze_approved') is True\n    assert all(row.get('reason') for row in review['changes'])")
p=p.replace('r437_source_archive_sha256','r441_source_archive_sha256')
p=p.replace('compiled_again=True,build_receipt=',"build_executed=True,complete_plan_trace=True,build_receipt=")
save('prepare.py',p)
test=source('test_preflight.py').replace('assert SELECTION == old_selection',
    "assert SELECTION == old_selection+['g1_complete_plan_trace:=true']")
test=test.replace("'g1_crop_edge_component_trial:=false']","'g1_crop_edge_component_trial:=false', 'g1_complete_plan_trace:=false']")
save('test_preflight.py',test)
test=source('test_launch_selection.py').replace("g1_current_corner_budget='r396_v1', g1_crop_edge_component_trial='true')",
    "g1_current_corner_budget='r396_v1', g1_crop_edge_component_trial='true', g1_complete_plan_trace='true')")
test=test.replace("('g1_runtime_policy', 'g1_current_corner_budget', 'g1_crop_edge_component_trial')",
    "('g1_runtime_policy', 'g1_current_corner_budget', 'g1_crop_edge_component_trial', 'g1_complete_plan_trace')")
test=test.replace("g1_crop_edge_component_trial='true' if selected else 'false')]",
    "g1_crop_edge_component_trial='true' if selected else 'false',\n                          g1_complete_plan_trace='true' if selected else 'false')]")
save('test_launch_selection.py',test)
save('build_v1/build.sh',source('build_v1/build.sh').replace('crop_crawl_drive_r441_v1','guard_crawl_drive_r452_v1'))
save('build_v1/import_paths.py',source('build_v1/import_paths.py').replace('crop_crawl_drive_r441_v1','guard_crawl_drive_r452_v1'))
save('preflight_v1/preflight.sh',source('preflight_v1/preflight.sh').replace('crop_crawl_drive_r441_v1','guard_crawl_drive_r452_v1'))
print(json.dumps(dict(prepared_tools=True,simulation_started=False)))
