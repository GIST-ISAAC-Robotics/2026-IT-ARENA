"""완료 영수증에서 실행 전 인계 정보만 만든다. 주행을 실행하지 않는다."""
from pathlib import Path
import hashlib
import json
import xml.etree.ElementTree as ET

A=Path(__file__).resolve().parent;BASE=A.parent/'applied_limit_drive_r437_v1'
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()
def read(path):return json.loads(path.read_bytes())
prepared=read(A/'prepared.json');source=read(A/'source_review_actual_v1.json')
assert (A/'preflight_v1/pytest.exit').read_text().strip()=='0'
suites=list(ET.parse(A/'preflight_v1/junit.xml').getroot().iter('testsuite'))
assert sum(int(s.attrib['tests']) for s in suites)==16
assert all(int(s.attrib.get(key,0))==0 for s in suites for key in ('failures','errors','skipped'))
assert not read(A/'preflight_v1/resource_release.json')['remaining_owned_work']
assert not (A/'registered.json').exists() and not (A/'run1').exists()
result=dict(schema='r441_preparation_manifest_v1',ready_for_root_run_review=True,run_started=False,
    root_execution_confirmation='pending',baseline='applied_limit_drive_r437_v1',
    common_delta=dict(total=5,changed_existing=3,new=2,r434=4,r440=1,
        unchanged_existing=source['unchanged_common_files'],current_common=source['current_common_files']),
    compiled_again=True,packages=['arena_autonomy','arena_vehicle_interface','arena_bringup'],
    build_exitcode=0,import_exitcode=0,preflight=dict(tests=16,failures=0,errors=0,skipped=0,exitcode=0,
        source_checks_before_after=True),frozen_input_count=len(prepared['frozen_inputs']),
    prepared_sha256=sha(A/'prepared.json'),sources_sha256=sha(A/'sources.zip'),
    source_scope_sha256=sha(A/'source_scope.json'),review_sha256=sha(A/'source_review_approved_v1.json'),
    config_actor_header_launch_preserved=True,sim_duration_s=80,real_time_factor=.25,
    stop_after_no_progress_sim_s=8,execution_limit=1,repeat_on_failure=False,
    r434_preview_off=True,r439_stage_not_integrated=True,prior35_and409_not_repeated=True,
    physical_or_jetson_access=False,remaining_owned_work=[])
(A/'preparation_manifest.json').write_bytes((json.dumps(result,ensure_ascii=False,indent=2)+'\n').encode())
execution=A/'execution_v1';execution.mkdir(exist_ok=False)
script=(BASE/'execution_v1/execute_once.sh').read_text(encoding='utf-8')
script=script.replace('applied_limit_drive_r437_v1','crop_crawl_drive_r441_v1').replace('r437','r441')
script=script.replace('8dc275de3c88cf192ed30ea55ba769f4defb9c4cdc1425e996e5285967854697',result['prepared_sha256'])
script=script.replace("suites)==16","suites)==16")
(execution/'execute_once.sh').write_bytes(script.encode())
# root_execution_review.json은 여기서 만들지 않는다. 명시 검토와 승인은 주 담당 소유다.
print(json.dumps(result,ensure_ascii=False))
