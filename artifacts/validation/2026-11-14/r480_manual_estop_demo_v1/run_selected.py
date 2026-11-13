"""기존 STOP 관련7개만 Windows 순수/모의로 실행. 새 테스트/장치 실행 없음."""
from pathlib import Path
import hashlib,json,os,sys
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.dont_write_bytecode=True;os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD']='1'
sys.path[:0]=[str(ROOT/'artifacts/validation/2026-11-13/complete_plan_trace_r449_v1/test_dependencies'),
    str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
tests=[
    'tests/test_operator_panel_manual.py::test_request_binding_and_stop_invalidate_queued_run',
    'tests/test_operator_panel_manual.py::test_child_reuses_jog_and_eof_stop_is_bounded',
    'tests/test_operator_panel_manual.py::test_successful_child_does_not_allow_second_pulse_without_prepare',
    'tests/test_actuation_contract.py::ContractTests::test_stop_release_clear_and_arm_are_separate',
    'tests/test_development_jog.py::test_start_requires_explicit_arm_and_no_existing_target',
    'tests/test_race_ops.py::test_stop_needs_no_view_even_when_status_is_stale',
    'tests/test_runtime_actuation_link.py::test_stop_purges_pending_controls_and_drive_and_does_not_queue_drive_behind']
sources=['src/arena_autonomy/arena_autonomy/'+s for s in ('race_ops_node.py','race_ops_manual.py','development_jog.py')]+[
    'src/arena_vehicle_interface/firmware_runtime_drive/main/main.cpp',
    'src/arena_vehicle_interface/firmware_runtime_preview/main/stop_switch.hpp',
    'src/arena_vehicle_interface/portable/actuation_receiver.hpp',
    'src/arena_vehicle_interface/arena_vehicle_interface/actuation_contract.py']
sources+=list(dict.fromkeys(x.split('::')[0] for x in tests))
sha=lambda p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
before={p:sha(p) for p in sources}
import pytest
code=pytest.main(['-q','-p','no:cacheprovider','--basetemp',str(A/'pytest_tmp_v1'),
    '--junitxml',str(A/'pytest.xml'),*[str(ROOT/t.split('::')[0])+'::'+'::'.join(t.split('::')[1:]) for t in tests]])
result=dict(exit_code=code,selected_tests=tests,source_sha256=before,
    changed=[p for p,h in before.items() if sha(p)!=h],python=sys.executable,pytest=pytest.__version__,
    new_tests=0,source_modified=False,ros_started=False,firmware_built=False,hardware_access=False,
    scope='existing pure Python/model/fake process tests plus read-only source connection review')
(A/'receipt.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
assert not result['changed'];raise SystemExit(code)
