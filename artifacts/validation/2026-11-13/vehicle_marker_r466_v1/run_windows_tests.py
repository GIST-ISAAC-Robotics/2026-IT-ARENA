"""공통 package를 읽기 전용 fallback으로 쓰고 수정 모듈은 stage만 import한다."""
import os
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parent
REPO=ROOT.parents[3]
os.environ['PYTHONDONTWRITEBYTECODE']='1'
sys.dont_write_bytecode=True
sys.path[:0]=[str(REPO/'artifacts/validation/2026-11-13/operator_panel_r429_v1/pydeps'),
    str(REPO/'src/arena_autonomy'),str(REPO/'src/arena_vehicle_interface'),str(REPO/'tests')]
import arena_autonomy
arena_autonomy.__path__.insert(0,str(ROOT/'stage/src/arena_autonomy/arena_autonomy'))
import arena_autonomy.race_runtime_policy
arena_autonomy.race_runtime_policy.__path__.insert(0,str(ROOT/'stage/src/arena_autonomy/arena_autonomy/race_runtime_policy'))

if __name__=='__main__':
    import pytest
    raise SystemExit(pytest.main(['-q','-p','no:cacheprovider','--basetemp='+str(ROOT/'testtmp'),
        '-k','not test_existing_node_command_is_unchanged_when_opt_in_is_off and not test_node_g1_input_failure_cannot_fall_back_to_r0',
        str(ROOT/'stage/tests/test_vehicle_marker_regions.py'),
        str(ROOT/'stage/tests/test_vehicle_marker_connection.py'),
        str(ROOT/'stage/tests/test_vehicle_marker_observer.py'),str(ROOT/'stage/tests/test_vision_process.py'),
        str(REPO/'tests/test_local_follow.py'),str(REPO/'tests/test_m3_execution_trial.py'),
        str(REPO/'tests/test_runtime_node_backend.py'),
        '--junitxml='+str(ROOT/'windows_tests.xml')]))
