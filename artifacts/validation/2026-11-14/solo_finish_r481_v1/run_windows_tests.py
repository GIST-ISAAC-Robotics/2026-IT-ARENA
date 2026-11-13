import os,sys
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.dont_write_bytecode=True;os.environ['PYTHONDONTWRITEBYTECODE']='1'
sys.path[:0]=[str(ROOT/'artifacts/validation/2026-11-13/operator_panel_r429_v1/pydeps'),
    str(A/'stage/src/arena_autonomy'),str(A/'stage/src/arena_vehicle_interface'),str(ROOT/'tests')]
if __name__=='__main__':
    import pytest
    raise SystemExit(pytest.main(['-q','-p','no:cacheprovider','--basetemp='+str(A/'testtmp'),
        str(A/'test_route_observation.py'),
        str(ROOT/'tests/test_runtime_dual_contract.py')+'::test_generation_invalidation_clears_deceleration_cap',
        str(ROOT/'tests/test_runtime_dual_contract.py')+'::test_stop_between_selection_and_publish_cancels_pending_ack',
        '--junitxml='+str(A/'windows_tests.xml')]))
