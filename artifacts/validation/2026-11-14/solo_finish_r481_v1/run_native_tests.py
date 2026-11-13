"""WSL 기존 pytest/native로만 실행. Windows 실패 근거를 덮어쓰지 않는다."""
import os,sys
from pathlib import Path
A=Path(__file__).resolve().parent
sys.dont_write_bytecode=True
os.environ['PYTHONDONTWRITEBYTECODE']='1'
sys.path[:0]=[str(A/'stage/src/arena_autonomy'),str(A/'stage/src/arena_vehicle_interface')]
from arena_autonomy.numeric_kernels import configure
configure('native_v1')
if __name__=='__main__':
    import pytest
    result=sys.argv[1] if len(sys.argv)>1 else 'native_run_v1'
    raise SystemExit(pytest.main(['-q','--tb=short','-p','no:cacheprovider',
        '--basetemp='+str(A/'native_test_tmp'),str(A/'test_route_observation.py'),
        str(A/'test_inputs/test_runtime_dual_contract.py')+'::test_generation_invalidation_clears_deceleration_cap',
        str(A/'test_inputs/test_runtime_dual_contract.py')+'::test_stop_between_selection_and_publish_cancels_pending_ack',
        '--junitxml='+str(A/result/'tests.xml')]))
