"""Windows stage overlay. ROS/장치 미실행, 기존 설치 pytest만 재사용."""
from pathlib import Path
import sys
sys.dont_write_bytecode=True

A=Path(__file__).resolve().parent
ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'artifacts/validation/2026-11-13/operator_panel_r429_v1/pydeps'),
    str(ROOT/'artifacts/validation/2026-11-13/installed_limit_chain_r439_v1/verify_v1/deps'),
    str(A/'stage/scripts'),str(ROOT/'scripts'),str(ROOT/'src/arena_autonomy'),
    str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'tests')]
import arena_autonomy
arena_autonomy.__path__.insert(0,str(A/'stage/src/arena_autonomy/arena_autonomy'))
# 기존 test_runtime_installed_limits는 자체 sys.path를 prepend하므로 stage를 먼저 고정한다.
import runtime_installed_limits
assert Path(runtime_installed_limits.__file__).resolve()==(A/'stage/scripts/runtime_installed_limits.py').resolve()
if __name__=='__main__':
    import pytest
    args=sys.argv[1:] or [str(A/'stage/tests/test_independent_guard_route.py'),
                           str(ROOT/'tests/test_runtime_installed_limits.py'),'-q','-p','no:cacheprovider']
    raise SystemExit(pytest.main(args))
