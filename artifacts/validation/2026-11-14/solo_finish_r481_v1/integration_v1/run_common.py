from pathlib import Path
import sys,os
ROOT=Path(__file__).resolve().parents[5]
A=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
os.environ['PYTHONDONTWRITEBYTECODE']='1'
from arena_autonomy.numeric_kernels import configure
configure('native_v1')
import arena_autonomy.local_follow as follow
assert Path(follow.__file__).resolve()==ROOT/'src/arena_autonomy/arena_autonomy/local_follow.py'
import pytest
raise SystemExit(pytest.main(['-q','--tb=short','-p','no:cacheprovider',
 str(A/'test_route_observation.py'),
 str(A/'test_inputs/test_runtime_dual_contract.py')+'::test_generation_invalidation_clears_deceleration_cap',
 str(A/'test_inputs/test_runtime_dual_contract.py')+'::test_stop_between_selection_and_publish_cancels_pending_ack',
 str(ROOT/'tests/test_current_lane_preference.py'),str(ROOT/'tests/test_current_lane_region_reuse.py'),
 '-k','not explicit_launch_option_and_recorded_header','--junitxml='+str(A/'integration_v1/tests.xml')]))
