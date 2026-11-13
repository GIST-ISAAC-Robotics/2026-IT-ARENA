"""명시 후보를 켠 상태의 관련 기존 회귀. 원 src/테스트를 편집하지 않는다."""
from pathlib import Path
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'tests'),str(ROOT/'scripts')]
import pytest
import candidate

if __name__=='__main__':
    selected=['test_pass_release_trial.py','test_parallel_lane_trial.py','test_masked_motion_speed.py',
              'test_guard_curve_trial.py','test_guard_masked_speed.py','test_released_body_gap.py',
              'test_current_path_recheck.py']
    with candidate.enabled():
        raise SystemExit(pytest.main(['-q','--tb=short','-p','no:cacheprovider',
            *[str(ROOT/'tests'/name) for name in selected]]))
