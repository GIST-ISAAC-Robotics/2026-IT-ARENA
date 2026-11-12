"""후보를 격리 프로세스에만 설치해 기존 측방·GUARD 회귀를 실행한다."""
import pytest
import candidate

candidate.install()
raise SystemExit(pytest.main(['-q',
    'tests/test_pass_release_trial.py', 'tests/test_guard_curve_trial.py',
    'tests/test_observed_guard_approach.py', 'tests/test_guard_masked_speed.py',
    'tests/test_current_lane_preference.py']))
