"""추가한 재생 import와 검사 의존성의 명시적 배포 목록을 확인한다."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import prepare_jetson_bundle as bundle


def test_replay_subsets_include_new_dependency_once():
    for names in (bundle.DDS_TOOLS, bundle.IMU_PROBE_TOOLS,
                  ('scripts/replay_sensor_bag.py', 'scripts/run_jetson_replay_trial.py', 'scripts/replay_imu_probe.py')):
        paths = [bundle.ROOT/name for name in names]
        result = bundle.with_replay_dependencies(paths)
        assert result.count(bundle.ROOT/'scripts/replay_timing.py') == 1
        assert set(paths) <= set(result)
        assert bundle.with_replay_dependencies(result) == result
        assert all(path.is_file() for path in result)


def test_full_bundle_includes_timing_and_imu_test_dependencies():
    assert {'replay_timing.py', 'analyze_imu_delivery.py', 'jetson_run_timing_cost.sh'} <= set(bundle.SCRIPTS)
    assert {'test_replay_timing.py', 'test_imu_delivery_analysis.py'} <= set(bundle.TESTS)
    assert all((bundle.ROOT/'scripts'/name).is_file() for name in bundle.SCRIPTS)
    assert all((bundle.ROOT/'tests'/name).is_file() for name in bundle.TESTS)
    assert bundle.with_replay_dependencies([]) == []
