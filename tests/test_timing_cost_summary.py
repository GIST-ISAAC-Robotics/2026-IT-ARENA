"""EOF 계측 비교의 구간 경계와 빈 구간을 확인한다."""
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
from summarize_timing_cost import age_windows


def row(stamp, age):
    return dict(node='local_pursuit', callback='imu', receive_ros_ns=stamp, source_age_ms=age)


def test_half_open_bins_include_last_timestamp_without_double_counting():
    result = age_windows([row(-1, 99), row(0, 1), row(9, 2), row(10, 3), row(20, 4), row(21, 99)], 0, 20, 10)
    values = [item['age_ms']['local_pursuit/imu'] for item in result]
    assert [v['count'] for v in values] == [2, 1, 1]
    assert [v['maximum'] for v in values] == [2, 3, 4]
    assert result[-1]['age_ms']['lidar_safety/wheels']['p99'] is None


@pytest.mark.parametrize('start,end,size', [(2,1,1), (0,1,0), (0,1,-1)])
def test_invalid_window(start, end, size):
    with pytest.raises(ValueError):
        age_windows([], start, end, size)
