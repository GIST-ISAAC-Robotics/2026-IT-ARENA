"""후열 좌/우 시험 허용과 선두/다른 구성의 무단 확장을 구별한다."""
import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
import official_traffic_course as course


@pytest.mark.parametrize('slot', range(6))
def test_only_rear_grid_slots_can_select_registered_traffic_course(slot):
    args = SimpleNamespace(mode='g1g2', handoff_reserve='paced', handoff_yield_progress=True,
                           handoff_rear_sweep=True, record_follow_decisions=True, seed=0)
    config = {'launch': {'speed_profile': 'g1_open_1p5', 'grid_slot': slot}}
    if slot in (0, 3):
        assert 'scripts/official_traffic_course.py' in course.inputs(args, config, {})
    else:
        with pytest.raises(ValueError):
            course.inputs(args, config, {})
