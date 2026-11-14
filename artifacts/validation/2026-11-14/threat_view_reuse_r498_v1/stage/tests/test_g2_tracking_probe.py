import math

import numpy as np
import pytest

from g2_geometry_probe import sample_path
from g2_tracking_probe import bicycle_step, local_reference, rollout


def test_bicycle_straight_and_circle_exact():
    assert bicycle_step(0, 0, 0, 1., 0, .1, .135) == pytest.approx((.1, 0, 0))
    steer = math.atan(.135)
    assert bicycle_step(0, 0, 0, 1., steer, math.pi/2, .135) == pytest.approx((1, 1, math.pi/2))


def test_local_reference_uses_body_not_rear_frame_and_monotonic_cursor():
    path = sample_path(0., .9)
    reference = np.array([(r['rear_x_m'], r['rear_y_m']) for r in path['samples']])
    local, cursor = local_reference(reference, (-.0675, 0, 0), .135, 0)
    assert local[0] == pytest.approx((-.0675, 0))
    assert len(local) == 36
    assert cursor == 0
    _, new_cursor = local_reference(reference, (1., 0, 0), .135, cursor)
    assert new_cursor > cursor


def test_straight_tracking_and_delay_are_not_real_drive_permission():
    result = rollout(0., .9, target_y=.5)
    assert result['checks']['checkpoint_reached']
    assert result['final_rear_pose'][1:] == pytest.approx([0, 0])
    assert not result['driving_authorized']
    delayed = rollout(0., .9, target_y=.5, delay_steps=5)
    assert all(row['applied_speed_mps'] == 0 for row in delayed['rows'][:5])
    assert delayed['rows'][5]['applied_speed_mps'] > 0


def test_command_slew_stays_at_existing_rate_and_every_pose_is_recorded():
    result = rollout(-.10, .9)
    rows = result['rows']
    assert result['steps'] == len(rows)
    assert all(abs(b['command_steering_rad']-a['command_steering_rad']) <= .06+1e-12
               for a,b in zip(rows, rows[1:]))
    assert all(abs(row['applied_steering_rad']) <= .37 for row in rows)
    assert all(np.isfinite([row['body_gap_lower_m'], row['road_edge_lower_m']]).all() for row in rows)


@pytest.mark.parametrize('kwargs', [{'speed_cap': 1.1}, {'speed_cap': float('nan')},
                                   {'delay_steps': True}, {'delay_steps': -1}, {'delay_steps': .5}])
def test_invalid_probes(kwargs):
    with pytest.raises(ValueError):
        rollout(-.1, .9, **kwargs)
