import numpy as np
import pytest
import json
from pathlib import Path
from arena_autonomy.local_path import pursuit_command


def straight_scan(seed, noise=.03, bias=0.):
    angles = np.linspace(-np.pi, np.pi, 500, endpoint=False)
    ranges = .425/np.maximum(1e-9, np.abs(np.sin(angles)))
    keep = (ranges < 12.) & (np.abs(angles) <= np.deg2rad(150))
    ranges += np.random.default_rng(seed).uniform(-noise, noise, len(ranges)) + bias
    return np.column_stack((ranges[keep]*np.cos(angles[keep])+.06,
                            ranges[keep]*np.sin(angles[keep])))


@pytest.mark.parametrize('side', ['left', 'right'])
def test_three_centimeter_radial_noise_does_not_consume_straight_horizon(side):
    for seed in range(50):
        speed, steer, meta = pursuit_command(straight_scan(seed), side, 1.4, max_speed=2.5)
        assert speed > 1.7
        assert abs(steer) < .04
        assert meta['observed_path_ahead_m'] > 1.25


def test_common_bias_remains_visible_not_magically_removed():
    for bias in (-.03, .03):
        _, steer, _ = pursuit_command(straight_scan(22, noise=0., bias=bias), 'left', 1.4)
        assert abs(steer) > .01


@pytest.mark.parametrize('run', ['lap_configured_v3', 'lap_configured_v5'])
def test_overshot_corner_stays_stopped_with_radial_noise(run):
    source = Path(__file__).resolve().parents[1]/'artifacts/validation/2026-09-21/local_pursuit'/run/'last_scan.json'
    scan = json.loads(source.read_text())
    ranges = np.array([np.nan if r is None else r for r in scan['ranges']])
    angles = scan['angle_min']+np.arange(len(ranges))*scan['angle_increment']
    keep = np.isfinite(ranges) & (np.abs(angles) <= np.deg2rad(150))
    for seed in range(50):
        noisy = ranges+np.random.default_rng(seed).uniform(-.03, .03, len(ranges))
        valid = keep & (noisy >= scan['range_min']) & (noisy <= scan['range_max'])
        points = np.column_stack((noisy[valid]*np.cos(angles[valid])+.06,
                                  noisy[valid]*np.sin(angles[valid])))
        assert pursuit_command(points, 'left', 0.)[0] == 0.
