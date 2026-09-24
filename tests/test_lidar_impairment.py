import math
from types import SimpleNamespace

import numpy as np
import pytest
from arena_vehicle_interface.lidar_impairment import ImpairmentConfig, ScanImpairment


def test_disabled_error_preserves_values_and_invalid_returns():
    model = ScanImpairment(ImpairmentConfig())
    values = [1., math.inf, -math.inf, math.nan, .01, 20.]
    result = model.distort(values, .05, 12.)
    assert np.allclose(result, values, equal_nan=True)
    assert model.summary()['samples'] == 1 and model.summary()['max_abs_error_m'] == 0.


def test_uniform_bounds_statistics_and_repeatable_seed():
    models = [ScanImpairment(ImpairmentConfig(mode='uniform', amplitude_m=.03, seed=22)) for _ in range(2)]
    values = models[0].distort([1.]*10000, .05, 12.)
    assert values == models[1].distort([1.]*10000, .05, 12.)
    assert min(values) >= .97 and max(values) <= 1.03
    stats = models[0].summary()
    assert abs(stats['mean_error_m']) < .001
    assert stats['rms_error_m'] == pytest.approx(.03/np.sqrt(3), abs=.001)


def test_bias_does_not_average_away_and_invalid_output_is_unknown():
    model = ScanImpairment(ImpairmentConfig(bias_m=-.03))
    result = model.distort([1., 1., .06], .05, 12.)
    assert result[:2] == [.97, .97] and math.isnan(result[2])
    assert model.summary()['invalidated'] == 1


def test_delay_preserves_acquisition_header_and_never_releases_early():
    model = ScanImpairment(ImpairmentConfig(delay_s=.08))
    scan = SimpleNamespace(stamp=1., time_increment=.0002, scan_time=.1)
    model.enqueue(scan, 1.1, {})
    assert not model.advance(1.179)
    released, audit = model.advance(1.18)[0]
    assert released is scan and released.stamp == 1. and released.time_increment == .0002
    assert audit['injected_delay_s'] == pytest.approx(.08)


def test_time_rollback_discards_queued_old_scans():
    model = ScanImpairment(ImpairmentConfig(delay_s=.3))
    model.advance(10.)
    model.enqueue(object(), 10., {})
    assert model.advance(1.) == [] and not model.queue


def test_jitter_is_bounded_and_scans_keep_original_order():
    model = ScanImpairment(ImpairmentConfig(delay_s=.05, jitter_s=.02, seed=23))
    released = []
    for i in range(1000):
        stamp = i*.002
        if i % 50 == 0:
            model.enqueue(i, stamp, {})
        released.extend(model.advance(stamp))
    assert [scan for scan, _ in released] == sorted(scan for scan, _ in released)
    assert all(.03-1e-9 <= info['injected_delay_s'] <= .072+1e-9 for _, info in released)


def test_rotating_node_delays_scan_but_preserves_first_ray_timestamp():
    import json
    from sensor_msgs.msg import LaserScan
    from arena_vehicle_interface.rotating_lidar import RotatingLidar, RevolutionAssembler
    scans, audits = [], []
    node = SimpleNamespace(mode='sequential', assembler=RevolutionAssembler(),
        impairment=ScanImpairment(ImpairmentConfig(mode='uniform', amplitude_m=.03, delay_s=.08)),
        publisher=SimpleNamespace(publish=scans.append), audit=SimpleNamespace(publish=audits.append))
    node.publish_ready = lambda ready: RotatingLidar.publish_ready(node, ready)
    for i in range(181):
        scan = LaserScan(range_min=.05, range_max=12., ranges=[1.]*500)
        scan.header.stamp.nanosec = i*1_000_000
        RotatingLidar.on_scan(node, scan)
        if i < 180:
            assert scans == []
    assert len(scans) == 1 and scans[0].header.stamp.nanosec == 0
    assert scans[0].scan_time == pytest.approx(.1) and scans[0].time_increment == pytest.approx(.0002)
    assert min(scans[0].ranges) >= .97-1e-6 and max(scans[0].ranges) <= 1.03+1e-6
    audit = json.loads(audits[0].data)
    assert audit['injected_delay_s'] == pytest.approx(.08)
    assert audit['max_capture_time_error_s'] <= .00100001


@pytest.mark.parametrize('kwargs', [{'mode':'fake'}, {'amplitude_m':float('nan')},
    {'bias_m':.3}, {'delay_s':-.1}, {'delay_s':.6}, {'jitter_s':.1}, {'seed':-1},
    {'seed':1.2}, {'mode':'none','amplitude_m':.03}])
def test_bad_profiles_rejected(kwargs):
    with pytest.raises(ValueError):
        ImpairmentConfig(**kwargs)
