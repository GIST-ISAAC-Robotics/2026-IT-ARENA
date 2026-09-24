import json
from pathlib import Path
import random
import sys
from collections import deque
from types import SimpleNamespace

import pytest
from sensor_msgs.msg import Imu, JointState

from arena_vehicle_interface.motion_impairment import MotionFaultConfig, StreamFault
from arena_vehicle_interface.motion_fault_relay import alter_message, MotionFaultRelay


@pytest.mark.parametrize('data', [
    {'other': 1}, {'imu': {'unknown': 1}}, {'imu': {'delay_s': -.1}},
    {'imu': {'jitter_s': .01}}, {'imu': {'bias': float('nan')}}, {'seed': True},
    {'wheels': {'bias': .1}}, {'imu': {'end_s': 0}}, {'imu': {'drop': 1}},
    {'imu': {'scale': 0}}, {'imu': {'stamp_offset_s': 2}}, {'imu': {'end_s': float('inf')}}])
def test_invalid_config(data):
    with pytest.raises((ValueError, TypeError)):
        MotionFaultConfig.from_dict(data)


def test_defaults_and_round_trip():
    cfg = MotionFaultConfig()
    assert MotionFaultConfig.from_dict(cfg.as_dict()) == cfg
    fault = cfg.imu
    assert fault.transform(2., 5.) == 2.
    assert fault.claimed_stamp(5.) == 5.
    assert fault.delay(5., random.Random(1)) == 0.


def test_window_and_delay_do_not_change_measurement_stamp():
    fault = StreamFault(bias=.02, scale=1.1, start_s=2., end_s=3., delay_s=.1, jitter_s=.02)
    assert fault.transform(1., 1.999) == 1.
    assert fault.transform(1., 2.) == pytest.approx(1.12)
    assert fault.transform(1., 3.) == 1.
    assert fault.claimed_stamp(2.) == 2.
    rng = random.Random(4)
    assert all(.08 <= fault.delay(2., rng) <= .12 for _ in range(100))


def test_imu_only_z_and_explicit_time_error_copy_input():
    msg = Imu()
    msg.header.stamp.sec = 2
    msg.header.frame_id = 'imu'
    msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = 3., 4., 5.
    changed, before, after = alter_message('imu', msg, StreamFault(bias=.01, scale=1.02, stamp_offset_s=-.005))
    assert before == [5.] and after == pytest.approx([5.11])
    assert msg.angular_velocity.z == 5.
    assert changed.angular_velocity.x == 3. and changed.angular_velocity.y == 4.
    assert changed.header.stamp.sec == 1 and changed.header.stamp.nanosec == 995000000
    assert msg.header.stamp.sec == 2


def test_wheel_gain_named_not_raw_tick_dropout():
    msg = JointState(name=['steering', 'car::rear_right_wheel_joint', 'rear_left_wheel_joint'],
                     position=[.1, 2., 3.], velocity=[.2, 4., 5.])
    result, _, values = alter_message('wheels', msg, StreamFault(scale=.95))
    assert values == pytest.approx([3.8, 4.75])
    assert list(result.position) == pytest.approx([.1, 1.9, 2.85])
    assert list(msg.velocity) == [.2, 4., 5.]
    with pytest.raises(ValueError):
        alter_message('wheels', JointState(), StreamFault())


def relay_double(config):
    node = object.__new__(MotionFaultRelay)
    node.config = config
    node.pending = {k: deque() for k in ('imu', 'wheels')}
    node.rng = {k: random.Random(1) for k in node.pending}
    node.sequence = 0
    node.t = 0.
    node.last_clock = None
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(node.t*1e9)))
    node.rows = []
    node.emit = node.rows.append
    node.sent = []
    node.outputs = {k: SimpleNamespace(publish=node.sent.append) for k in node.pending}
    return node


def test_fifo_delay_then_normal_does_not_reorder_and_clock_reset_clears():
    node = relay_double(MotionFaultConfig(imu=StreamFault(delay_s=.1, end_s=1.)))
    a, b = Imu(), Imu()
    a.header.stamp.nanosec = 990000000
    b.header.stamp.sec = 1
    node.t = .99
    node.capture('imu', a)
    node.t = 1.
    node.capture('imu', b)
    assert not node.sent  # second packet waits for the first, despite zero extra delay
    node.t = 1.09
    node.release()
    assert [m.header.stamp.sec for m in node.sent] == [0, 1]
    node.t = .5
    node.capture('imu', a)
    node.t = .1
    node.clock()
    assert not node.pending['imu']
    assert any(r['event'] == 'clock_reset' for r in node.rows)


def test_drop_window_and_recovery():
    node = relay_double(MotionFaultConfig(imu=StreamFault(drop=True, start_s=1., end_s=2.)))
    msg = Imu()
    msg.header.stamp.sec = 1
    node.t = 1.
    node.capture('imu', msg)
    assert not node.sent and node.rows[-1]['event'] == 'drop'
    msg.header.stamp.sec = 2
    node.t = 2.
    node.capture('imu', msg)
    assert len(node.sent) == 1


def test_audit_detects_wrong_values_and_missing_stream(tmp_path):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'scripts'))
    from motion_fault_audit import audit
    cfg = MotionFaultConfig(imu=StreamFault(bias=.01))
    rows = [{'event': 'config', 'config': cfg.as_dict()}]
    for kind, count in [('imu', 1), ('wheels', 2)]:
        for i in range(30):
            t = i*.01
            rows.append({'event': 'publish', 'stream': kind, 'source_stamp_s': t,
                         'received_s': t, 'released_s': t, 'due_s': t, 'delay_s': 0.,
                         'claimed_stamp_s': t, 'fault_active': True,
                         'before': [1.]*count, 'after': [1.01 if kind == 'imu' else 1.]*count})
    rows.append({'event': 'close'})
    log = tmp_path/'input.jsonl'
    log.write_text('\n'.join(json.dumps(r) for r in rows))
    assert audit(log, cfg.as_dict())['verified']
    rows[1]['after'] = [1.]
    log.write_text('\n'.join(json.dumps(r) for r in rows))
    assert not audit(log, cfg.as_dict())['verified']


def test_validator_forwards_motion_profile(monkeypatch):
    monkeypatch.syspath_prepend(str(Path(__file__).resolve().parents[1]/'scripts'))
    import validate_basic_autonomy as validator
    calls = []
    monkeypatch.setattr(validator.subprocess, 'Popen', lambda cmd, **kw: calls.append(cmd))
    validator.start_demo_process(None, motion_test_profile='/tmp/motion with spaces.json')
    assert 'motion_test_profile:=/tmp/motion with spaces.json' in calls[0]
