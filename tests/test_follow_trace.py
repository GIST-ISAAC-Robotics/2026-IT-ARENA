import copy
import json
import math
from types import SimpleNamespace

import pytest

from arena_autonomy.follow_trace import (FollowTrace, equal_output, freeze, replay_events,
                                        restore_call, snapshot_call, thaw)
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.local_follow import FollowConfig, LocalFollow
from arena_autonomy.local_pursuit import LocalPursuit
from validate_scan_motion_diagnostic import synthetic_scan


def context(reference=.32):
    raw = synthetic_scan(reference-.12)
    t = raw.pop('stamp_s')
    scan = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=0, nanosec=round(t*1e9)),
                                                  frame_id='laser_frame'),
                           range_min=.05, range_max=12., **raw)
    history = MotionHistory(MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06))
    for i in range(71):
        history.add_mean_wheel(i*.01, 0.)
        history.add_gyro(i*.01, 0.)
    return scan, history


def controller(trace=None):
    # ROS 전체 실행 없이 실제 command()/reset_follow()를 검사한다.
    node = LocalPursuit.__new__(LocalPursuit)
    scan, history = context()
    node.motion = SimpleNamespace(history=history, last_meta={'reference_stamp_s': .32},
                                  verified=True, counts={'resets': 0})
    node.follow = LocalFollow()
    node.follow_trace = trace
    node.scan, node.side, node.last_steering = scan, 'left', 0.
    node.settings = {'control_rate_hz': 50.}
    node.rear_blind_half_angle = 30.
    return node


def make_trace(tmp_path):
    path = tmp_path/'trace.jsonl'
    scan, history = context()
    recorder = FollowTrace(path, history.config, FollowConfig())
    enabled, disabled = controller(recorder), controller()
    for i, ref in enumerate((.32, .34, .42, .44, .52, .54)):
        for node in (enabled, disabled):
            node.motion.last_meta['reference_stamp_s'] = ref
            node.scan = context(ref if i % 2 == 0 else ref-.02)[0]
        a, b = enabled.command(None), disabled.command(None)
        assert equal_output(freeze(a), freeze(b))  # 기록은 판단 결과를 바꾸지 않는다.
        if i == 3:
            enabled.reset_follow('SENSOR_STOP')
            disabled.reset_follow('SENSOR_STOP')
    # 잘못된 frame의 실제 예외 경로와 내부 reset까지 일치해야 한다.
    enabled.scan.header.frame_id = 'wrong_frame'
    assert enabled.command(None)[2]['reason'] == 'follow_input_stop'
    recorder.close()
    return path, [json.loads(line) for line in path.read_text().splitlines()]


def test_live_decision_stage_replay_and_reset_match(tmp_path):
    _, events = make_trace(tmp_path)
    result = replay_events(events)
    assert result['passed'] and result['matched'] == result['calls'] == 7
    assert any(e['kind'] == 'reset' for e in events)
    assert events[-1]['data']['dropped'] == events[-1]['data']['failed'] == 0


def test_snapshot_owns_inputs_and_preserves_nonfinite():
    scan, history = context()
    scan.ranges[:3] = [math.nan, math.inf, -math.inf]
    data = snapshot_call(scan, history, .32, 0, 0., 'left', 0., .02, True, 30.)
    json.dumps(data, allow_nan=False)
    scan.ranges[3] = 999.
    history.wheels.values[-1] = 99.
    decoded, restored_scan, restored = restore_call(data, history.config)
    assert math.isnan(restored_scan.ranges[0])
    assert restored_scan.ranges[1:3] == [math.inf, -math.inf]
    assert restored_scan.ranges[3] != 999.
    assert restored.wheels.values[-1] == 0.


def test_stored_history_not_double_converted():
    scan, history = context()
    history.wheels.values[-1] = 1.23
    history.gyro.values[-1] = -.45
    data = snapshot_call(scan, history, .32, 0, 1.23, 'left', 0., .02, True, 30.)
    _, _, restored = restore_call(data, history.config)
    assert restored.wheels.values[-1] == 1.23 and restored.gyro.values[-1] == -.45


@pytest.mark.parametrize('fault', ['no_footer', 'gap', 'trailing', 'dropped', 'failed', 'pending', 'capture_errors'])
def test_invalid_capture_is_not_replayed_as_pass(tmp_path, fault):
    _, events = make_trace(tmp_path)
    if fault == 'no_footer':
        events.pop()
    elif fault == 'gap':
        del events[2]
    elif fault == 'trailing':
        events.append(copy.deepcopy(events[-1]))
    else:
        events[-1]['data'][fault] += 1
    with pytest.raises(ValueError):
        replay_events(events)


def test_modified_output_is_mismatch(tmp_path):
    _, events = make_trace(tmp_path)
    next(e for e in events if e['kind'] == 'call')['data']['output'][0] += .1
    result = replay_events(events)
    assert not result['passed'] and len(result['mismatches']) == 1


def test_existing_file_not_overwritten(tmp_path):
    path, _ = make_trace(tmp_path)
    before = path.read_bytes()
    with pytest.raises(FileExistsError):
        FollowTrace(path, context()[1].config, FollowConfig())
    assert path.read_bytes() == before


def test_invalid_history_rejected():
    scan, history = context()
    data = snapshot_call(scan, history, .32, 0, 0., 'left', 0., .02, True, 30.)
    data['history']['wheels']['times'][1] = data['history']['wheels']['times'][0]
    with pytest.raises(ValueError, match='captured history'):
        restore_call(data, history.config)


def test_encoding_rejects_unsupported_object():
    with pytest.raises(TypeError):
        freeze(object())
    with pytest.raises(ValueError):
        thaw({'nonfinite': 'anything'})
