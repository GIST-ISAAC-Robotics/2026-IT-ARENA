"""카메라 취득과 독립인 선택 ROS 발행 주기: 누적 밀림·시계·잘못된 인자 경계."""
import pytest
from bridge_b0390_preview import PublishCadence


def test_default_forwards_every_received_frame():
    gate=PublishCadence()
    assert all(gate.admit(t) for t in [0,1,100,100,10**15])


def test_60hz_jitter_does_not_turn_20hz_into_15hz():
    gate=PublishCadence(20)
    selected=[]
    for frame in range(600):
        stamp=round(frame*1e9/60)+(100_000 if frame%2 else -100_000)
        if gate.admit(stamp):selected.append(stamp)
    assert len(selected)==200
    assert max(b-a for a,b in zip(selected,selected[1:]))<67_000_000


def test_long_pause_drops_old_deadlines_without_burst():
    gate=PublishCadence(20)
    assert gate.admit(0)
    assert gate.admit(1_123_000_000)
    assert not gate.admit(1_123_000_000)
    assert not gate.admit(1_124_000_000)
    assert gate.admit(1_150_000_000)


def test_clock_before_next_deadline_does_not_reopen_gate():
    gate=PublishCadence(20)
    assert gate.admit(100_000_000)
    assert not gate.admit(90_000_000)
    assert gate.admit(150_000_000)


@pytest.mark.parametrize('hz',[0,-1,60.1,float('nan'),float('inf'),float('-inf')])
def test_invalid_rate_rejected_before_ros_or_camera(hz):
    with pytest.raises(ValueError):PublishCadence(hz)
