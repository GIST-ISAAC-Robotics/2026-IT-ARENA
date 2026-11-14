"""실제 WallFollow 발행 분기의 pending·timestamp·센서 STOP 계약."""
from types import SimpleNamespace as NS
import time
import pytest

pytest.importorskip('rclpy')
from rclpy.time import Time
from arena_autonomy.wall_follow import WallFollow


def node(command):
    messages=[]
    n=NS(now_s=lambda:1.2, last_control_time=1.1, last_status='RUNNING',last_status_time=1.2,
        enabled=True,signal=NS(started=True,observed='green'),ids=[],side='left',
        last_speed=.5,last_steering=.1,scan=NS(header=NS(stamp=NS(sec=1,nanosec=100000000))),
        scan_wall_time=time.monotonic(),rgb_stamp=1.1,image_wall_time=time.monotonic(),
        settings=dict(scan_timeout_s=.3,image_timeout_s=1.,sensor_wall_timeout_s=3.,
            control_rate_hz=50.,acceleration_mps2=.5,wheelbase_m=.145,lateral_acceleration_limit_mps2=3.),
        motion=NS(project=lambda _: ([],100),last_meta={'reason':'ok'}),
        command=lambda _:command,timing=NS(call=lambda _,f,*a,**kw:f(*a,**kw)),
        state_log=NS(snapshot=lambda:{},submit=lambda _:None),
        publisher=NS(publish=messages.append),status_publisher=NS(publish=lambda _:None),
        get_clock=lambda:NS(now=lambda:Time(nanoseconds=1_200_000_000)))
    return n,messages


def test_pending_preserves_last_request_without_publishing_or_new_timestamp():
    n,m=node(None);WallFollow.control(n)
    assert m==[] and not n.command_was_published
    assert (n.last_speed,n.last_steering)==(.5,.1)


def test_completed_command_uses_original_reference_and_elapsed_rate_limit():
    n,m=node((1.,.4,{'reason':'ok'}))
    n.decision_output_dt_s=.04;n.decision_command_stamp_s=1.1
    WallFollow.control(n)
    assert len(m)==1 and n.command_was_published
    assert m[0].header.stamp.sec==1 and m[0].header.stamp.nanosec==100000000
    assert m[0].drive.speed==pytest.approx(.52)
    assert m[0].drive.steering_angle==pytest.approx(.22)


def test_disabled_or_lost_sensor_still_publishes_immediate_zero():
    for key,value in [('enabled',False),('scan',None),('rgb_stamp',None)]:
        n,m=node(None);setattr(n,key,value);WallFollow.control(n)
        assert len(m)==1 and m[0].drive.speed==0.
        assert m[0].header.stamp.nanosec==200000000


def test_inline_path_keeps_existing_20ms_clamp():
    n,m=node((1.,.4,{'reason':'ok'}));WallFollow.control(n)
    assert m[0].drive.speed==pytest.approx(.51)
    assert m[0].drive.steering_angle==pytest.approx(.16)
