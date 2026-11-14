"""독립 시계 송신 후보: 실제 수신기 만료가 최종이며 실물 출력 시험이 아니다."""
from types import SimpleNamespace

import pytest

from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig
from arena_vehicle_interface.offer_clock import RECEIVER_LEASE, SHARED_CLOCK, offer_send_eligible


def eligible(offer, *, policy=RECEIVER_LEASE, host=1_700_000_000_000_000, elapsed=.002):
    return offer_send_eligible(offer, policy=policy, host_now_us=host,
                               received_wall=0., now_wall=elapsed, wall_timeout=3.)


@pytest.fixture
def receiver():
    # 합성 정지 피드백. 실제 보드/엔코더에 이 값을 주입하지 않는다.
    r = ActuationReceiver(boot_id='actual-clock-not-host', config=ContractConfig(
        feedback_mode='drive_motor_shaft', ticks_per_rev=28, gear_ratio=2.5*81/17, wheel_radius_m=.0325))
    return r


def settle(r, start=0, end=220_000):
    for t in range(start, end+1, 10_000):
        r.update_motor_encoder(t, sequence=t//10_000, capture_us=t, motor_counts=0)
        r.tick(t)


@pytest.mark.parametrize('host_now', [10_000, 1_700_000_000_000_000])
def test_independent_offsets_keep_receiver_deadline(receiver, host_now):
    r=receiver; settle(r)
    producer=CommandProducer(client_id='host')
    offer=r.offer(220_000)
    assert eligible(offer, host=host_now)
    assert not eligible(offer, host=host_now, policy=SHARED_CLOCK)
    assert r.receive(producer.control(offer, 'ARM'), 220_000)[0]
    offer=r.offer(220_000)
    cmd=producer.drive(offer, speed_mps=.2, steering_rad=0., source_us=host_now-2000, host_now_us=host_now)
    settle(r, 230_000, 240_000)
    assert r.receive(cmd, 240_000)[0]
    assert cmd['source_age_us']==2000
    assert r.deadline_us==318_000  # issue +100ms -2ms; receipt does not start a new lease.


def test_delayed_offer_can_be_sent_but_receiver_rejects_expiry(receiver):
    r=receiver; settle(r)
    p=CommandProducer(client_id='host')
    old=r.offer(220_000)
    settle(r, 230_000, 340_000)
    assert eligible(old)  # host has just received it; no assertion of MCU validity.
    assert r.receive(p.control(old, 'ARM'), 340_000)==(False, 'expired_lease')
    assert r.target_speed==0


def test_delayed_drive_does_not_extend_source_or_rearm(receiver):
    r=receiver; settle(r); p=CommandProducer(client_id='host')
    assert r.receive(p.control(r.offer(220_000), 'ARM'), 220_000)[0]
    old=r.offer(220_000)
    cmd=p.drive(old, speed_mps=.2, steering_rad=0., source_us=999_960_000, host_now_us=1_000_000_000)
    settle(r, 230_000, 280_000)
    assert r.receive(cmd, 280_000)==(False, 'source_expired')
    assert r.target_speed==0
    assert r.receive({'v':1,'kind':'STOP'}, 280_000)[0]
    assert not r.receive(cmd, 280_000)[0]


@pytest.mark.parametrize('elapsed', [-.001, .1, 3., float('nan'), float('inf')])
def test_local_receipt_is_bounded(receiver, elapsed):
    assert not eligible(receiver.offer(0), elapsed=elapsed)


@pytest.mark.parametrize('change', [{'lease_expires_us': None}, {'lease_expires_us':100001},
    {'lease_expires_us':0}, {'lease_issued_us':True}, {'generation':-1}, {'boot':''}, {'lease':0}])
def test_receiver_policy_requires_structural_lease(receiver, change):
    offer=receiver.offer(0); offer.update(change)
    assert not eligible(offer)


def test_original_shared_clock_boundaries_preserved(receiver):
    offer=receiver.offer(0)
    for host in (0, 99_999, 100_000, -1):
        for elapsed in (0., 2.99, 3.):
            assert eligible(offer, policy=SHARED_CLOCK, host=host, elapsed=elapsed)==(elapsed<3. and 0<=host<100_000)


def test_unknown_policy_fails():
    with pytest.raises(ValueError):
        eligible({}, policy='auto')


def test_ros_callback_keeps_age_and_output_gate(receiver, monkeypatch):
    pytest.importorskip('rclpy')
    from ackermann_msgs.msg import AckermannDriveStamped
    from arena_vehicle_interface import actuation_ros as ros
    from arena_vehicle_interface.actuation_wire import encode, Decoder
    r=receiver; settle(r); p=CommandProducer(client_id='host')
    assert r.receive(p.control(r.offer(220_000),'ARM'), 220_000)[0]
    offer=dict(r.offer(220_000), type='offer', output_ready=False)
    sent=[]
    h=SimpleNamespace(producer=p, offer=None, offer_wall=0., decoder=Decoder(), pending={},
        arm_barrier=999_900_000, now_us=lambda:1_000_000_000, wall_timeout=3.,
        offer_time_policy=RECEIVER_LEASE, tx=None, send=lambda _,x: sent.append(x))
    h.fresh_offer=lambda:ros.ActuationHost.fresh_offer(h)
    monkeypatch.setattr(ros.time,'monotonic',lambda:100.)
    ros.ActuationHost.on_rx(h, SimpleNamespace(data=encode(offer)))
    message=AckermannDriveStamped()
    message.header.stamp.sec=999; message.header.stamp.nanosec=995_000_000
    message.drive.speed=.2
    ros.ActuationHost.on_drive(h,message)
    assert not sent
    h.offer['output_ready']=True  # synthetic receiver only; not an output-ready override on hardware.
    ros.ActuationHost.on_drive(h,message)
    assert sent[0]['command']['source_age_us']==5000
    assert r.receive(sent[0]['command'],220_000)[0]
    ros.ActuationHost.on_drive(h,message)
    assert len(sent)==1  # duplicate source not refreshed.
