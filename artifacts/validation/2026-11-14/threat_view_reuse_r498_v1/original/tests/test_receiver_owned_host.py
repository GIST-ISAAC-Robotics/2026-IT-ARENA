"""선언한 시계 비율 아래의 수명·세대 계약. 장치/ROS 시계 교정 검사는 아니다."""
from fractions import Fraction
import pytest

from receiver_owned_actuation_host import ReceiverLeaseTiming, ReceiverLeaseHostSession, source_time
from arena_vehicle_interface.clock_freshness import HostTimeWindow
from arena_vehicle_interface.actuation_contract import ActuationReceiver, ContractConfig
from audit_actuation_offer_clock import KEYS


def w(ns, upper=None, epoch='epoch'):
    return HostTimeWindow(epoch, ns, ns if upper is None else upper)


def offer(**changes):
    return dict(type='offer', v=1, boot='boot', generation=0, lease=1,
        state='DISARMED', owner=None, lease_issued_us=250_000,
        lease_expires_us=350_000, output_ready=True) | changes


def timing(ppm=1000):
    return ReceiverLeaseTiming(100_000_000, 100_000_000, ppm, '합성 비율 가정; 실물 미검증')


def host(ppm=1000):
    return ReceiverLeaseHostSession(timing=timing(ppm), max_speed_mps=2., max_steering_rad=.45,
        boot='boot', host_epoch='epoch', connection_id='link', client_id='host',
        max_lease_us=100_000, request_timeout_ns=100_000_000)


def record(h, row, ns):
    return h.observe(row, received=w(ns), connection_id='link')


def armed():
    h=host();record(h,offer(),1_000_000_000)
    req=h.begin_control('ARM',w(1_001_000_000))['envelope']
    assert req is not None
    record(h,dict(type='ack',request=req['request'],accepted=True,reason='explicit_arm'),1_002_000_000)
    record(h,offer(generation=1,lease=2,lease_issued_us=255_000,lease_expires_us=355_000,
                   state='ARMED',owner='host'),1_005_000_000)
    assert h.arm_barrier==w(1_005_000_000)
    return h


@pytest.mark.parametrize('ppm',[0,1000,5000])
@pytest.mark.parametrize('age_ns',[0,1,20_000_001,95_000_000])
def test_deadline_not_extended_with_arbitrary_offset_and_clock_rate(ppm,age_ns):
    policy=timing(ppm);now=8_000_000_000;source=now-age_ns
    row=offer()
    report=source_time(row,w(now-3_000_000),w(now),w(source),w(source),policy,100_000)
    assert report['timestamp_policy_satisfied']
    delta=Fraction(ppm,1_000_000)
    for ratio in (1-delta,Fraction(1),1+delta):
        # 실제 offset은 후보에 주지 않고 시험 쪽에서만 MCU 시각을 host로 옮긴다.
        issue_host=now-7_000_000
        deadline_host=issue_host+ratio*1000*(report['receiver_deadline_us']-row['lease_issued_us'])
        assert deadline_host<=source+policy.source_ttl_ns
    assert not report['arrival_before_expiry_proven']


def test_uncertain_source_uses_earliest_source_and_latest_send():
    report=source_time(offer(),w(1_000_000),w(4_000_000,4_010_000),
        w(2_000_000,2_100_000),w(1_500_000),timing(0),100_000)
    assert report['candidate_source_age_us']==2010
    assert report['receiver_deadline_us']==347_990
    assert report['source_deadline_lower_ns']==102_000_000


@pytest.mark.parametrize('source,now,arm',[
    (w(101),w(100),w(10)),
    (w(10),w(100_000_010),w(10)),
    (w(50),w(100),w(51)),
])
def test_future_expired_or_prearm_source_is_not_created(source,now,arm):
    result=source_time(offer(),w(0),now,source,arm,timing(0),100_000)
    assert not result['timestamp_policy_satisfied'] and result['receiver_deadline_us'] is None


def test_new_policy_reuses_ack_generation_and_source_watermark():
    h=armed();s=w(1_006_000_000)
    result=h.prepare_drive(speed_mps=.2,steering_rad=.1,source=s,now=w(1_008_000_000))
    assert result['command_created'] and result['envelope']['command']['seq']==2
    assert result['envelope']['command']['source_age_us']==2003
    assert result['envelope']['command']['ttl_us']==99900
    assert not h.prepare_drive(speed_mps=.2,steering_rad=.1,source=s,now=w(1_009_000_000))['command_created']
    stop=h.begin_control('STOP',w(1_010_000_000))['envelope']
    assert stop['command']==dict(v=1,kind='STOP')
    assert not h.prepare_drive(speed_mps=.2,steering_rad=.1,source=w(1_011_000_000),now=w(1_012_000_000))['command_created']


def test_output_readiness_and_unconfirmed_arm_are_not_bypassed():
    h=host();record(h,offer(output_ready=False),100)
    assert h.begin_control('ARM',w(101))['envelope'] is None
    record(h,offer(lease=2),102)
    req=h.begin_control('ARM',w(103))['envelope']
    record(h,dict(type='ack',request=req['request'],accepted=True,reason='explicit_arm'),104)
    assert h.arm_barrier is None
    assert not h.prepare_drive(speed_mps=.1,steering_rad=0.,source=w(105),now=w(106))['command_created']


def test_stop_needs_no_offer_and_invalid_values_do_not_consume_source():
    assert host().begin_control('STOP',w(1))['envelope']['command']==dict(v=1,kind='STOP')
    h=armed();s=w(1_006_000_000);n=w(1_007_000_000)
    assert not h.prepare_drive(speed_mps=float('nan'),steering_rad=0,source=s,now=n)['command_created']
    assert h.prepare_drive(speed_mps=.1,steering_rad=0,source=s,now=n)['command_created']


def test_epoch_and_generation_changes_revoke_arm():
    h=armed()
    record(h,offer(generation=2,lease=3),1_006_000_000)
    assert not h.prepare_drive(speed_mps=.1,steering_rad=0,source=w(1_007_000_000),now=w(1_008_000_000))['command_created']
    h=armed();h.poll(w(1_006_000_000,epoch='new'))
    assert h.closed and h.arm_barrier is None


@pytest.mark.parametrize('delay_us',[0,1000,20_000,120_000])
def test_actual_reference_receiver_handles_send_queue_delay(delay_us):
    rx=ActuationReceiver(ContractConfig(feedback_mode='drive_motor_shaft',ticks_per_rev=28,
        gear_ratio=2.5*81/17,wheel_radius_m=.0325),boot_id='boot')
    sequence=0
    def tick_to(start,end):
        nonlocal sequence
        for t in range(start,end+1,1000):
            if t%10_000==0:
                rx.update_motor_encoder(t,sequence=sequence,capture_us=t,motor_counts=0);sequence+=1
            rx.tick(t)
    def current_offer(t):
        raw=rx.offer(t)|dict(type='offer',output_ready=True)
        return {k:raw[k] for k in KEYS}
    offset=9_000_000_000
    H=lambda t:offset+t*1000
    h=host(ppm=0);tick_to(0,250_000)
    record(h,current_offer(250_000),H(252_000))
    arm=h.begin_control('ARM',w(H(255_000)))['envelope']
    tick_to(251_000,256_000)
    assert rx.receive(arm['command'],256_000)[0]
    record(h,dict(type='ack',request=arm['request'],accepted=True,reason='explicit_arm'),H(258_000))
    tick_to(257_000,260_000)
    record(h,current_offer(260_000),H(262_000))
    source=H(267_000)
    result=h.prepare_drive(speed_mps=.2,steering_rad=.1,source=w(source),now=w(H(270_000)))
    assert result['command_created']
    arrival=272_000+delay_us
    tick_to(261_000,arrival)
    accepted,_=rx.receive(result['envelope']['command'],arrival)
    if accepted:
        assert H(rx.deadline_us)<=source+100_000_000
    else:
        assert delay_us==120_000
        assert not rx.armed or rx.target_speed==0


def test_age_then_new_lease_rebinding_is_a_real_counterexample():
    # 호출자는 생성한 envelope를 고정해야 한다. 묵은 age와 새 lease를 섞으면 이 증명이 깨진다.
    old=offer();source=8_000_000_000
    r=source_time(old,w(source),w(source),w(source),w(source),timing(0),100_000)
    new_issue_host=source+50_000_000
    incorrectly_rebound_deadline=new_issue_host+1000*(r['candidate_ttl_us']-r['candidate_source_age_us'])
    assert incorrectly_rebound_deadline>source+100_000_000


def test_prepared_wire_is_bound_to_reviewed_lease_not_a_later_offer():
    from arena_vehicle_interface.actuation_wire import Decoder
    h=armed();original=h.review_source
    def review_then_new_offer(*args,**kwargs):
        r=original(*args,**kwargs)
        record(h,offer(generation=1,lease=3,lease_issued_us=270_000,lease_expires_us=370_000,
                       state='ARMED',owner='host'),1_009_000_000)
        return r
    h.review_source=review_then_new_offer
    r=h.prepare_drive(speed_mps=.2,steering_rad=0,source=w(1_006_000_000),now=w(1_008_000_000))
    assert r['envelope']['command']['lease']==2
    wire=r['wire'];r['envelope']['command']['lease']=999
    assert Decoder().feed(wire)[0]['command']['lease']==2
