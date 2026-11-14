"""별도 시계 source_age/TTL의 보수적 단위 환산. 실제 교정/명령 송신 없음."""
from dataclasses import replace
from fractions import Fraction
import math
from pathlib import Path

import pytest

from actuation_source_time_review import review_source_time
from audit_actuation_offer_clock import fixture
from arena_vehicle_interface.actuation_contract import ActuationReceiver, CommandProducer, ContractConfig
from arena_vehicle_interface.clock_bounds import ClockAssumptions, ClockExchange, fit_clock
from arena_vehicle_interface.clock_freshness import HostTimeWindow


def window(ns, upper=None, epoch='host-epoch-A'):
    return HostTimeWindow(epoch, ns, ns if upper is None else upper)


def review(data=None, source=None, arm=None, ttl=100_000_000):
    offer, fit, receipt, now, options = data or fixture(ppm=200)
    return review_source_time(offer, fit, receipt, now,
                              source or window(9_015_000_000), arm or window(8_999_000_000),
                              **options, source_ttl_ns=ttl)


def test_age_rounded_up_ttl_down_and_never_command():
    result = review()
    assert result['timestamp_policy_satisfied']
    assert result['candidate_source_age_us'] == 5002
    assert result['candidate_ttl_us'] == 99980
    assert result['receiver_deadline_us'] == 1_000_100+99980-5002
    assert result['mapped_receiver_deadline']['host_upper_ns'] <= result['source_deadline_lower_ns']
    for key in ('hardware_assumptions_verified', 'source_host_mapping_verified', 'source_watermark_checked',
                'arm_ack_correlated', 'command_created', 'ready_to_drive', 'runtime_connected'):
        assert result[key] is False


def test_shared_clock_exact_units_matches_original_deadline_arithmetic():
    result = review(fixture())
    assert result['candidate_source_age_us'] == 5000
    assert result['candidate_ttl_us'] == 100_000
    assert result['receiver_deadline_us'] == 1_095_100


@pytest.mark.parametrize('source,arm,condition', [
    (window(9_020_000_001), None, 'source_order_certain'),
    (window(9_019_000_000, 9_021_000_000), None, 'source_order_certain'),
    (window(8_900_000_000), None, 'source_age_representable'),
    (window(9_015_000_000), window(9_015_000_001), 'source_after_arm_certain'),
    (window(9_015_000_000), window(9_014_000_000, 9_016_000_000), 'source_after_arm_certain'),
])
def test_future_stale_or_before_arm_not_usable(source, arm, condition):
    result = review(source=source, arm=arm)
    assert result['conditions'][condition] is False
    assert not result['timestamp_policy_satisfied']


def test_source_is_fresh_but_receiver_conservative_deadline_can_already_be_too_close():
    result = review(source=window(8_940_000_000))
    assert result['offer_review']['timestamp_policy_satisfied']
    assert result['conditions']['arrival_before_source_expiry']
    assert not result['conditions']['arrival_before_receiver_deadline']
    assert not result['timestamp_policy_satisfied']


def test_expired_offer_not_rescued_by_new_source():
    data = fixture(now_ns=9_120_000_000, received_ns=9_119_000_000)
    result = review(data, source=window(9_120_000_000))
    assert not result['conditions']['offer_time_conditions']
    assert not result['timestamp_policy_satisfied']


@pytest.mark.parametrize('ttl', [0, True, -1, 1., 10_000_000_001])
def test_invalid_source_ttl(ttl):
    with pytest.raises(ValueError):
        review(ttl=ttl)


def test_too_small_ttl_for_tick_resolution_is_rejected_not_rounded_up():
    result = review(source=window(9_020_000_000), ttl=999)
    assert result['candidate_ttl_us'] == 0
    assert result['conditions']['source_age_representable'] is False
    assert result['receiver_deadline_us'] is None


@pytest.mark.parametrize('which', ['source', 'arm'])
def test_epoch_mismatch_is_error(which):
    kwargs = {which: window(9_015_000_000, epoch='old-host')}
    with pytest.raises(ValueError):
        review(**kwargs)


def test_wider_source_window_never_provides_longer_deadline():
    exact = review(source=window(9_015_000_000))
    wide = review(source=window(9_014_000_000, 9_016_000_000))
    assert wide['candidate_source_age_us'] >= exact['candidate_source_age_us']
    assert wide['receiver_deadline_us'] <= exact['receiver_deadline_us']
    assert wide['source_deadline_lower_ns'] < exact['source_deadline_lower_ns']


def test_large_monotonic_offset_keeps_integer_decision():
    data = list(fixture(ppm=200))
    shift = 2**54+7
    data[1] = replace(data[1], host_anchor_ns=data[1].host_anchor_ns+shift)
    data[2] = replace(data[2], lower_ns=data[2].lower_ns+shift, upper_ns=data[2].upper_ns+shift)
    data[3] = replace(data[3], lower_ns=data[3].lower_ns+shift, upper_ns=data[3].upper_ns+shift)
    first, second = review(), review(data, window(9_015_000_000+shift), window(8_999_000_000+shift))
    for key in ('conditions', 'receiver_deadline_us', 'candidate_ttl_us', 'candidate_source_age_us'):
        assert first[key] == second[key]


@pytest.mark.parametrize('true_ppm', [-200, 0, 200])
@pytest.mark.parametrize('source_error_ns', [0, 500_000])
def test_any_accepted_time_case_expires_no_later_than_original_source_ttl(true_ppm, source_error_ns):
    rate = Fraction(1_000_000+true_ppm, 1_000_000)
    offset = 8_000_000_000
    def truth(mcu): return rate*mcu*1000+offset
    # Timestamp 둥글림도 선언한 1 ns / 1 us 오차 범위에 둔다.
    assumptions = ClockAssumptions(200, 1, 1, 5_000_000, 200_000)
    exchange = ClockExchange('boot-A', 1, math.floor(truth(1_000_000))-1_000_000,
                             1_000_000, 1_000_100, math.ceil(truth(1_000_100))+9_000_000)
    fit = fit_clock([exchange], assumptions)
    accepted = rejected = 0
    for decision_age in (20, 40, 70, 90):
        now_ns = math.ceil(truth(1_000_100))+decision_age*1_000_000
        data = list(fixture(now_ns=now_ns, received_ns=math.ceil(truth(1_000_100))+10_000_000))
        data[1] = fit
        for source_age_ms in (0, 5, 20, 50, 99, 110):
            source = now_ns-source_age_ms*1_000_000
            report = review(data, window(source-source_error_ns, source+source_error_ns), arm=window(8_800_000_000))
            if report['timestamp_policy_satisfied']:
                accepted += 1
                deadline = truth(report['receiver_deadline_us'])
                assert report['offer_review']['arrival_upper_ns'] < deadline
                assert deadline <= source+100_000_000
                assert report['candidate_source_age_us']*1000*rate >= source_age_ms*1_000_000
                assert report['candidate_ttl_us']*1000*rate <= 100_000_000
            else:
                rejected += 1
    assert accepted and rejected


def test_python_motor_shaft_receiver_uses_same_candidate_deadline():
    config = ContractConfig(feedback_mode='drive_motor_shaft', ticks_per_rev=28,
                            gear_ratio=15., wheel_radius_m=.0325)
    receiver = ActuationReceiver(config, boot_id='boot-A')
    producer = CommandProducer(client_id='host-A')
    for stamp in range(0, 1_000_001, 10_000):
        receiver.update_motor_encoder(stamp, sequence=stamp//10_000, capture_us=stamp, motor_counts=0)
        receiver.tick(stamp)
    assert receiver.receive(producer.control(receiver.offer(1_000_000), 'ARM'), 1_000_000)[0]
    issued = receiver.offer(1_000_100)
    data = list(fixture(ppm=200))
    for field in ('generation', 'lease', 'lease_issued_us', 'lease_expires_us', 'state', 'owner'):
        data[0][field] = issued[field]
    data[4]['expected_generation'] = issued['generation']
    result = review(data)
    assert result['timestamp_policy_satisfied']
    for stamp in (1_010_000, 1_020_000, 1_025_000):
        receiver.update_motor_encoder(stamp, sequence=stamp, capture_us=stamp, motor_counts=0)
        receiver.tick(stamp)
    command = producer.drive(issued, speed_mps=.5, steering_rad=.1, source_us=0, host_now_us=5000)
    # 시험 fixture에서만 필드를 대입한다. 후보 자체는 command를 생성하지 않는다.
    command.update(source_age_us=result['candidate_source_age_us'], ttl_us=result['candidate_ttl_us'])
    assert receiver.receive(command, 1_025_000)[0]
    assert receiver.deadline_us == result['receiver_deadline_us']


def test_source_timing_candidate_not_imported_by_production():
    import ast
    root = Path(__file__).resolve().parents[1]
    for package in ('arena_autonomy', 'arena_vehicle_interface'):
        for path in (root/'src'/package/package).glob('*.py'):
            for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
                if isinstance(node, ast.ImportFrom):
                    assert node.module != 'actuation_source_time_review'
                elif isinstance(node, ast.Import):
                    assert all(n.name != 'actuation_source_time_review' for n in node.names)
