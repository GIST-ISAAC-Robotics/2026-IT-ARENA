from pathlib import Path
import sys
import math
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'staged'))
from motor_feedback_runtime import MotorFeedbackRuntime
from arena_vehicle_interface.drive_feedback import MotorEncoderSpec
from arena_vehicle_interface.mcu_status_clock import McuStatusClock

SPEC = MotorEncoderSpec(28, 2.5*81/17, .0325)


def converter(sign=1):
    return MotorFeedbackRuntime(SPEC, encoder_sign=sign, maximum_interval_ns=200_000_000,
                               maximum_age_ns=200_000_000, maximum_speed_mps=5.)


def sample(seq=1, count=0, capture_us=100000, **changes):
    raw = dict(schema='motor_feedback_observation_v1', boot='boot-a', generation=2, clock_epoch=1,
               source_valid=True, sequence=seq, motor_counts=count, mcu_capture_us=capture_us,
               estimated_capture_ns=10_000_000_000+capture_us*1000,
               host_receipt_ns=10_001_000_000+capture_us*1000,
               time_basis='minimum_receipt_offset_mcu_state_estimate', time_contract_verified=False)
    raw.update(changes)
    return {k: raw[k] for k in ('boot', 'generation', 'clock_epoch', 'host_receipt_ns')} | {'motor_feedback':raw}


@pytest.mark.parametrize('delta', [-3, 0, 3])
@pytest.mark.parametrize('sign', [-1, 1])
def test_real_count_difference_not_synthetic_zero(delta, sign):
    c = converter(sign)
    assert c.convert(sample()) is None
    result = c.convert(sample(2, delta, 120000))
    assert result.velocity_rad_s == pytest.approx(sign*delta*math.tau/28/.02)
    assert SPEC.mean_wheel_mps(result.velocity_rad_s) == pytest.approx(sign*delta*math.tau/28/.02/SPEC.gear_ratio*.0325)
    assert result.position_rad == pytest.approx(sign*delta*math.tau/28)
    assert result.motor_counts == delta and result.stamp_ns == 10_120_000_000
    assert result.time_contract_verified is False


def test_repeated_offer_does_not_refresh_encoder_source():
    c=converter(); c.convert(sample())
    assert c.convert(sample(host_receipt_ns=10_110_000_000)) is None
    assert c.reason=='duplicate'
    result=c.convert(sample(3, 2, 140000))
    assert result.velocity_rad_s==pytest.approx(2*math.tau/28/.04)


@pytest.mark.parametrize('fault', ['missing','invalid','future','stale','bad_counts','identity'])
def test_bad_sample_cannot_be_zero_motion_or_bridge_through_invalid(fault):
    c=converter(); c.convert(sample())
    bad=sample(2, 1, 120000)
    if fault=='missing':bad.pop('motor_feedback')
    elif fault=='invalid':bad['motor_feedback']['source_valid']=False
    elif fault=='future':bad['motor_feedback']['estimated_capture_ns']=bad['host_receipt_ns']+1
    elif fault=='stale':bad['motor_feedback']['estimated_capture_ns']-=300_000_000
    elif fault=='bad_counts':bad['motor_feedback']['motor_counts']=False
    elif fault=='identity':bad['generation']+=1
    assert c.convert(bad) is None
    assert c.convert(sample(3, 2, 140000)) is None
    assert c.convert(sample(4, 3, 160000)) is not None


def test_clock_offset_updates_do_not_change_raw_interval_velocity():
    c=converter();c.convert(sample())
    result=c.convert(sample(2, 2, 120000, estimated_capture_ns=10_115_000_000))
    assert result.velocity_rad_s==pytest.approx(2*math.tau/28/.02)
    assert result.stamp_ns==10_115_000_000


@pytest.mark.parametrize('change',[{'boot':'boot-b'},{'generation':3},{'clock_epoch':2}])
def test_identity_change_restarts_difference_and_rejects_retired_source(change):
    c=converter();c.convert(sample())
    assert c.convert(sample(2, 2, 120000,**change)) is None
    assert c.convert(sample(3, 4, 140000,**change)) is not None
    assert c.convert(sample(4, 6, 160000)) is None
    assert c.convert(sample(5, 6, 160000,**change)) is not None


def test_long_gap_reanchors_and_large_jump_does_not_poison_next_velocity():
    c=converter();c.convert(sample())
    assert c.convert(sample(2, 10, 500000)) is None
    assert c.reason=='long_interval'
    assert c.convert(sample(3, 11, 520000)) is not None
    assert c.convert(sample(4, 100000, 540000)) is None
    assert c.reason=='implausible_speed'
    assert c.convert(sample(5, 12, 560000)) is None
    assert c.convert(sample(6, 13, 580000)) is not None


def test_actual_status_decoder_chain_keeps_raw_sample_and_estimate():
    clock=McuStatusClock(warmup_ns=0);c=converter()
    outputs=[]
    for seq,count in [(1,0),(2,2)]:
        now=100000+seq*20000
        raw=dict(type='offer',v=1,boot='mcu-boot',generation=0,state='DISARMED',estop_held=False,
                 output_ready=False,reason='startup',status_now_us=now,lease_issued_us=now,
                 motor_feedback_v1=dict(valid=True,sequence=seq,capture_us=now-1000,counts=count))
        rows=clock.convert_batch([raw],monotonic_ns=now*1000+1000000,realtime_ns=10_000_000_000+now*1000)
        assert len(rows)==1
        outputs.append(c.convert(rows[0]))
    assert outputs[0] is None and outputs[1].velocity_rad_s==pytest.approx(2*math.tau/28/.02)
    assert outputs[1].mcu_capture_us==139000 and outputs[1].motor_counts==2
    assert outputs[1].time_contract_verified is False
