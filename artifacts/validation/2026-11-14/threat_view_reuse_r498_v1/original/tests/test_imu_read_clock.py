"""USB 묶음 수신, boot/시계 경계와 진단 시각의 범위를 검사한다."""
import pytest
from arena_vehicle_interface.imu_read_clock import ImuReadClock


def row(t, boot=1):
    return dict(read_begin_us=t-10, read_end_us=t, boot=boot, gyro_rad_s=[.1,.2,.3])


def test_queued_samples_keep_mcu_spacing_instead_of_one_receipt_stamp():
    c=ImuReadClock(warmup_ns=0)
    r=c.convert_batch([row(1000),row(6000),row(11000)],
                      host_monotonic_ns=2_000_000_000, ros_receipt_ns=10_000_000_000)
    assert [x.stamp_ns for x in r]==[9_990_000_000,9_995_000_000,10_000_000_000]
    assert [x.receipt_age_ns for x in r]==[10_000_000,5_000_000,0]
    assert not c.time_contract_verified


def test_transport_delay_change_does_not_become_gyro_sample_spacing():
    c=ImuReadClock(warmup_ns=0)
    a=c.convert_batch([row(1000)],host_monotonic_ns=2_000_000_000,ros_receipt_ns=10_000_000_000)
    b=c.convert_batch([row(6000)],host_monotonic_ns=2_020_000_000,ros_receipt_ns=10_020_000_000)
    assert b[0].stamp_ns-a[0].stamp_ns==5_000_000
    assert b[0].receipt_age_ns==15_000_000


def test_warmup_new_boot_and_retired_boot_are_separate():
    c=ImuReadClock(warmup_ns=5_000_000)
    r=c.convert_batch([row(1000),row(6000)],host_monotonic_ns=20_000_000,ros_receipt_ns=20_000_000)
    assert len(r)==1
    old=r[0].generation
    assert not c.convert_batch([row(1000,2)],host_monotonic_ns=30_000_000,ros_receipt_ns=30_000_000)
    assert not c.convert_batch([row(11000,1)],host_monotonic_ns=31_000_000,ros_receipt_ns=31_000_000)
    new=c.convert_batch([row(6000,2)],host_monotonic_ns=35_000_000,ros_receipt_ns=35_000_000)
    assert new[0].generation>old and c.counts['old_boot']==1


def test_ros_clock_step_starts_new_warmup():
    c=ImuReadClock(warmup_ns=5_000_000)
    first=c.convert_batch([row(1000),row(6000)],host_monotonic_ns=20_000_000,ros_receipt_ns=20_000_000)
    r=c.convert_batch([row(11000)],host_monotonic_ns=25_000_000,ros_receipt_ns=1_025_000_000)
    assert not r and c.generation>first[0].generation and c.counts['clock_reset']==1


def test_dropped_and_duplicate_frames_are_not_filled_or_zeroed():
    c=ImuReadClock(warmup_ns=0)
    r=c.convert_batch([row(1000),row(1000),row(21000)],
                      host_monotonic_ns=30_000_000,ros_receipt_ns=30_000_000)
    assert len(r)==2 and r[1].stamp_ns-r[0].stamp_ns==20_000_000
    assert r[1].sample['gyro_rad_s']==[.1,.2,.3]
    assert c.counts['nonmonotonic']==1


@pytest.mark.parametrize('bad', [dict(read_begin_us=200,read_end_us=100,boot=1),
    dict(read_begin_us=0,read_end_us=30000,boot=1),
    dict(read_begin_us=True,read_end_us=100,boot=1),
    dict(read_begin_us=90,read_end_us=100,boot=-1)])
def test_invalid_read_metadata_does_not_create_samples(bad):
    c=ImuReadClock(warmup_ns=0)
    assert not c.convert_batch([bad],host_monotonic_ns=100_000,ros_receipt_ns=100_000)
    assert c.counts['invalid']==1
