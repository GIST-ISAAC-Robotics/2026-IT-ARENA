import struct
from fifo_candidate import FifoAssociator


def row(seq, kind=4, slot=0, raw=100, boot=1, **extra):
    tag = kind*8 + slot*2
    tag |= tag.bit_count() % 2
    payload = (struct.pack('<I', raw) + bytes(12) if kind == 4
               else struct.pack('<hhh', 100, -2, 0) + bytes(10))
    return dict(type=1, boot=boot, sequence=seq, tag_raw=tag, payload_hex=payload.hex(),
                read_begin_us=seq*100+100, read_end_us=seq*100+192,
                host_monotonic_ns=seq*100000, **extra)


def test_streams_keep_one_slot_raw_values_and_distinct_read_times():
    p = FifoAssociator()
    assert p.feed(row(0)) is None
    a, b = p.feed(row(1, 1)), p.feed(row(2, 2))
    assert a.sensor_tick == b.sensor_tick == 100
    assert a.raw_xyz == b.raw_xyz == (100, -2, 0)
    assert a.read_begin_us != b.read_begin_us
    assert a.kind == 'gyro' and b.kind == 'accel'
    assert a.time_basis == 'sensor_ticks_unmapped'


def test_gap_waits_for_timestamp_but_recovers_on_next_complete_slot():
    p = FifoAssociator(); p.feed(row(0))
    assert p.feed(row(2, 1)) is None
    assert p.feed(row(3, 2)) is None
    p.feed(row(4, slot=1, raw=292))
    assert p.feed(row(5, 1, slot=1)).sensor_tick == 292


def test_slot_mismatch_and_duplicate_cannot_reuse_old_timestamp():
    p = FifoAssociator(); p.feed(row(0))
    assert p.feed(row(1, 1, slot=1)) is None
    assert p.feed(row(2, 1)) is None
    p.feed(row(3, raw=292)); assert p.feed(row(4, 1))
    assert p.feed(row(5, 1)) is None
    assert p.feed(row(6, 2)) is None


def test_even_parity_required_and_reset_does_not_carry_old_tick():
    p = FifoAssociator(); p.feed(row(0))
    bad = row(1, 1); bad['tag_raw'] ^= 1
    assert p.feed(bad) is None
    assert p.feed(row(0, 1, boot=2)) is None
    p.feed(row(1, raw=25, boot=2))
    sample = p.feed(row(2, 1, boot=2))
    assert sample.epoch == 1 and sample.sensor_tick == 25


def test_timestamp_wrap_is_not_a_reset():
    p = FifoAssociator(); p.feed(row(0, raw=2**32-100))
    p.feed(row(1, slot=1, raw=92))
    a = p.feed(row(2, 1, slot=1))
    assert a.sensor_tick == 2**32+92 and a.epoch == 0


def test_reversed_time_discards_epoch_until_new_timestamp():
    p = FifoAssociator(); p.feed(row(0, raw=1000))
    assert p.feed(row(1, raw=100)) is None
    assert p.feed(row(2, 1)) is None
    p.feed(row(3, raw=292))
    a = p.feed(row(4, 1))
    assert a.epoch == 1 and a.sensor_tick == 292


def test_fifo_overrun_requires_fresh_timestamp():
    p = FifoAssociator(); p.feed(row(0))
    assert p.feed(row(1, 1, fifo_status=0x40)) is None
    assert p.feed(row(2, 2)) is None
    p.feed(row(3, raw=292))
    assert p.feed(row(4, 1)).sensor_tick == 292


def test_stop_start_same_boot_creates_new_time_epoch_once():
    p = FifoAssociator(); p.feed(row(0, raw=1000))
    initial = p.feed(row(1, 1))
    stopped = row(2); stopped.update(type=2, active=False)
    p.feed(stopped)
    stopped['sequence'] = 3; p.feed(stopped)
    assert p.feed(row(4, 1)) is None
    p.feed(row(5, raw=50))
    restarted = p.feed(row(6, 1))
    assert restarted.sensor_tick == 50 and restarted.epoch == initial.epoch+1
