"""실제 normal handler 원문과 합성 바이트/시계의 제한 검사. 장치 시험 아님."""
import json
import random
import shutil
from types import SimpleNamespace

import pytest

from audit_c1_normal_decoder import (
    SOURCE, PINNED, audit, build, chunk, decode, fixture_text, geometry, node, summary,
)
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.lidar_point_review import project_acquisition_points


@pytest.fixture(scope='module')
def executable(tmp_path_factory):
    return build(SOURCE, tmp_path_factory.mktemp('normal_decoder')/'native')


def test_wire_encoder_known_vector():
    # 90 degrees = Q6 5760; quality 40, sync=1; 1 m = Q2 4000.
    assert node(5760, 4000, 40, True).hex() == 'a1012da00f'


@pytest.mark.parametrize('angle,dist,quality,sync', [
    (0, 0, 0, False), (5760, 4000, 40, True), (11520, 65535, 63, False),
    (17280, 1, 1, False), (23039, 48000, 17, True),
])
def test_native_types_units_and_order(executable, angle, dist, quality, sync):
    rows = decode(executable, [chunk(node(angle, dist, quality, sync))])
    assert geometry(rows) == [((angle*256//90) % 65536, dist, quality*4, int(sync))]
    assert rows[0]['sdk_estimate_us'] == 1_000_000-408
    assert rows[-1] == dict(kind='end', points=1, pending_bytes=0, input_bytes=5, answer_type=129)


@pytest.mark.parametrize('split', range(1, 20))
def test_every_two_chunk_boundary_preserves_original_order(executable, split):
    raw = b''.join(node(a*64, d, sync=i == 0) for i, (a, d) in enumerate(
        [(270, 11), (0, 22), (90, 33), (180, 44)]))
    rows = decode(executable, [chunk(raw[:split]), chunk(raw[split:])])
    assert geometry(rows) == [(49152, 11, 160, 1), (0, 22, 160, 0),
                              (16384, 33, 160, 0), (32768, 44, 160, 0)]
    assert [r['sequence'] for r in rows[:-1]] == [0, 1, 2, 3]
    for r in rows[:-1]:
        assert r['completion_chunk_offset'] in (0, split)


def test_seeded_arbitrary_chunk_boundaries(executable):
    rng = random.Random(1234)
    values = [(rng.randrange(23040), rng.randrange(65536), rng.randrange(64), i % 50 == 0)
              for i in range(200)]
    raw = b''.join(node(*v) for v in values)
    actions, offset = [], 0
    while offset < len(raw):
        n = rng.randrange(1, 34)
        actions.append(chunk(raw[offset:offset+n], 1_000_000+offset))
        offset += n
    rows = decode(executable, actions)
    assert geometry(rows) == [(a*256//90, d, q*4, int(s)) for a, d, q, s in values]
    assert rows[-1]['pending_bytes'] == 0


@pytest.mark.parametrize('tail', range(1, 5))
def test_partial_tail_waits_until_completed(executable, tail):
    raw = node(4000, 12000)
    truncated = decode(executable, [chunk(raw[:tail])])
    assert truncated[-1]['points'] == 0
    assert truncated[-1]['pending_bytes'] == tail
    completed = decode(executable, [chunk(raw[:tail]), chunk(raw[tail:], 2_000_000)])
    assert completed[0]['decode_clock_us'] == 2_000_000
    assert completed[-1]['points'] == 1


def test_reset_is_explicit_not_automatically_inferred_from_time_gap(executable):
    raw = node(4000, 12000)
    resumed = decode(executable, [chunk(raw[:2]), chunk(raw[2:], 3_000_000)])
    assert resumed[-1]['points'] == 1  # SDK has no timeout rule here.
    reset = decode(executable, [chunk(raw[:2]), 'reset', chunk(raw, 3_000_000)])
    assert geometry(reset) == geometry(resumed)
    assert reset[0]['action'] == 2


@pytest.mark.parametrize('baud,sample,link,interface,delay', [
    (460800, 200, 0, 0, 408), (0, 200, 0, 0, 734), (460800, 201, 23, 0, 432),
    (460800, 200, 0, 1, 400), (460800, 200, 0, 2, 408),
])
def test_native_delay_formula_branches(executable, baud, sample, link, interface, delay):
    rows = decode(executable, [chunk(node(4000, 12000))], sample_us=sample, baud=baud,
                  linkage_us=link, interface=interface)
    assert rows[0]['sdk_estimate_us'] == 1_000_000-delay


def test_decode_clock_not_acquisition_clock(executable):
    raw = node(4000, 12000)*50
    batch = decode(executable, [chunk(raw)])
    slow_cpu = decode(executable, [chunk(raw, step_us=1)])
    spaced = decode(executable, [chunk(raw[:5], 1_000_000+i*200) for i in range(50)])
    assert geometry(batch) == geometry(spaced) == geometry(slow_cpu)
    assert summary(batch)['equal_stamps'] == 49
    assert summary(slow_cpu)['stamp_span_us'] == 49
    assert summary(spaced)['stamp_span_us'] == 9800


def test_native_underflow_and_clock_reversal_remain_visible(executable):
    raw = node(4000, 12000)
    assert summary(decode(executable, [chunk(raw, 100)]))['invalid_future_stamps'] == 1
    backwards = decode(executable, [chunk(raw, 2000), chunk(raw, 1000)])
    assert summary(backwards)['backwards_stamps'] == 1


def test_zero_quality_and_distance_not_silently_filtered(executable):
    assert geometry(decode(executable, [chunk(node(4000, 0, quality=0))])) == [(4000*256//90, 0, 0, 0)]


def test_header_valid_distance_corruption_not_detected(executable):
    raw = bytearray(node(4000, 12000))
    raw[4] ^= 0x40
    rows = decode(executable, [chunk(raw)])
    assert rows[-1]['points'] == 1
    assert rows[0]['distance_q2'] != 12000


@pytest.mark.parametrize('prefix', ['00000303', 'a200000000'])
def test_sync_and_checkbit_invalid_fixture_resynchronizes(executable, prefix):
    raw = node(4000, 12000)
    assert geometry(decode(executable, [chunk(bytes.fromhex(prefix)+raw)])) == geometry(
        decode(executable, [chunk(raw)]))


def test_still_no_timing_permission_for_native_callback(executable):
    rows = decode(executable, [chunk(node(4000, 12000)*2)])
    args = dict(history=SimpleNamespace(config=SimpleNamespace(scan_timeout_s=.5)),
                ranges=[3., 3.], angles_rad=[0., .01],
                stamps_s=[r['sdk_estimate_us']/1e6 for r in rows[:-1]], now=1.01,
                range_min=.05, range_max=12.)
    with pytest.raises(MotionUnavailable, match='scan_timing_unverified'):
        project_acquisition_points(**args)
    # Test-only assertion: even accidentally claiming verified does not invent distinct times.
    with pytest.raises(MotionUnavailable, match='point_acquisition_order_invalid'):
        project_acquisition_points(**args, timing_verified=True)


@pytest.mark.parametrize('bad', [
    dict(data='aa', clock_us=True, step_us=0), dict(data='a', clock_us=1, step_us=0),
    dict(data='AA', clock_us=1, step_us=0), dict(data='aa', clock_us=-1, step_us=0),
    dict(data='aa', clock_us=1, step_us=-1), dict(data='a a', clock_us=1, step_us=0),
    dict(data='aa', clock_us=2**64-1, step_us=1), dict(data='aa', clock_us=1, step_us=0, trusted=True),
])
def test_malformed_fixture_rejected_before_native(bad):
    with pytest.raises(ValueError):
        fixture_text([bad])


def test_limits_and_empty_stream(executable):
    assert decode(executable, [])[-1]['points'] == 0
    with pytest.raises(ValueError):
        fixture_text(['reset']*10001)
    with pytest.raises(ValueError):
        fixture_text([chunk(b'\x00'*65536)]*3)


def test_hash_check_before_compile_and_existing_output(tmp_path):
    source = tmp_path/'source'
    shutil.copytree(SOURCE, source)
    first = next(iter(PINNED))
    (source/first).write_bytes(b'changed')
    with pytest.raises(ValueError, match='hash mismatch'):
        build(source, tmp_path/'build')
    assert not (tmp_path/'build').exists()
    with pytest.raises(FileExistsError):
        audit(SOURCE, tmp_path)


def test_saved_evidence_has_no_hardware_or_operational_approval():
    report = json.loads((SOURCE.parent/'c1_normal_decoder_v1/report.json').read_text())
    assert report['hardware_verified'] is report['ready_to_drive'] is report['timing_verified'] is False
    assert all(report['checks'].values())
