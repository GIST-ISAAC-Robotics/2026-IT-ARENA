"""수신/원래 header/기록 완료 시계를 혼합하지 않는 오프라인 진단."""
import json
import math

import pytest

import analyze_sensor_probe_timing as tool
from analyze_sensor_probe_timing import Cadence, Moments, ProbeTiming, analyze
from test_sensor_probe_audit import fixture  # 실제 audit의 CDR/receipt 대응 stub 재사용


def row(index, mono, ros, source, wait=1_000_000, topic='/scan', gid=None):
    return dict(topic=topic, index=index, receive_monotonic_ns=mono,
                receive_ros_ns=ros, source_ns=source, writer_monotonic_ns=mono+wait,
                publisher_gid=gid)


def test_regular_cadence_has_independent_epochs_and_no_latency_claim():
    t = ProbeTiming()
    for i in range(4):
        t.observe(row(i+1, 20_000_000_000+i*100_000_000,
                      1_700_000_000_000_000_000+i*100_000_000,
                      50_000_000_000+i*100_000_000))
    r = t.result()['/scan']
    for key in ('callback_monotonic', 'recorder_ros', 'original_header'):
        assert r[key]['span_message_rate_hz'] == 10
        assert r[key]['interval']['mean_ms'] == 100
        assert r[key]['interval']['population_stddev_ms'] == 0
    assert r['callback_to_writer_receipt']['mean_ms'] == 1
    assert r['recorder_ros_span_over_monotonic_span'] == 1
    assert r['publisher_gid_missing'] == 4


def test_bursts_are_visible_even_with_ordered_source_headers():
    t = ProbeTiming()
    for i, mono in enumerate((10, 10, 100_000_010, 100_000_010)):
        t.observe(row(i+1, mono, 1_000_000_000+mono,
                      5_000_000_000+i*100_000_000, (1, 2, 20, 30)[i]*1_000_000))
    r = t.result()['/scan']
    assert r['callback_monotonic']['equal_adjacent'] == 2
    assert r['callback_monotonic']['span_message_rate_hz'] == 30
    assert r['original_header']['span_message_rate_hz'] == 10
    interval = r['callback_monotonic']['interval']
    assert interval['minimum_ms'] == 0 and interval['maximum_ms'] == 100
    assert interval['population_stddev_ms'] == pytest.approx(math.sqrt(20000/9))
    assert r['callback_to_writer_receipt']['mean_ms'] == 13.25
    assert r['callback_to_writer_receipt']['maximum_ms'] == 30


@pytest.mark.parametrize('headers,field,count', [
    ([0, 100, 200], 'zero_stamps', 1), ([-1, 100, 200], 'negative_stamps', 1),
    ([100, 100, 200], 'equal_adjacent', 1), ([100, 50, 200], 'regressions', 1),
])
def test_invalid_header_cadence_is_retained_not_presented_as_sensor_rate(headers, field, count):
    t = ProbeTiming()
    for i, stamp in enumerate(headers):
        t.observe(row(i+1, i*10_000_000, 100+i*10_000_000, stamp))
    r = t.result()['/scan']
    assert r['original_header'][field] == count
    assert r['original_header']['span_message_rate_hz'] is None
    assert r['original_header']['samples'] == 3
    assert r['callback_monotonic']['span_message_rate_hz'] == 100


def test_paused_sim_clock_not_confused_with_wall_rate():
    t = ProbeTiming()
    for i in range(3):
        t.observe(row(i+1, 100+i*10_000_000, 1_000_000_000, 1_000_000_000))
    r = t.result()['/scan']
    assert r['callback_monotonic']['span_message_rate_hz'] == 100
    assert r['recorder_ros']['span_message_rate_hz'] is None
    assert r['original_header']['span_message_rate_hz'] is None
    assert r['recorder_ros_span_over_monotonic_span'] == 0


def test_scaled_sim_clock_is_separate_from_callback_rate():
    t = ProbeTiming()
    for i in range(3):
        t.observe(row(i+1, 100+i*10_000_000, 100+i*20_000_000, 100+i*20_000_000))
    r = t.result()['/scan']
    assert r['callback_monotonic']['span_message_rate_hz'] == 100
    assert r['recorder_ros']['span_message_rate_hz'] == 50
    assert r['recorder_ros_span_over_monotonic_span'] == 2


def test_interleaved_topics_do_not_share_previous_sample():
    t = ProbeTiming()
    for i in range(2):
        t.observe(row(i+1, 100+i*100_000_000, 100+i*100_000_000, 100+i*100_000_000))
        t.observe(row(i+1, 200+i*10_000_000, 200+i*10_000_000, 200+i*10_000_000, topic='/imu/data'))
    assert t.result()['/scan']['callback_monotonic']['span_message_rate_hz'] == 10
    assert t.result()['/imu/data']['callback_monotonic']['span_message_rate_hz'] == 100


@pytest.mark.parametrize('gid', [None, '', '0'*48])
def test_missing_identity_not_proved_single_publisher(gid):
    t = ProbeTiming()
    t.observe(row(1, 1, 1, 1, gid=gid))
    r = t.result()['/scan']
    assert r['publisher_gid_missing'] == 1 and r['publisher_gid_present'] == 0
    assert r['original_header']['span_message_rate_hz'] is None


def test_multiple_known_publishers_disable_header_rate():
    t = ProbeTiming()
    t.observe(row(1, 100, 100, 100, gid='01'*24))
    t.observe(row(2, 200, 200, 200, gid='02'*24))
    r = t.result()['/scan']
    assert r['multiple_known_publishers']
    assert r['original_header']['span_message_rate_hz'] is None
    assert r['callback_monotonic']['span_message_rate_hz'] == 10_000_000


def test_no_samples_and_single_sample_do_not_invent_statistics():
    assert Moments().result()['mean_ms'] is None
    assert Cadence().result()['span_message_rate_hz'] is None
    assert ProbeTiming().result() == {}
    c = Cadence()
    c.add(123)
    assert c.result()['interval']['count'] == 0
    assert c.result()['span_message_rate_hz'] is None


def test_integer_moments_keep_small_variation_at_large_offset():
    m = Moments()
    for n in (10**18, 10**18+1, 10**18+2):
        m.add(n)
    assert m.result()['population_stddev_ms'] == pytest.approx(math.sqrt(2/3)/1e6)
    assert not any(isinstance(v, list) for v in vars(m).values())


@pytest.mark.parametrize('key,value', [
    ('receive_monotonic_ns', True), ('receive_monotonic_ns', 2**63),
    ('receive_ros_ns', float('nan')), ('source_ns', -2**63-1),
    ('writer_monotonic_ns', 0), ('writer_monotonic_ns', True),
    ('publisher_gid', 123), ('publisher_gid', 'xy'), ('publisher_gid', '1'),
    ('topic', ''), ('topic', '/'+('x'*256)),
])
def test_malformed_diagnostic_input_fails_explicitly(key, value):
    r = row(1, 100, 100, 100)
    r[key] = value
    with pytest.raises(ValueError):
        ProbeTiming().observe(r)


def test_analysis_runs_cdr_integrity_before_returning_diagnostics(fixture):
    root, _, _, _, _ = fixture
    r = analyze(root)
    assert r['record_integrity']['record_integrity_verified']
    assert r['per_topic']['/imu/data']['original_header']['zero_stamps'] == 1
    assert r['per_topic']['/imu/data']['callback_to_writer_receipt']['mean_ms'] == .0001
    assert not any(v for k, v in r['interpretation'].items() if k != 'notes')
    assert 'source_age' not in json.dumps(r['per_topic'])


def test_late_digest_failure_does_not_write_partial_timing_report(fixture, tmp_path):
    root, _, _, manifest, save = fixture
    manifest['stats']['/imu/data']['cdr_sha256'] = '0'*64
    save()
    out = tmp_path/'report.json'
    with pytest.raises(ValueError, match='digest'):
        tool.main([str(root), '--output', str(out)])
    assert not out.exists()


def test_existing_output_preserved_without_reading_bag(tmp_path, monkeypatch):
    out = tmp_path/'report.json'
    out.write_text('old evidence')
    monkeypatch.setattr(tool, 'analyze', lambda root: pytest.fail('must not analyze'))
    with pytest.raises(FileExistsError):
        tool.main([str(tmp_path), '--output', str(out)])
    assert out.read_text() == 'old evidence'
