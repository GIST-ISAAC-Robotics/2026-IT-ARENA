"""콜백 발생량과 직렬화 산술만 검증한다. DDS/펌웨어/전기 출력 시험이 아니다."""
from pathlib import Path
import sys

import pytest

pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from audit_actuation_callback_budget import build_report, sample_schedule


@pytest.fixture(scope='module')
def report():
    return build_report()


@pytest.mark.parametrize('scenario,offers,commands,acks,rejected', [
    ('idle', 50, 0, 0, 0), ('drive50', 100, 50, 0, 0),
    ('duplicate50', 100, 50, 0, 49), ('drive50_ack', 100, 50, 50, 0)])
def test_periodic_and_on_command_offers_are_additive(report, scenario, offers, commands, acks, rejected):
    row = next(r for r in report['scenarios'] if r['scenario'] == scenario)
    assert row['duration_us'] == 1000000
    count = lambda kind: sum(s['frames'] for s in row['summary'] if s['kind'] == kind)
    assert (count('offer'), count('command'), count('ack')) == (offers, commands, acks)
    assert row['rejected_command_callbacks'] == rejected
    assert row['non_wire_publications']['status_ros_only'] == 200 + commands
    assert sum(s['frames'] for s in row['summary'] if s['source'] == 'timer') == 50


def test_trace_byte_totals_and_full_duplex_load(report):
    for row in report['scenarios']:
        for direction in ('mcu_to_host', 'host_to_mcu'):
            total = sum(event['bytes'] for event in row['trace'] if event['direction'] == direction)
            assert row['byte_counts'][direction] == total
            for baud in row['baud_load']:
                assert baud[direction] == pytest.approx(total * 10 / baud['baud'])
    row = next(r for r in report['scenarios'] if r['scenario'] == 'drive50')
    loads = {v['baud']: v for v in row['baud_load']}
    assert loads[460800]['mcu_to_host'] > 1
    assert loads[921600]['mcu_to_host'] < 1


def test_rejected_duplicates_generate_traffic_but_never_refresh_deadline(report):
    row = next(r for r in report['scenarios'] if r['scenario'] == 'duplicate50')
    assert row['final_state'] == 'FAULT_LATCHED' and row['final_reason'] == 'command_expired'


def test_nominal_fixture_remains_active_and_not_hardware_proof(report):
    for row in report['scenarios']:
        if row['scenario'] in ('drive50', 'drive50_ack'):
            assert row['final_state'] == 'ACTIVE'
    for field in ('runtime_changed', 'real_transport_measured', 'deadline_proven', 'firmware_verified', 'ready_to_drive'):
        assert report[field] is False


def test_unknown_schedule_rejected():
    with pytest.raises(ValueError):
        sample_schedule('not-a-schedule')
