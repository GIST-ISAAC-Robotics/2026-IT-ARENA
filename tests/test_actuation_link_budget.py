"""실제 콜백 봉투/직렬 길이 산술 대조. UART/USB 실측으로 확대하지 않는다."""
import importlib.util
import json
from pathlib import Path
import struct
import sys
from types import SimpleNamespace

import pytest

pytest.importorskip('rclpy')
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
spec = importlib.util.spec_from_file_location('audit_actuation_link_budget', ROOT/'scripts/audit_actuation_link_budget.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)
from arena_vehicle_interface.actuation_wire import Decoder
from arena_vehicle_interface.actuation_ros import ActuationHost


@pytest.fixture(scope='module')
def report():
    return audit.build_report()


def test_actual_envelopes_and_exact_serialized_lengths(report):
    assert len(report['samples']) == 3
    for row in report['samples']:
        for name, payload in row['payloads'].items():
            frame = audit.encode(payload)
            assert len(frame) == row['frame_bytes'][name]
            assert len(frame) == struct.unpack_from('<H', frame, 2)[0] + 8
            assert Decoder().feed(frame) == [payload]
        assert row['payloads']['offer']['feedback_mode'] == 'drive_motor_shaft'
        assert row['payloads']['offer']['left_wheel_mps'] is None
        assert row['payloads']['drive']['type'] == 'command'
        assert row['payloads']['drive']['request'] == ''
        assert row['payloads']['drive']['command']['source_age_us'] == 0


def test_compact_projection_satisfies_only_current_host_offer_schema(report):
    for row in report['samples']:
        compact = row['payloads']['compact_offer_projection_not_implemented']
        obj = SimpleNamespace(decoder=Decoder(), offer=None, offer_wall=0., pending={})
        ActuationHost.on_rx(obj, SimpleNamespace(data=list(audit.encode(compact))))
        assert obj.offer == compact
        assert set(compact) == set(audit.COMPACT_KEYS)
        assert 'mean_wheel_mps' not in compact  # 피드백이 필요한 다른 경로의 대체품은 아니다.


def test_full_duplex_capacity_is_not_sum_of_directions():
    row = audit.budget(100, 100, 100_000)
    assert row['mcu_to_host_load'] == row['host_to_mcu_load'] == .5
    assert row['both_directions_strictly_below_capacity']
    assert row['required_baud_at_unit_load'] == 50_000
    assert row['zero_queue_offer_then_drive_ms'] == 20
    assert row['remaining_lease_zero_queue_ms'] == 80


def test_exact_unit_load_has_no_headroom():
    assert not audit.budget(200, 100, 100_000)['both_directions_strictly_below_capacity']


@pytest.mark.parametrize('baud', audit.BAUDS)
def test_byte_duration_formula_and_max_payload(report, baud):
    assert report['max_legal_frame_bytes'] == 1544
    assert report['max_legal_frame_serialization_ms'][str(baud)] == pytest.approx(15440*1000/baud)
    sizes = report['sample_max_frame_bytes']
    row = audit.budget(sizes['offer'], sizes['drive'], baud)
    assert row['zero_queue_offer_then_drive_ms'] == pytest.approx((sizes['offer']+sizes['drive'])*10000/baud)


@pytest.mark.parametrize('args', [(0, 10, 100000), (10, 0, 100000), (10, 10, 0)])
def test_invalid_arithmetic_inputs(args):
    with pytest.raises(ValueError):
        audit.budget(*args)


def test_sample_and_projection_are_not_hardware_or_worst_case_claims(report):
    for field in ['measured_baud_support', 'native_usb_budget_inferred',
                  'worst_case_all_field_widths_proven', 'transport_changed',
                  'firmware_verified', 'ready_to_drive']:
        assert report[field] is False
    assert report['assumptions']['other_telemetry_acks_logs_excluded']
    assert json.loads(json.dumps(report, allow_nan=False)) == report
