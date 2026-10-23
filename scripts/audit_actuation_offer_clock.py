#!/usr/bin/env python3
"""별도 시계 offer 사용 상한의 오프라인 후보. wire/ROS/구동 허가와 미연결.

왕복 근거로 발행/만료를 host 구간으로 옮긴 뒤 모든 가능한 나이·도착 시각을
보수적으로 대조한다. 수신 시점부터 임대 수명을 다시 시작하지 않는다.
"""
import argparse
from dataclasses import asdict
import hashlib
import json
from pathlib import Path
import zipfile

from arena_vehicle_interface.clock_bounds import ClockFit, ClockAssumptions, ClockExchange, fit_clock, integer
from arena_vehicle_interface.clock_freshness import HostTimeWindow, assess_freshness

ROOT = Path(__file__).resolve().parents[1]
SOURCES = ('scripts/audit_actuation_offer_clock.py', 'tests/test_actuation_offer_clock.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/clock_bounds.py',
           'src/arena_vehicle_interface/arena_vehicle_interface/clock_freshness.py')
KEYS = {'type', 'v', 'boot', 'generation', 'state', 'owner', 'lease',
        'lease_issued_us', 'lease_expires_us', 'output_ready'}
STATES = {'DISARMED', 'ARMED', 'ACTIVE', 'ESTOP_LATCHED', 'FAULT_LATCHED'}


def text_id(value, name):
    if type(value) is not str or not value.strip() or len(value) > 64:
        raise ValueError(name + ': nonempty identifier of at most 64 codepoints required')
    return value


def assess_offer(offer, fit, received, now, *, mapping_host_epoch, expected_boot, expected_generation,
                 max_lease_us, max_issue_age_ns, forward_delay_upper_ns, policy_note):
    """선언한 시계/지연 가정 아래 시간 조건만 검사. ARM/DRIVE/실물 안전 판정이 아니다.

    forward_delay_upper_ns는 host 판단->MCU 수신 완료까지의 상한(호스트/드라이버 큐,
    직렬화·USB·스케줄 지연 포함)이다. 측정값이나 자동 산정값으로 간주하지 않는다.
    boot/generation 예상값 역시 호출자의 별도 세션 근거이며 최신성을 증명하지 않는다.
    """
    if type(offer) is not dict or set(offer) != KEYS:
        raise ValueError('exact compact offer fields required')
    if offer['type'] != 'offer' or type(offer['v']) is not int or offer['v'] != 1:
        raise ValueError('unsupported offer schema')
    text_id(offer['boot'], 'boot')
    text_id(expected_boot, 'expected boot')
    integer(expected_generation, 'expected generation')
    for field in ('generation', 'lease', 'lease_issued_us', 'lease_expires_us'):
        integer(offer[field], field)
    if offer['lease'] == 0 or type(offer['output_ready']) is not bool:
        raise ValueError('positive lease ID and boolean readiness required')
    if type(offer['state']) is not str or offer['state'] not in STATES:
        raise ValueError('known state required')
    if offer['owner'] is not None:
        text_id(offer['owner'], 'owner')
    if not isinstance(fit, ClockFit) or not isinstance(received, HostTimeWindow) or not isinstance(now, HostTimeWindow):
        raise ValueError('typed clock fit and host windows required')
    if received.host_epoch != mapping_host_epoch or now.host_epoch != mapping_host_epoch:
        raise ValueError('host epoch mismatch')
    if offer['boot'] != expected_boot or fit.boot != expected_boot or offer['generation'] != expected_generation:
        raise ValueError('MCU boot or state generation mismatch')
    integer(max_lease_us, 'maximum lease', 10_000_000)
    integer(max_issue_age_ns, 'maximum issue age')
    integer(forward_delay_upper_ns, 'forward delay upper bound')
    if max_lease_us == 0 or max_issue_age_ns == 0 or type(policy_note) is not str or not policy_note.strip():
        raise ValueError('explicit positive time limits and nonempty policy note required')
    duration = offer['lease_expires_us'] - offer['lease_issued_us']
    if not 0 < duration <= max_lease_us:
        raise ValueError('invalid lease duration')
    issue = fit.map_interval(offer['lease_issued_us'], boot=expected_boot)
    expiry = fit.map_interval(offer['lease_expires_us'], boot=expected_boot)
    # 공개 dataclass 검사는 기존 진단기를 재사용한다. 구간 중간값은 채택하지 않는다.
    age = assess_freshness(issue, now, mapping_host_epoch=mapping_host_epoch,
                           max_age_ns=max_issue_age_ns, inclusive=False)
    receive_order = received.upper_ns <= now.lower_ns and issue.host_upper_ns <= received.lower_ns
    arrival_upper = now.upper_ns + forward_delay_upper_ns
    integer(arrival_upper, 'arrival upper bound')
    before_expiry = arrival_upper < expiry.host_lower_ns
    conditions = dict(receive_order_certain=receive_order,
                      issue_age_certain=age['timestamp_policy_satisfied'],
                      arrival_before_earliest_expiry=before_expiry)
    satisfied = all(conditions.values())
    return dict(schema='arena_actuation_offer_clock_review_v1',
                status='within_declared_time_bounds' if satisfied else 'time_conditions_not_proven',
                timestamp_policy_satisfied=satisfied, conditions=conditions,
                offer=dict(offer), received=asdict(received), now=asdict(now),
                mapped_issue=asdict(issue), mapped_expiry=asdict(expiry), age=age,
                forward_delay_upper_ns=forward_delay_upper_ns, arrival_upper_ns=arrival_upper,
                expiry_margin_lower_ns=expiry.host_lower_ns-arrival_upper,
                policy_note=policy_note, declared_clock_assumptions=asdict(fit.assumptions),
                hardware_assumptions_verified=False, session_evidence_verified=False,
                source_timestamp_checked=False, ready_to_drive=False,
                firmware_verified=False, runtime_connected=False)


def fixture(*, now_ns=9_020_000_000, received_ns=9_010_000_000, delay_ns=5_000_000,
            ppm=0, forward_ns=1_000_000, reverse_ns=9_000_000):
    assumptions = ClockAssumptions(ppm, 0, 0, 5_000_000, 200_000)
    exchange = ClockExchange('boot-A', 1, 9_000_000_000-forward_ns,
                             1_000_000, 1_000_100, 9_000_100_000+reverse_ns)
    fit = fit_clock([exchange], assumptions)
    offer = dict(type='offer', v=1, boot='boot-A', generation=3, state='ACTIVE', owner='host-A',
                 lease=8, lease_issued_us=1_000_100, lease_expires_us=1_100_100, output_ready=True)
    received = HostTimeWindow('host-epoch-A', received_ns, received_ns)
    now = HostTimeWindow('host-epoch-A', now_ns, now_ns)
    options = dict(mapping_host_epoch='host-epoch-A', expected_boot='boot-A', expected_generation=3,
                   max_lease_us=100_000, max_issue_age_ns=100_000_000,
                   forward_delay_upper_ns=delay_ns,
                   policy_note='합성 100 ms 임대·명시 지연 가정. 운영값/실측값 아님')
    return offer, fit, received, now, options


def cases():
    yield 'fresh_asymmetric', fixture(), True
    yield 'delayed_offer_not_reaged', fixture(now_ns=9_120_000_000, received_ns=9_119_000_000), False
    yield 'transmit_queue_80ms', fixture(delay_ns=80_000_000), False
    yield 'wide_asymmetry', fixture(reverse_ns=90_000_000, received_ns=9_095_000_000,
                                   now_ns=9_096_000_000), False
    yield 'drift_bounds', fixture(ppm=200), True
    args = list(fixture())
    expiry = args[1].map_interval(args[0]['lease_expires_us'], boot='boot-A')
    args[3] = HostTimeWindow('host-epoch-A', expiry.host_lower_ns-5_000_000, expiry.host_lower_ns-5_000_000)
    yield 'exact_expiry_rejected', tuple(args), False
    args = list(fixture())
    args[0]['boot'] = 'boot-B'
    yield 'reboot_rejected', tuple(args), None
    args = list(fixture())
    args[0]['generation'] += 1
    yield 'generation_change_rejected', tuple(args), None
    args = list(fixture())
    args[3] = HostTimeWindow('host-epoch-B', 9_020_000_000, 9_020_000_000)
    yield 'host_restart_rejected', tuple(args), None
    args = list(fixture())
    args[0]['lease_issued_us'] += 1_000_000
    args[0]['lease_expires_us'] += 1_000_000
    yield 'mapping_extrapolation_rejected', tuple(args), None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    hashes = {p: hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES}
    results = []
    with zipfile.ZipFile(args.output/'source_snapshot.zip', 'x', zipfile.ZIP_DEFLATED) as bundle:
        for p in SOURCES:
            bundle.write(ROOT/p, p)
    for name, data, expected in cases():
        offer, fit, received, now, options = data
        try:
            report = assess_offer(offer, fit, received, now, **options)
            passed = expected is not None and report['timestamp_policy_satisfied'] is expected
        except ValueError as error:
            report = dict(error=str(error), ready_to_drive=False, runtime_connected=False)
            passed = expected is None
        results.append(dict(name=name, passed=passed, expected=expected, report=report))
    unchanged = all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest() == h for p, h in hashes.items())
    report = dict(scope='offline_synthetic_offer_time_bounds_not_permission', cases=results,
                  passed=all(r['passed'] for r in results) and unchanged, source_sha256=hashes,
                  source_unchanged=unchanged, runtime_connected=False, ready_to_drive=False)
    (args.output/'report.json').write_text(json.dumps(report, indent=2, ensure_ascii=False)+'\n', encoding='utf-8')
    print(json.dumps(dict(passed=report['passed'], cases=len(results), source_unchanged=unchanged)))
    return 0 if report['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
