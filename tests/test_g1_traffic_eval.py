"""G1-b 평가 전용 함수·actor 실행 사본 삽입의 회귀."""
import importlib.util
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
import g1_traffic_eval as ev  # noqa: E402

spec = importlib.util.spec_from_file_location('runtime_world_launch_g1b', ROOT/'src/arena_bringup/launch/simulation.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)


def straight_line(length=20., step=.02):
    count = int(length/step)
    return ev.Centerline([0.]*count, [i*step for i in range(count)], [i*step for i in range(count)], length)


def test_centerline_pose_and_unwrap():
    line = straight_line()
    x, y, yaw = line.pose(3.01)
    assert math.isclose(y, 3.01, abs_tol=1e-9) and math.isclose(yaw, math.pi/2, abs_tol=1e-9)
    assert math.isclose(line.unwrap(.5, 19.5), 20.5)
    assert math.isclose(line.unwrap(19.5, .5), -.5)


def test_schedule_travel_and_validation():
    schedule = [{'from_s': 0., 'to_s': 4., 'speed_mps': .35}, {'from_s': 4., 'to_s': 9., 'speed_mps': 0.},
                {'from_s': 9., 'to_s': 15., 'speed_mps': .35}]
    ev.validate_schedule(schedule, 15.)
    assert ev.actor_travel_m(schedule, -1) == 0.
    assert math.isclose(ev.actor_travel_m(schedule, 6.), 1.4)
    assert math.isclose(ev.actor_travel_m(schedule, 15.), 3.5)
    assert ev.actor_speed_mps(schedule, 5.) == 0. and ev.actor_speed_mps(schedule, 10.) == .35
    with pytest.raises(ValueError):
        ev.validate_schedule([{'from_s': 0., 'to_s': 4., 'speed_mps': .35}, {'from_s': 5., 'to_s': 6., 'speed_mps': 0.}], 6.)
    with pytest.raises(ValueError):
        ev.validate_schedule([{'from_s': 0., 'to_s': 4., 'speed_mps': -.1}], 4.)


def test_sampled_and_swept_gap_agree_for_slow_approach():
    actor = [(0., 2., 0., 0.), (10., 2., 0., 0.)]
    ego = [(i*.02, .5+i*.02*.3, 0., 0.) for i in range(50)]
    sampled = ev.sampled_gaps(ego, actor, .20, .15)
    swept = ev.swept_gap_lower_bounds(ego, actor, .20, .15, .01, 3.)
    assert math.isclose(sampled[0]['gap_m'], 1.3, abs_tol=1e-9)
    lower = min(r['lower_bound_m'] for r in swept)
    assert lower <= min(r['gap_m'] for r in sampled)+1e-9
    assert lower > min(r['gap_m'] for r in sampled)-.01


def test_swept_detects_jump_through_actor_between_samples():
    # 두 표본 모두 actor와 떨어져 있지만 사이에 actor를 통과한다.
    actor = [(0., 1., 0., 0.)]
    ego = [(0., .6, 0., 0.), (.05, 1.4, 0., 0.)]
    sampled = ev.sampled_gaps(ego, actor, .20, .15)
    assert min(r['gap_m'] for r in sampled) > .15
    swept = ev.swept_gap_lower_bounds(ego, actor, .20, .15, .01, 3.)
    assert swept[0]['lower_bound_m'] <= 0


def test_swept_includes_teleporting_actor_window_and_rotation_inflation():
    ego = [(0., 0., 0., 0.), (.02, 0., 0., .2)]
    actor = [(-.01, 1., 0., 0.), (.015, .45, 0., 0.)]
    row = ev.swept_gap_lower_bounds(ego, actor, .20, .15, .01, 3.)[0]
    assert row['actor_poses_in_window'] == 2
    half_diagonal = math.hypot(.20, .15)/2
    assert row['inflation_m'] >= 2*half_diagonal*.2
    missing = ev.swept_gap_lower_bounds(ego, [(.01, 1., 0., 0.)], .20, .15, .01, 3.)[0]
    assert missing['lower_bound_m'] is None


def test_longest_interval_is_time_not_sample_count():
    rows = [{'t': t*.1, 'v': v} for t, v in enumerate([1, 1, 0, 0, 0, 1, 0])]
    assert math.isclose(ev.longest_true_interval(rows, lambda r: r['v'] == 0), .2)
    assert ev.longest_true_interval(rows[:1], lambda r: r['v'] == 1) == 0.


def test_contacts_only_count_ego_pairs():
    contacts = [{'t': 1., 'pairs': [['eval_g1b_actor::body::collision', 'it_arena_track_static::walls::c']]},
                {'t': 2., 'pairs': [['eval_g1b_actor::body::collision', 'arena_car::chassis::collision']]}]
    assert ev.interval_contacts(contacts) == [2.]


def synthetic_run(final_x, hold=True, overlap=False, **changes):
    """정지 actor 앞에서 접근 후 정지하는 기록. 평가기 판정 연결 확인용."""
    config = json.loads((ROOT/'config/tests/g1_traffic_v1.json').read_text(encoding='utf-8'))
    case = next(c for c in config['cases'] if c['name'] == 'stopped_center')
    line = straight_line()
    ego, trace = [], []
    for i in range(0, 1200):
        t = 10.+i*.01
        x = min(1.+(t-10.)*.5, final_x)
        ego.append((t, 0., x, math.pi/2))
        if i % 10 == 0:
            moving = x < final_x
            trace.append({'t': t, 'ground_speed_mps': .5 if moving else 0., 'safe_speed_mps': .5 if moving or not hold else 0.,
                          'autonomy': {'behavior': 'FOLLOW' if moving else 'HOLD', 'mode': 'g1_follow', 'passing_enabled': False,
                                       'reason': 'local_follow' if moving else 'follow_gap_hold'}})
    actor_y = 3.6 if not overlap else final_x+.1
    actor = [(9.+i*.01, 0., actor_y, math.pi/2) for i in range(1400)]
    inputs = dict(config=config, case=case, mode='g1', t_green=10., trace=trace, ego=ego, actor=actor,
                  contacts=[], commands=[], stop_start=20., schedule_pose=lambda t: (0., actor_y),
                  centerline=line, s0=actor_y, road=None, runtime_world="<model name='eval_g1b_actor'>")
    for name, transform in changes.items():
        inputs[name] = transform(inputs[name]) if callable(transform) else transform
    return ev.evaluate_run(**inputs)


def test_evaluate_stopped_case_gates():
    result = synthetic_run(3.2)
    gates = result['gates']
    assert gates['hold'] and gates['approached'] and gates['sampled_gap'] and gates['swept_gap']
    assert gates['ego_stays_behind'] and gates['no_sampled_overlap']
    assert gates['inside_road'] is False  # 노면 다각형이 없으면 통과로 바꾸지 않는다.
    assert result['harness']['valid']
    assert math.isclose(result['gap']['min_sampled_gap_m'], .2, abs_tol=1e-6)
    stalled = synthetic_run(1.2)['gates']
    assert not stalled['approached']
    unsafe = synthetic_run(3.45)['gates']
    assert not unsafe['sampled_gap'] and not unsafe['swept_gap']
    not_held = synthetic_run(3.2, hold=False)['gates']
    assert not not_held['hold']


def test_extended_case_alias_preserves_hold_and_gap_gates():
    config = json.loads((ROOT/'config/tests/g1_traffic_v1.json').read_text(encoding='utf-8'))
    config['acceptance']['occluded_stop'] = config['acceptance']['stopped_center']
    case = {'name': 'occluded_stop', 'behavior_kind': 'stopped_center', 'duration_s': 10., 'schedule': []}
    result = synthetic_run(3.2, config=config, case=case)
    assert result['gates']['hold'] and result['gates']['sampled_gap']
    assert not synthetic_run(3.45, config=config, case=case)['gates']['sampled_gap']


def test_curve_cannot_pass_without_exit_and_heading_change():
    config = json.loads((ROOT/'config/tests/g1_traffic_v1.json').read_text(encoding='utf-8'))
    config['acceptance']['curve_probe'] = {
        **config['acceptance']['stopped_center'], 'required_exit_s_m': 9.8, 'minimum_heading_change_deg': 150.}
    case = {'name': 'curve_probe', 'behavior_kind': 'stopped_center', 'route_kind': 'first_curve',
            'duration_s': 10., 'schedule': []}
    result = synthetic_run(3.2, config=config, case=case)
    assert result['gates']['hold']
    assert not result['gates']['curve_exit_reached']
    assert not result['gates']['curve_heading_changed']


def test_curve_evaluator_accepts_registered_exit_and_turn_evidence():
    # 두 추가 판정의 연결만 검사하는 인공 자세열이며 주행 물리 시험이 아니다.
    config = json.loads((ROOT/'config/tests/g1_traffic_v1.json').read_text(encoding='utf-8'))
    config['acceptance']['curve_probe'] = {
        **config['acceptance']['stopped_center'], 'required_exit_s_m': 2.9, 'minimum_heading_change_deg': 150.}
    case = {'name': 'curve_probe', 'behavior_kind': 'stopped_center', 'route_kind': 'first_curve',
            'duration_s': 10., 'schedule': []}
    result = synthetic_run(3.2, config=config, case=case, s0=1.,
        ego=lambda rows: [(t, x, y, yaw+math.pi*min(1., (t-10.)/10.)) for t, x, y, yaw in rows])
    assert result['gates']['curve_exit_reached'] and result['gates']['curve_heading_changed']
    assert result['case_metrics']['ego_net_heading_change_deg'] == pytest.approx(180.)


def verdict_base(gates=None, behavior_rows=0, g1_rows=0):
    base = {'evaluation': {'harness': {'valid': True}, 'gates': gates or {'hold': False},
                           'controller_mode': {'behavior_rows': behavior_rows, 'g1_rows': g1_rows}},
            'sensor_only_inputs': True, 'safety_sensor_only_inputs': True, 'actuation': {'sensor_only_inputs': True,
            'latched_before_stop_samples': 0, 'stop_latch_timing': {'valid': True}},
            'actor_pose_subscribers': ['g1b_traffic_validator'],
            'drive_publishers': ['local_pursuit'], 'safe_drive_publishers': ['lidar_safety'],
            'sequential_acquisition_verified': True, 'lidar_nominal_interface_passed': True, 'source_unchanged': True,
            'signal_states': ['green', 'red', 'yellow'], 'false_start': False, 'shutdown_clean': True,
            'stop': {'stable': True, 'wall_collision_samples': 0}, 'actuation_stop_passed': True,
            'abort': None, 'actuation_diagnostics': {'control_deadline_samples': 0}, 'wall_collision_samples': 0}
    base['evaluation_readers'] = {'valid': True}
    base['contact_evidence'] = {'validated_source': True}
    return base


def test_finalize_separates_run_validity_and_r0_comparison():
    base = verdict_base()
    r0 = ev.finalize_verdict(json.loads(json.dumps(base)), 'r0')
    assert r0['passed'] and r0['run_valid'] and not r0['r0_meets_g1_gates']
    g1 = ev.finalize_verdict(json.loads(json.dumps(base)), 'g1')
    assert not g1['passed'] and not g1['integrity']['controller_mode_matches']


# --- v3: STOP 요청 직전 /clock 정수 ns와 MCU now_us 정수 비교 ---
REQUEST_NS = 15_720_000_000  # 보존 seed 1 등속 실행의 STOP 요청 ms와 첫 ESTOP 표본 ms


def latch(now_us, state='ESTOP_LATCHED', reason='software_stop'):
    return {'now_us': now_us, 'state': state, 'reason': reason}


def test_v2_float_comparison_reproduces_same_millisecond_stop_as_prior_latch():
    # 보존 실패의 재현: 요청 뒤 자세 시각 15.72초와 같은 ms의 명시 STOP 응답.
    assert 15_720_000*1e-6 < 15.72
    timing = ev.stop_latch_timing([latch(15_715_000, 'ACTIVE', 'drive'), latch(15_720_000), latch(15_725_000)], REQUEST_NS)
    assert timing['valid'] and timing['latched_before_stop_samples'] == 0
    assert timing['same_instant_stop_response'] == 1 and timing['request_clock_us'] == 15_720_000
    assert timing['first_latched'] == latch(15_720_000)


@pytest.mark.parametrize('request_ns,sample_us,before,same', [
    (REQUEST_NS, 15_720_000, 0, 1),              # 같은 µs
    (REQUEST_NS+999, 15_720_000, 0, 1),          # 요청 ns가 같은 µs 안
    (REQUEST_NS, 15_719_999, 1, 0),              # 1 µs 이른 고정은 사유와 무관하게 STOP 전
    (REQUEST_NS, 15_720_001, 0, 0),              # 1 µs 뒤는 요청 이후
    (REQUEST_NS+1000, 15_720_000, 1, 0)])        # 요청이 1 µs 늦으면 같은 표본도 이전
def test_stop_latch_timing_uses_integer_microsecond_boundaries(request_ns, sample_us, before, same):
    timing = ev.stop_latch_timing([latch(sample_us)], request_ns)
    assert timing['valid'] and timing['latched_before_stop_samples'] == before
    assert timing['same_instant_stop_response'] == same


@pytest.mark.parametrize('sample', [latch(15_720_000, 'FAULT_LATCHED', 'control_deadline_missed'),
                                    latch(15_720_000, 'FAULT_LATCHED', 'software_stop'),
                                    latch(15_720_000, 'ESTOP_LATCHED', 'hardware_stop')])
def test_same_instant_fault_or_other_reason_is_still_counted_before_stop(sample):
    timing = ev.stop_latch_timing([sample], REQUEST_NS)
    assert timing['same_instant_other_latch'] == 1 and timing['latched_before_stop_samples'] == 1


def test_real_early_fault_is_counted_even_when_followed_by_explicit_stop():
    trace = [latch(9_000_000, 'FAULT_LATCHED', 'motor_feedback_no_progress'), latch(15_720_000)]
    timing = ev.stop_latch_timing(trace, REQUEST_NS)
    assert timing['latched_before_request'] == 1 and timing['latched_before_stop_samples'] == 1
    assert timing['first_latched']['state'] == 'FAULT_LATCHED'


@pytest.mark.parametrize('now', [None, 15.72e6, float('nan'), float('inf'), True, -1, '15720000'])
def test_missing_or_nonfinite_mcu_time_invalidates_stop_latch_check(now):
    trace = [latch(15_000_000, 'ACTIVE', 'drive'), {'state': 'ACTIVE', 'reason': 'drive', 'now_us': now}]
    timing = ev.stop_latch_timing(trace, REQUEST_NS)
    assert not timing['valid'] and timing['invalid_time_samples'] == 1
    assert ev.stop_latch_timing([{'state': 'ESTOP_LATCHED'}], REQUEST_NS)['invalid_time_samples'] == 1


@pytest.mark.parametrize('request_clock', [None, 15.72e9, True, -1, float('nan')])
def test_missing_or_nonfinite_request_clock_invalidates_stop_latch_check(request_clock):
    timing = ev.stop_latch_timing([latch(1_000_000, 'FAULT_LATCHED', 'boot_fault')], request_clock)
    assert not timing['valid'] and timing['request_clock_us'] is None


class FakeClock:
    """spin마다 정해진 /clock 값을 하나씩 전달하고 벽시계를 진행시키는 시험용 대역."""

    def __init__(self, deliveries, state, step=.01):
        self.deliveries, self.state, self.step, self.wall = list(deliveries), state, step, 100.

    def monotonic(self):
        return self.wall

    def spin(self):
        self.wall += self.step
        if self.deliveries:
            value = self.deliveries.pop(0)
            if value is not None:
                self.state['clock_ns'] = value
                self.state['clock_receipts'] = self.state.get('clock_receipts', 0)+1


def test_stale_stored_clock_is_refreshed_with_two_new_receipts_before_stop():
    # v3 실행: 사례 루프 끝 15.601 s 값이 그래프 감사 동안 남아 요청 직후 자세 15.719 s보다 0.118 s 이전.
    state = {'clock_ns': 15_601_000_000, 'clock_receipts': 50}
    fake = FakeClock([15_717_000_000, 15_719_000_000, 15_720_000_000], state)
    clock, info = ev.fresh_clock_ns(fake.spin, state, fake.monotonic)
    assert clock == 15_719_000_000 and info['fresh'] and info['receipts_after_mark'] == 2


@pytest.mark.parametrize('deliveries', [[], [None]*200, [15_719_000_000]+[None]*200])
def test_missing_or_single_retained_clock_cannot_become_stop_reference(deliveries):
    state = {'clock_ns': 15_601_000_000, 'clock_receipts': 50}
    fake = FakeClock(deliveries, state)
    clock, info = ev.fresh_clock_ns(fake.spin, state, fake.monotonic, timeout_s=1.)
    assert clock is None and not info['fresh'] and info['wait_wall_s'] > 1.
    assert not ev.stop_latch_timing([latch(15_720_000)], clock)['valid']


def test_nonfinite_clock_value_is_not_accepted_as_stop_reference():
    state = {'clock_receipts': 0}
    fake = FakeClock([15.719e9, float('nan')]+[None]*200, state)
    clock, info = ev.fresh_clock_ns(fake.spin, state, fake.monotonic, timeout_s=1.)
    assert clock is None and not info['fresh']


@pytest.mark.parametrize('mode', ['g1', 'r0'])
def test_stop_latch_validity_and_stop_service_result_feed_both_g1_and_r0_verdicts(mode):
    gates = {'hold': True}
    ok = ev.finalize_verdict(verdict_base(gates, 1, 1 if mode == 'g1' else 0), mode)
    assert ok['stop_checks']['no_mcu_latch_before_stop'] and ok['behavior_gates_passed']
    for change in ('invalid_timing', 'latched', 'abort', 'stop_service_failed'):
        report = verdict_base(gates, 1, 1 if mode == 'g1' else 0)
        if change == 'invalid_timing':
            report['actuation']['stop_latch_timing'] = {'valid': False}
        elif change == 'latched':
            report['actuation']['latched_before_stop_samples'] = 1
        elif change == 'abort':
            report['abort'] = 'mcu_latched_before_stop: control_deadline_missed'
        else:
            report['actuation_stop_passed'] = False
            report['stop']['stable'] = False
        result = ev.finalize_verdict(report, mode)
        assert not result['behavior_gates_passed']
        if mode == 'g1':
            assert not result['passed']
        else:
            # R0는 비교 실행이므로 실행 유효성과 G1 판정 충족을 따로 기록한다.
            assert not result['r0_meets_g1_gates']
    missing = verdict_base(gates, 1, 1 if mode == 'g1' else 0)
    del missing['actuation']['stop_latch_timing']
    assert not ev.finalize_verdict(missing, mode)['stop_checks']['no_mcu_latch_before_stop']


def test_registered_config_cases_stay_on_start_straight():
    import validate_g1_traffic as harness
    config = harness.load_config()
    centerline, scene = harness.load_centerline()
    for case in config['cases']:
        for seed in (0, 1, 2):
            s0 = harness.actor_start_s(config, scene, seed)
            result = harness.check_straight(config, centerline, case, s0)
            assert result['actor_end_s_m'] <= result['first_corner_s_m']-1.0
    assert config['acceptance']['minimum_body_gap_m'] == .15
    sdf = ET.fromstring(harness.actor_sdf(config, (10., 6., math.pi/2)))
    model = sdf.find('model')
    assert model.get('name') == 'eval_g1b_actor' and model.findtext('static') == 'true'
    assert model.find("link/sensor[@type='contact']/contact/topic").text == harness.CONTACT_TOPIC
    assert model.find("plugin[@name='gz::sim::systems::PosePublisher']") is not None
    assert float(model.findtext('pose').split()[2]) == pytest.approx(.06)


def test_runtime_world_inserts_only_validated_evaluation_actor(tmp_path):
    import validate_g1_traffic as harness
    source = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
    before = source.read_bytes()
    actor = tmp_path/'actor.sdf'
    actor.write_text(harness.actor_sdf(harness.load_config(), (10., 6., math.pi/2)), encoding='utf-8')
    output = tmp_path/'world.sdf'
    launch._runtime_world_overrides(source, output, 'configured', True, False, False, actor)
    assert source.read_bytes() == before
    world = ET.parse(output).getroot().find('world')
    assert world.find("model[@name='eval_g1b_actor']") is not None
    assert len(world.findall("plugin[@filename='gz-sim-contact-system']")) == 1
    plain = tmp_path/'plain.sdf'
    launch._runtime_world_overrides(source, plain, 'configured', True, False, False)
    assert ET.parse(plain).getroot().find("world/model[@name='eval_g1b_actor']") is None
    for bad in ("<sdf><model name='g1b_actor'><static>true</static></model></sdf>",
                "<sdf><model name='eval_x'><static>false</static></model></sdf>",
                "<sdf><model name='eval_x'><static>true</static></model><plugin filename='gz-sim-imu-system' name='gz::sim::systems::Imu'/></sdf>",
                "<sdf><plugin filename='gz-sim-contact-system' name='gz::sim::systems::Contact'/></sdf>",
                "<sdf><model name='arena_car'><static>true</static></model></sdf>"):
        path = tmp_path/'bad.sdf'
        path.write_text(bad, encoding='utf-8')
        with pytest.raises(RuntimeError):
            launch._runtime_world_overrides(source, tmp_path/'bad_out.sdf', 'configured', True, False, False, path)


def test_comparison_speed_profile_only_changes_cap():
    profiles = launch.SPEED_PROFILES
    assert profiles['g1_compare_1mps'] == {**profiles['exploratory'], 'max_speed_mps': 1.0}


@pytest.mark.parametrize('stream', ['ego', 'actor', 'trace'])
def test_empty_or_truncated_required_stream_is_not_valid(stream):
    empty = synthetic_run(3.2, **{stream: []})
    assert not empty['harness']['valid'] and not empty['gates']['data_coverage']
    truncated = synthetic_run(3.2, **{stream: lambda rows: rows[:10]})
    assert not truncated['harness']['valid'] and not truncated['gates']['data_coverage']


def test_stale_actor_pose_cannot_be_reused_until_case_end():
    result = synthetic_run(3.2, actor=lambda rows: rows[:200])
    assert not result['harness']['checks']['actor_window_covered']
    assert result['gap']['sampled_missing_actor_samples'] > 0
    assert result['gap']['swept_missing_actor_intervals'] > 0
    assert not result['gates']['sampled_gap'] and not result['gates']['ego_stays_behind']


@pytest.mark.parametrize('stream', ['ego', 'actor'])
@pytest.mark.parametrize('invalid', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_pose_cannot_produce_geometry_success(stream, invalid):
    result = synthetic_run(3.2, **{stream: lambda rows: [(rows[0][0], invalid, rows[0][2], rows[0][3]), *rows[1:]]})
    assert not result['harness']['valid'] and result['gates'] == {'data_finite_ordered': False}


@pytest.mark.parametrize('stream', ['ego', 'actor', 'trace'])
def test_reversed_or_duplicate_stream_time_is_invalid(stream):
    result = synthetic_run(3.2, **{stream: lambda rows: [rows[0], rows[0], *rows[1:]]})
    assert not result['harness']['valid'] and not result['gates']['data_finite_ordered']


def test_trace_missing_or_nonfinite_speed_is_not_stationary_evidence():
    for changes in ({'ground_speed_mps': float('nan')}, {'safe_speed_mps': None}):
        result = synthetic_run(3.2, trace=lambda rows: [{**rows[0], **changes}, *rows[1:]])
        assert not result['harness']['valid'] and not result['gates']['data_finite_ordered']


def test_trace_gap_cannot_bridge_a_continuous_hold():
    result = synthetic_run(3.2, trace=lambda rows: [r for r in rows if not 17. < r['t'] < 19.])
    assert not result['harness']['checks']['trace_window_covered']
    assert not result['gates']['hold']
    rows = [{'t': t, 'hold': True} for t in (0., .1, 3., 3.1)]
    assert ev.longest_true_interval(rows, lambda r: r['hold'], max_gap_s=.2) == pytest.approx(.1)


def test_no_commands_or_no_moving_poses_cannot_validate_moving_actor():
    config = json.loads((ROOT/'config/tests/g1_traffic_v1.json').read_text(encoding='utf-8'))
    case = next(c for c in config['cases'] if c['name'] == 'slow_lead')
    missing = synthetic_run(3.2, case=case, actor=[], commands=[])
    assert not missing['harness']['checks']['commands_acknowledged']
    assert not missing['harness']['checks']['moving_tracking_error']
    assert not missing['harness']['valid']


def test_no_contact_messages_are_reported_as_event_only_not_coverage_proof():
    result = synthetic_run(3.2)
    evidence = result['contact_evidence']
    assert evidence['observed_messages'] == 0 and evidence['observed_contact_events_with_ego'] == 0
    assert 'zero messages is not' in evidence['coverage']
    assert 'validated_source' not in evidence


def test_missing_stop_request_cannot_validate_full_case():
    result = synthetic_run(3.2, stop_start=None)
    assert not result['harness']['checks']['case_reached_stop_request']
    assert not result['harness']['valid']


def test_nonfinite_schedule_and_centerline_rejected():
    with pytest.raises(ValueError):
        ev.validate_schedule([{'from_s': 0., 'to_s': float('inf'), 'speed_mps': .35}], 12.)
    with pytest.raises(ValueError):
        ev.Centerline([0., float('nan'), 0.], [0., 1., 2.], [0., 1., 2.], 3.)


def test_contact_topic_subscriber_alone_is_not_publisher_evidence():
    connection = {'returncode': 0, 'reader_alive_at_graph_audit': True,
                  'output': 'Publishers [Address, Message Type]:\nSubscribers [Address, Message Type]:\n tcp://127.0.0.1:2000, gz.msgs.Contacts'}
    assert not ev.contact_connection_valid(connection)
    connection['output'] = 'Publishers [Address, Message Type]:\n tcp://127.0.0.1:2000, gz.msgs.Contacts\nSubscribers [Address, Message Type]:'
    assert ev.contact_connection_valid(connection)
