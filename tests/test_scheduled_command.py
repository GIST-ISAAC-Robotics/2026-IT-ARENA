"""시뮬레이션 시계 정수 격자 발행(평가 전용)의 순수 단위 검사와 기본 폴링 경로 불변."""
import json
from pathlib import Path
import random
import sys

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
from braking_command_profile import load_profile, audit_requests  # noqa: E402
import probe_braking_contact as probe  # noqa: E402
from scheduled_command import (  # noqa: E402
    GridSchedule, ProfileSampler, ScheduleError, align_up, audit_scheduled_requests, seconds_to_ns)

MS = 1_000_000
PERIOD = 20*MS
REGISTERED = ROOT/'artifacts/validation/2026-10-08/braking_ab_v1/command_profile.json'


def profile(tmp_path):
    path = tmp_path/'profile.json'
    path.write_text(json.dumps(dict(schema_version=1, interpolation='previous_sample', duration_s=.5,
                                    brake_window_s=[.2, .4],
                                    samples_t_speed_steering=[[0, 0, 0], [.1, .5, .1], [.3, .25, 0], [.336, 0, 0],
                                                              [.5, 0, 0]])))
    return load_profile(path)


@pytest.mark.parametrize('earliest,phase,expected', [
    (0, 0, 0), (1, 0, PERIOD), (PERIOD, 0, PERIOD), (PERIOD-1, 0, PERIOD),
    (0, 10*MS, 10*MS), (10*MS, 10*MS, 10*MS), (10*MS+1, 10*MS, 30*MS), (2_001_000_000, 0, 2_020_000_000),
    (2_001_000_000, 10*MS, 2_010_000_000), (2_000_250_000, 10*MS, 2_010_000_000),
])
def test_align_up_boundaries(earliest, phase, expected):
    value = align_up(earliest, PERIOD, phase)
    assert value == expected and value % PERIOD == phase and value >= earliest


@pytest.mark.parametrize('period,phase', [(PERIOD, -1), (PERIOD, PERIOD), (PERIOD, 10.), (PERIOD, True),
                                          (0, 0), (-PERIOD, 0), (20e6, 0)])
def test_invalid_grid_rejected(period, phase):
    with pytest.raises(ValueError):
        align_up(0, period, phase)
    with pytest.raises(ValueError):
        GridSchedule(period, phase, 0)


def test_origin_off_grid_or_float_clock_rejected():
    with pytest.raises(ValueError):
        GridSchedule(PERIOD, 10*MS, 0)
    with pytest.raises(ValueError):
        align_up(1.5, PERIOD, 0)
    with pytest.raises(ValueError):
        GridSchedule(PERIOD, 0, 0, max_jump_ns=PERIOD-1)
    with pytest.raises(ScheduleError):
        GridSchedule(PERIOD, 0, 0).poll(.02)


def test_exact_boundary_and_no_drift_over_long_run():
    schedule = GridSchedule(PERIOD, 10*MS, 10*MS)
    assert schedule.poll(10*MS-1) is None
    ticks = []
    for now in range(10*MS, 10*MS+60_000*MS+1, 250_000):  # 0.25 ms clock for 60 s
        tick = schedule.poll(now)
        if tick:
            ticks.append(tick)
    assert [t.scheduled_ns for t in ticks] == [10*MS+k*PERIOD for k in range(3001)]
    assert all(t.lateness_ns == 0 and t.missed_before == 0 for t in ticks)
    assert schedule.missed_ticks == 0


def test_duplicate_clock_does_not_publish_twice():
    schedule = GridSchedule(PERIOD, 0, 0)
    assert schedule.poll(0).index == 0
    assert schedule.poll(0) is None
    assert schedule.poll(0) is None
    assert schedule.duplicate_clock_polls == 2
    assert schedule.poll(PERIOD-1) is None
    assert schedule.poll(PERIOD).index == 1


def test_regression_and_large_jump_abort():
    schedule = GridSchedule(PERIOD, 0, 0)
    schedule.poll(5*MS)
    with pytest.raises(ScheduleError):
        schedule.poll(5*MS-1)
    schedule = GridSchedule(PERIOD, 0, 0, max_jump_ns=100*MS)
    schedule.poll(0)
    assert schedule.poll(100*MS) is not None  # 경계값은 허용
    with pytest.raises(ScheduleError):
        schedule.poll(200*MS+1)


def test_late_tick_keeps_grid_and_missed_ticks_are_not_replayed():
    schedule = GridSchedule(PERIOD, 0, 0)
    assert schedule.poll(0).index == 0
    late = schedule.poll(PERIOD+3*MS)
    assert (late.index, late.scheduled_ns, late.lateness_ns, late.missed_before) == (1, PERIOD, 3*MS, 0)
    # 다음 예정은 실제 발행 시각이 아니라 예정 tick 기준이다.
    assert schedule.poll(2*PERIOD-1) is None
    assert schedule.poll(2*PERIOD).index == 2
    # 2.5주기 건너뜀: 가장 최근 tick 하나만, 놓친 2개는 진단으로만 센다.
    skipped = schedule.poll(4*PERIOD+PERIOD//2+PERIOD)
    assert (skipped.index, skipped.missed_before, skipped.lateness_ns) == (5, 2, PERIOD//2)
    assert schedule.poll(4*PERIOD+PERIOD//2+PERIOD) is None
    assert schedule.missed_ticks == 2 and schedule.emitted == 4
    assert schedule.diagnostics()['missed_events'][0]['count'] == 2


def test_float_seconds_map_to_exact_ns():
    assert seconds_to_ns(.1+.2) == 300*MS
    assert seconds_to_ns(.336) == 336*MS
    assert seconds_to_ns(2.256) == 2256*MS
    with pytest.raises(ValueError):
        seconds_to_ns(1e-7)
    assert seconds_to_ns(1e-6) == 1_000


def test_sampler_boundaries_match_profile_previous_sample(tmp_path):
    prof = profile(tmp_path)
    sampler = ProfileSampler(prof)
    assert sampler.at_ns(-1) == (0., 0.)
    assert sampler.at_ns(100*MS-1) == (0., 0.) and sampler.at_ns(100*MS) == (.5, .1)
    assert sampler.at_ns(336*MS-1) == (.25, 0.) and sampler.at_ns(336*MS) == (0., 0.)
    assert sampler.at_ns(500*MS) == (0., 0.) and sampler.at_ns(10_000*MS) == (0., 0.)
    for ns in range(0, 500*MS, 7*MS+13):
        assert sampler.at_ns(ns) == prof.at(ns/1e9)
    with pytest.raises(ValueError):
        sampler.at_ns(.1)


def simulate(prof, phase_ns, arm_ns, step_ns, seed):
    """불규칙한 /clock 수신(1~7 step 묶음)으로 평가기 루프를 흉내 낸다."""
    sampler = ProfileSampler(prof)
    origin = align_up(arm_ns, PERIOD, phase_ns)
    start = align_up(arm_ns+1_000*MS, PERIOD, phase_ns)
    schedule = GridSchedule(PERIOD, phase_ns, origin)
    rng = random.Random(seed)
    rows, now = [], arm_ns
    while now <= start+sampler.duration_ns+100*MS:
        tick = schedule.poll(now)
        if tick:
            speed, steering = sampler.at_ns(tick.scheduled_ns-start)
            rows.append(dict(sim_s=now/1e9, speed_mps=speed, steering_rad=steering, scheduled_ns=tick.scheduled_ns,
                             observed_ns=now, header_ns=now, index=tick.index, missed_before=tick.missed_before))
        now += step_ns*rng.randint(1, 7)
    return rows, sampler, start


@pytest.mark.parametrize('step_ns', [MS, MS//4])
def test_value_sequence_is_phase_and_jitter_independent(tmp_path, step_ns):
    prof = profile(tmp_path)
    sequences = []
    for phase, seed in ((0, 1), (10*MS, 2), (0, 3)):
        rows, sampler, start = simulate(prof, phase, 2_000_250_000, step_ns, seed)
        audit = audit_scheduled_requests(rows, sampler, start, start+sampler.duration_ns, PERIOD, phase)
        assert audit['passed'], audit
        active = [r for r in rows if start <= r['scheduled_ns'] <= start+sampler.duration_ns]
        sequences.append([(r['scheduled_ns']-start, r['speed_mps'], r['steering_rad']) for r in active])
    assert sequences[0] == sequences[1] == sequences[2]


def test_scheduled_audit_negatives(tmp_path):
    prof = profile(tmp_path)
    rows, sampler, start = simulate(prof, 10*MS, 2_000_250_000, MS, 4)
    end = start+sampler.duration_ns
    audit = lambda r, phase=10*MS: audit_scheduled_requests(r, sampler, start, end, PERIOD, phase)
    assert audit(rows)['passed']
    index = next(i for i, r in enumerate(rows) if r['scheduled_ns'] == start+120*MS)
    cases = {
        'every_registered_tick_once': [rows[:index]+rows[index+1:], rows[:index+1]+[dict(rows[index])]+rows[index+1:]],
        'values_match_scheduled_sample': [[*rows[:index], {**rows[index], 'speed_mps': .3}, *rows[index+1:]]],
        'not_backdated': [[*rows[:index], {**rows[index], 'observed_ns': rows[index]['scheduled_ns']-1,
                                           'header_ns': rows[index]['scheduled_ns']-1}, *rows[index+1:]]],
        'header_is_observed_clock': [[*rows[:index], {**rows[index], 'header_ns': rows[index]['scheduled_ns']},
                                      *rows[index+1:]] if rows[index]['observed_ns'] != rows[index]['scheduled_ns'] else
                                     [*rows[:index], {**rows[index], 'header_ns': rows[index]['observed_ns']+1}, *rows[index+1:]]],
        'lateness_within_limit': [[*rows[:index], {**rows[index], 'observed_ns': rows[index]['scheduled_ns']+11*MS,
                                                   'header_ns': rows[index]['scheduled_ns']+11*MS}, *rows[index+1:]]],
        'no_missed_ticks': [[*rows[:index], {**rows[index], 'missed_before': 1}, *rows[index+1:]]],
    }
    for check, variants in cases.items():
        for variant in variants:
            result = audit(variant)
            assert not result['passed'] and not result['checks'][check], check
    assert not audit(rows, phase=0)['checks']['start_on_grid']


def test_actual_time_sample_difference_is_reported_not_hidden(tmp_path):
    prof = profile(tmp_path)
    sampler = ProfileSampler(prof)
    start = 2_020*MS
    rows = []
    for k in range(26):
        scheduled = start+k*PERIOD
        observed = scheduled+(19*MS if k == 16 else 0)  # 336 ms 전환 직전 tick(320 ms)이 늦게 발행됨
        speed, steering = sampler.at_ns(scheduled-start)
        rows.append(dict(speed_mps=speed, steering_rad=steering, scheduled_ns=scheduled, observed_ns=observed,
                         header_ns=observed, missed_before=0))
    result = audit_scheduled_requests(rows, sampler, start, start+sampler.duration_ns, PERIOD, 0, late_limit_ns=PERIOD)
    assert result['passed']
    assert result['actual_time_sample_differs']['count'] == 1


def test_registered_profile_times_lie_on_ns_grid():
    sampler = ProfileSampler(load_profile(REGISTERED))
    assert sampler.duration_ns == 3_500*MS
    assert all(t % MS == 0 for t in sampler.times_ns)


def test_default_poll_path_and_arguments_unchanged():
    assert probe.schedule_arguments('poll', None, True) is None
    assert probe.schedule_arguments('poll', None, False) is None
    assert probe.evaluation_extra() == []
    assert probe.schedule_arguments('sim_grid', '10', True) == {'mode': 'sim_grid', 'period_ns': PERIOD, 'phase_ns': 10*MS}
    assert probe.schedule_arguments('sim_grid', '0', True)['phase_ns'] == 0
    for bad in (('poll', '0', True), ('sim_grid', None, True), ('sim_grid', '10', False), ('sim_grid', '5', True),
                ('other', None, True)):
        with pytest.raises(ValueError):
            probe.schedule_arguments(*bad)


def test_legacy_poll_audit_unchanged_for_old_rows(tmp_path):
    prof = profile(tmp_path)
    # 기존 감사는 실제 발행 시각 기준 float 표본과 비교한다. 이 판정 방식은 바꾸지 않는다.
    rows = [dict(sim_s=10.+i*.02, speed_mps=prof.at(10.+i*.02-10.)[0], steering_rad=prof.at(10.+i*.02-10.)[1])
            for i in range(26)]
    assert audit_requests(rows, prof, 10., 10.5)['passed']


def test_clock_summary_histogram():
    summary = probe.clock_summary([0, MS, 2*MS, 2*MS, 5*MS, 4*MS, 99*MS], 0, 10*MS)
    assert summary['messages'] == 6 and summary['repeats'] == 1 and summary['regressions'] == 1
    assert summary['min_positive_delta_ns'] == MS and summary['max_delta_ns'] == 3*MS


def test_scheduled_contract_chains_without_runtime_change():
    import hashlib
    sha = lambda path: hashlib.sha256((ROOT/path).read_bytes()).hexdigest()
    r5 = json.loads((ROOT/'config/tests/scheduled_command_r5_sha256.json').read_text())
    r6 = json.loads((ROOT/'config/tests/scheduled_command_r6_sha256.json').read_text())
    r7 = json.loads((ROOT/'config/tests/scheduled_command_r7_sha256.json').read_text())
    r7_changes = {**r7['approved_changes'], **r7['approved_evaluation_scripts'], **r7['approved_implementation_changes']}
    # 아래는 r6까지의 이력 검사. 현재 소스에 이르는 r7 연결은 별도 계약 검사에서 확인한다.
    r6_sha = lambda path: r7_changes[path]['before'] if path in r7_changes else sha(path)
    contract = json.loads((ROOT/'config/tests/scheduled_command_r0_sha256.json').read_text())
    parent = json.loads((ROOT/contract['parent_contract']).read_text())
    assert contract['parent_contract'] == 'config/tests/physics_step_r0_sha256.json'
    assert sha(contract['parent_contract']) == contract['parent_contract_sha256']
    # r0/r1은 런타임 소스·구동 플러그인 구현을 바꾸지 않았다(당시 값은 직전 계약과 같다).
    assert contract['approved_changes'] == {}
    assert contract['sha256'] == parent['sha256'] and contract['implementation_sha256'] == parent['implementation_sha256']
    scripts = contract['approved_evaluation_scripts']
    assert set(scripts) == {'scripts/probe_braking_contact.py', 'scripts/scheduled_command.py'}
    assert scripts['scripts/probe_braking_contact.py']['before'] ==         parent['approved_evaluation_scripts']['scripts/probe_braking_contact.py']['after']
    assert scripts['scripts/scheduled_command.py']['before'] is None
    # 6회 실행은 r0 probe를 썼다. 이후 수정판은 r1 후속 계약에 등록한다(r0은 소급 수정하지 않음).
    r1 = json.loads((ROOT/'config/tests/scheduled_command_r1_sha256.json').read_text())
    assert r1['parent_contract'] == 'config/tests/scheduled_command_r0_sha256.json'
    assert sha(r1['parent_contract']) == r1['parent_contract_sha256']
    assert r1['approved_changes'] == {} and r1['sha256'] == contract['sha256']
    assert r1['implementation_sha256'] == contract['implementation_sha256']
    assert set(r1['approved_evaluation_scripts']) == {'scripts/probe_braking_contact.py'}
    fix = r1['approved_evaluation_scripts']['scripts/probe_braking_contact.py']
    assert fix['before'] == scripts['scripts/probe_braking_contact.py']['after']
    # 2026-10-09 r2: launch 추적 인자·플러그인 계측·평가기 변경을 r1 뒤에 연결한다(r0/r1은 소급 수정하지 않음).
    r2 = json.loads((ROOT/'config/tests/scheduled_command_r2_sha256.json').read_text())
    assert r2['parent_contract'] == 'config/tests/scheduled_command_r1_sha256.json'
    assert sha(r2['parent_contract']) == r2['parent_contract_sha256']
    expected = dict(r1['sha256'])
    assert set(r2['approved_changes']) == {'src/arena_bringup/launch/simulation.launch.py'}
    for path, change in r2['approved_changes'].items():
        assert change['before'] == expected[path]
        expected[path] = change['after']
    assert r2['sha256'] == expected
    implementation = dict(r1['implementation_sha256'])
    for path, change in r2['approved_implementation_changes'].items():
        assert change['before'] == implementation.get(path)
        implementation[path] = change['after']
    assert r2['implementation_sha256'] == implementation
    later = r2['approved_evaluation_scripts']
    assert later['scripts/probe_braking_contact.py']['before'] == fix['after']
    assert later['scripts/scheduled_command.py']['before'] == scripts['scripts/scheduled_command.py']['after']
    for path, value in {**r2['sha256'], **r2['implementation_sha256'], **r2['evaluation_tools_sha256']}.items():
        if path in r5['approved_changes']:
            assert value == r5['approved_changes'][path]['before']
        else:
            assert r6_sha(path) == value, path
    # r3/r4는 평가기 GC 옵션·계측 순서만 후속 등록한다. 이전 계약/실행 해시는 보존한다.
    r3 = json.loads((ROOT/'config/tests/scheduled_command_r3_sha256.json').read_text())
    r4 = json.loads((ROOT/'config/tests/scheduled_command_r4_sha256.json').read_text())
    assert r3['parent_contract'] == 'config/tests/scheduled_command_r2_sha256.json'
    assert r4['parent_contract'] == 'config/tests/scheduled_command_r3_sha256.json'
    for successor in (r3, r4):
        assert sha(successor['parent_contract']) == successor['parent_contract_sha256']
        assert successor['approved_changes'] == {}
    probe = 'scripts/probe_braking_contact.py'
    helper = 'scripts/evaluation_gc.py'
    assert r3['approved_evaluation_scripts'][probe]['before'] == later[probe]['after']
    assert r4['sha256'][probe] == r3['approved_evaluation_scripts'][probe]['after']
    assert r4['approved_implementation_changes'][helper]['before'] == r3['implementation_sha256'][helper]
    assert r4['approved_implementation_changes'][helper]['after'] == r4['implementation_sha256'][helper]
    assert r4['sha256']['scripts/scheduled_command.py'] == later['scripts/scheduled_command.py']['after']
    for path, value in {**r4['sha256'], **r4['implementation_sha256'], **r4['evaluation_tools_sha256']}.items():
        change = {**r5['approved_changes'], **r5['approved_evaluation_scripts']}.get(path)
        assert (value == change['before'] if change else r6_sha(path) == value), path
    assert r5['parent_contract'] == 'config/tests/scheduled_command_r4_sha256.json'
    assert sha(r5['parent_contract']) == r5['parent_contract_sha256']
    assert set(r5['approved_changes']) == {'src/arena_bringup/launch/simulation.launch.py'}
    assert set(r5['approved_evaluation_scripts']) == {probe}
    assert set(r5['approved_implementation_changes']) == {'scripts/evaluation_startup.py'}
    for path, value in {**r5['sha256'], **r5['implementation_sha256'], **r5['evaluation_tools_sha256']}.items():
        change = r6['approved_implementation_changes'].get(path)
        assert (value == change['before'] if change else r6_sha(path) == value), path
    assert r6_sha(probe) == r5['approved_evaluation_scripts'][probe]['after']
    assert r6['parent_contract'] == 'config/tests/scheduled_command_r5_sha256.json'
    assert sha(r6['parent_contract']) == r6['parent_contract_sha256']
    assert r6['approved_changes'] == {} and r6['approved_evaluation_scripts'] == {}
    assert set(r6['approved_implementation_changes']) == {'scripts/evaluation_startup.py'}
    for path, value in {**r6['sha256'], **r6['implementation_sha256'], **r6['evaluation_tools_sha256']}.items():
        assert r6_sha(path) == value, path


def test_grid_mode_shares_profile_validity_block():
    """격자 모드도 진단 coverage·최종 정지·요청 감사·발행 그래프를 input_valid에 반영해야 한다."""
    import ast
    tree = ast.parse((ROOT/'scripts/probe_braking_contact.py').read_text(encoding='utf-8'))
    blocks = [node for node in ast.walk(tree) if isinstance(node, ast.If) and isinstance(node.test, ast.Name)
              and node.test.id == 'profile' and any(isinstance(b, ast.If) and isinstance(b.test, ast.Name)
                                                    and b.test.id == 'grid_spec' for b in node.body)]
    assert len(blocks) == 1
    text = ast.unparse(ast.Module(body=[b for b in blocks[0].body if not isinstance(b, ast.If)], type_ignores=[]))
    assert "report['dynamics_coverage']" in text and "report['input_valid'] &=" in text
    assert "report['request_audit']['passed']" in text and "report['drive_graph_valid']" in text


@pytest.mark.parametrize('grid_mode', [False, True])
@pytest.mark.parametrize('failure', [None, 'request', 'dynamics', 'final', 'graph'])
def test_profile_validity_block_behavior(grid_mode, failure):
    """실제 main의 공유 블록을 실행해 각 실패가 두 모드의 input_valid로 전파되는지 검사한다.

    ROS/Gazebo는 실행하지 않는다. 감사 함수 내부는 별도 단위 시험 대상이며 여기서는 배선만 검사한다.
    """
    import ast
    from types import SimpleNamespace
    tree = ast.parse((ROOT/'scripts/probe_braking_contact.py').read_text(encoding='utf-8'))
    block = next(node for node in ast.walk(tree) if isinstance(node, ast.If) and isinstance(node.test, ast.Name)
                 and node.test.id == 'profile' and any(isinstance(b, ast.If) and isinstance(b.test, ast.Name)
                 and b.test.id == 'grid_spec' for b in node.body))
    scope = dict(profile=SimpleNamespace(duration_s=.5), sampler=SimpleNamespace(duration_ns=500*MS),
                 grid_spec={'period_ns': PERIOD, 'phase_ns': 0} if grid_mode else None,
                 start=0., start_ns=0, stop_elapsed=1.5, requests=[], clock_values=[],
                 active=[{'sim_time_s': i*.02} for i in range(75)],
                 final=[{'sim_time_s': 2.+i*.02} for i in range(26)],
                 report={'input_valid': True, 'drive_graph_valid': failure != 'graph',
                         'command_schedule': {'origin_ns': 0}, 'finish_sim_s': 2.5},
                 audit_scheduled_requests=lambda *a: {'passed': failure != 'request'},
                 audit_requests=lambda *a: {'passed': failure != 'request'},
                 clock_summary=lambda *a: {},
                 # 2026-10-09 r2 블록의 추가 자유 변수(전체 tick 감사·추적 끔). 판정 변화는 test_schedule_gate가 검사.
                 audit_all_ticks=lambda *a: {'passed': True}, trace_extra=[], plugin_events=[], trace_audit={},
                 command_trace_audit=lambda *a: {'valid': True})
    if failure == 'dynamics':
        del scope['active'][5:15]
    if failure == 'final':
        scope['final'] = scope['final'][:5]
    exec(compile(ast.Module(body=[block], type_ignores=[]), '<profile-validity-test>', 'exec'), scope)
    assert scope['report']['input_valid'] is (failure is None)
