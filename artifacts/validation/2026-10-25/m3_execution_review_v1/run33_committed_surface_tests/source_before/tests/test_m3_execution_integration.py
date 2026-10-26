"""선택형 M3 노드·기록·평가기 모드의 연결. 주행 성공과 별도다."""
import copy
import json
from pathlib import Path

import pytest

from arena_autonomy.follow_trace import FollowTrace, controller_mode, replay_events
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig
from test_follow_trace import context, controller
from test_g1_traffic_eval import verdict_base
from test_race_session_node import make_node, close, topics
import g1_traffic_eval as ev


@pytest.mark.parametrize('fallback, release, small', [(False, False, False), (True, False, False), (True, True, False),
                                                     (True, True, True)])
def test_node_command_m3_trace_reset_and_fault_replay(tmp_path, fallback, release, small):
    cfg = FollowConfig(occupancy_timing_mode='per_ray_candidate')
    execution = ExecutionConfig(command_fallback=fallback, release_to_baseline=release, small_center_straight=small)
    path = tmp_path/'trace.jsonl'
    recorder = FollowTrace(path, context()[1].config, cfg, mode='m3_execution_trial', execution_config=execution)
    node = controller(recorder)
    node.follow = M3ExecutionFollow(cfg, execution, allow_simulation_trial=True)
    for ref in (.32, .34, .42, .52):
        node.scan = context(ref)[0]
        node.motion.last_meta['reference_stamp_s'] = ref
        node.command(None)
    node.reset_follow('test_reset')
    node.scan.header.frame_id = 'wrong_frame'
    assert node.command(None)[2]['mode'] == 'm3_execution_trial'
    recorder.close()
    events = [json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['data']['schema'] == 3
    assert events[0]['data']['execution_config']['command_fallback'] is fallback
    assert events[0]['data']['execution_config']['release_to_baseline'] is release
    assert events[0]['data']['execution_config']['small_center_straight'] is small
    result = replay_events(events)
    assert result['passed'] and result['matched'] == 5
    assert result['scope'] == 'm3_execution_decision_stage_only'
    wrong = copy.deepcopy(events)
    wrong[0]['data']['controller_mode'] = 'g1_follow'
    with pytest.raises(ValueError, match='header'):
        replay_events(wrong)
    with pytest.raises(ValueError, match='typed'):
        FollowTrace(tmp_path/'invalid.jsonl', context()[1].config, cfg, mode='m3_execution_trial')
    assert not (tmp_path/'invalid.jsonl').exists()


def test_m3_mode_cannot_receive_r0_validity_only_pass():
    for mode in ('g1', 'r0', 'r0plus', 'm3'):
        base = verdict_base(behavior_rows=2, g1_rows=0)
        base['evaluation']['controller_mode']['m3_rows'] = 2
        result = ev.finalize_verdict(base, mode)
        assert result['integrity']['controller_mode_matches'] == (mode == 'm3')
        if mode == 'm3':
            assert result['run_valid'] and not result['passed']
            assert 'r0_meets_g1_gates' not in result


@pytest.mark.parametrize('fallback, release, small', [(False, False, False), (True, False, False), (True, True, False),
                                                     (True, True, True)])
def test_actual_ros_constructor_keeps_protection_boundary(tmp_path, fallback, release, small):
    node = make_node(g1_follow_enabled='true', g1_m3_execution_trial='true',
                     g1_m3_command_fallback=str(fallback).lower(), g1_m3_release_to_baseline=str(release).lower(),
                     g1_m3_small_center_straight=str(small).lower(),
                     g1_occupancy_timing_mode='per_ray_candidate', g1_trace_path=str(tmp_path/'ros_trace.jsonl'))
    try:
        assert isinstance(node.follow, M3ExecutionFollow)
        assert controller_mode(node.follow) == 'm3_execution_trial'
        assert node.follow.execution.command_fallback is fallback
        assert node.follow.execution.release_to_baseline is release
        assert node.follow.execution.small_center_straight is small
        subscriptions, publishers = topics(node)
        assert '/drive' in publishers and '/drive/safe' not in publishers
        assert not any('eval_' in p or 'pose' in p for p in subscriptions)
        node.control()  # 센서 없음: 실제 노드 관문은 0 출력 유지
        assert node.last_speed == 0.
    finally:
        close(node)


@pytest.mark.parametrize('override', [
    dict(g1_follow_enabled='false'), dict(use_sim_time='false'), dict(g1_trace_path=None),
    dict(g1_occupancy_timing_mode='oldest'),
    dict(g1_corridor_fallback_enabled='true', g1_boundary_evidence_mode='corner_distance_candidate',
         g1_recent_observation_enabled='true'),
])
def test_actual_ros_constructor_rejects_incompatible_m3(tmp_path, override):
    parameters = dict(g1_follow_enabled='true', g1_m3_execution_trial='true',
                      g1_occupancy_timing_mode='per_ray_candidate', g1_trace_path=str(tmp_path/'trace.jsonl'))
    parameters.update(override)
    if parameters.get('g1_trace_path') is None:
        parameters.pop('g1_trace_path')  # ROS의 빈 CLI 문자열 파싱 대신 노드의 기본 빈 값으로 검사
    with pytest.raises(ValueError, match='M3'):
        make_node(**parameters)


def test_default_launch_does_not_enable_m3():
    root = Path(__file__).resolve().parents[1]
    text = (root/'src/arena_bringup/launch/g1_follow.launch.py').read_text()
    assert "DeclareLaunchArgument('g1_m3_execution_trial', default_value='false'" in text
    assert "DeclareLaunchArgument('g1_m3_command_fallback', default_value='false'" in text
    assert "DeclareLaunchArgument('g1_m3_release_to_baseline', default_value='false'" in text
    assert "DeclareLaunchArgument('g1_m3_small_center_straight', default_value='false'" in text
    assert '/drive/safe' not in text


def test_command_fallback_without_m3_is_rejected():
    with pytest.raises(ValueError, match='M3'):
        make_node(g1_m3_command_fallback='true')


def test_baseline_release_without_m3_is_rejected():
    with pytest.raises(ValueError, match='M3'):
        make_node(g1_m3_release_to_baseline='true')


def test_small_center_requires_m3_and_observed_release(tmp_path):
    with pytest.raises(ValueError, match='M3'):
        make_node(g1_m3_small_center_straight='true')
    with pytest.raises(ValueError, match='release'):
        make_node(g1_follow_enabled='true', g1_m3_execution_trial='true', g1_m3_command_fallback='true',
                  g1_m3_small_center_straight='true', g1_occupancy_timing_mode='per_ray_candidate',
                  g1_trace_path=str(tmp_path/'trace.jsonl'))


@pytest.mark.parametrize('flag', ['--m3-command-fallback', '--m3-release-to-baseline', '--m3-small-center-straight'])
def test_validator_rejects_m3_options_without_m3_mode(tmp_path, flag, monkeypatch, capsys):
    import validate_g1_traffic as vt
    monkeypatch.setattr('sys.argv', ['validate_g1_traffic.py', '--case', 'slow_lead', '--seed', '0', '--mode', 'g1',
                                     flag, '--output', str(tmp_path/'out')])
    with pytest.raises(SystemExit) as stop:
        vt.main()
    assert stop.value.code == 2 and 'requires --mode m3' in capsys.readouterr().err
    assert not (tmp_path/'out').exists()


def test_validator_small_center_requires_observed_release(tmp_path, monkeypatch, capsys):
    import validate_g1_traffic as vt
    monkeypatch.setattr('sys.argv', ['validate_g1_traffic.py', '--case', 'stop_restart', '--seed', '0', '--mode', 'm3',
                                     '--m3-command-fallback', '--m3-small-center-straight', '--record-follow-decisions',
                                     '--occupancy-timing', 'per_ray_candidate', '--output', str(tmp_path/'out')])
    with pytest.raises(SystemExit) as stop:
        vt.main()
    assert stop.value.code == 2 and '--m3-release-to-baseline' in capsys.readouterr().err
    assert not (tmp_path/'out').exists()


def test_actual_m3_output_survives_compaction_and_mode_check():
    """run9에서 요약 trace가 simulation_trial_only를 버리던 결함의 입출력 회귀."""
    cfg = FollowConfig(occupancy_timing_mode='per_ray_candidate')
    node = controller(None)
    node.follow = M3ExecutionFollow(cfg, allow_simulation_trial=True)
    detail = node.command(None)[2]
    status = ev.compact_autonomy_status(detail)
    assert status['mode'] == 'm3_execution_trial' and status['simulation_trial_only'] is True
    assert status['passing_enabled'] is False
    assert ev.controller_mode_summary([status])['m3_rows'] == 1
    for missing in ('simulation_trial_only', 'passing_enabled'):
        bad = dict(status)
        bad.pop(missing)
        assert ev.controller_mode_summary([bad])['m3_rows'] == 0
