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


def test_node_command_m3_trace_reset_and_fault_replay(tmp_path):
    cfg = FollowConfig(occupancy_timing_mode='per_ray_candidate')
    execution = ExecutionConfig()
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


def test_actual_ros_constructor_keeps_protection_boundary(tmp_path):
    node = make_node(g1_follow_enabled='true', g1_m3_execution_trial='true',
                     g1_occupancy_timing_mode='per_ray_candidate', g1_trace_path=str(tmp_path/'ros_trace.jsonl'))
    try:
        assert isinstance(node.follow, M3ExecutionFollow)
        assert controller_mode(node.follow) == 'm3_execution_trial'
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
    assert '/drive/safe' not in text
