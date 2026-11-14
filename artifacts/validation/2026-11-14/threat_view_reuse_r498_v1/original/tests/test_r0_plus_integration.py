"""R0+ 명시 연결·기록의 G1/R0 분리. 물리 시험과 별도."""
import ast
from dataclasses import replace
import json
from pathlib import Path

import pytest

from arena_autonomy.follow_trace import FollowTrace, freeze, replay_events
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.r0_plus import R0Plus
from test_follow_trace import controller, context
from test_g1_traffic_eval import verdict_base
import g1_traffic_eval as ev

ROOT = Path(__file__).resolve().parents[1]


def test_r0plus_trace_replays_real_node_and_is_not_old_g1_schema(tmp_path):
    path = tmp_path/'trace.jsonl'
    cfg = FollowConfig(occupancy_timing_mode='per_ray_candidate')
    recorder = FollowTrace(path, context()[1].config, cfg, mode='r0_plus_candidate')
    node = controller(recorder)
    node.follow, node.r0_plus_enabled = R0Plus(cfg), True
    for ref in (.32, .34, .42, .52):
        node.scan = context(ref)[0]
        node.motion.last_meta['reference_stamp_s'] = ref
        node.command(None)
    node.reset_follow('test_reset')
    node.scan.header.frame_id = 'wrong_frame'
    assert node.command(None)[2]['mode'] == 'r0_plus_candidate'
    recorder.close()
    events = [json.loads(line) for line in path.read_text().splitlines()]
    assert events[0]['data']['schema'] == 2
    result = replay_events(events)
    assert result['passed'] and result['matched'] == 5
    assert result['scope'] == 'r0plus_decision_stage_only'
    events[0]['data']['controller_mode'] = 'g1_follow'
    with pytest.raises(ValueError, match='header'):
        replay_events(events)


def test_r0plus_does_not_get_baseline_r0_validity_only_pass():
    base = verdict_base(behavior_rows=2, g1_rows=0)
    base['evaluation']['controller_mode']['r0plus_rows'] = 2
    result = ev.finalize_verdict(base, 'r0plus')
    assert result['run_valid'] and not result['passed']
    assert 'r0_meets_g1_gates' not in result
    assert not ev.finalize_verdict(base, 'g1')['integrity']['controller_mode_matches']
    assert not ev.finalize_verdict(base, 'r0')['integrity']['controller_mode_matches']


def test_r0plus_launch_explicit_no_protection_remap():
    text = (ROOT/'src/arena_bringup/launch/r0_plus.launch.py').read_text()
    assert "SetParameter(name='r0_plus_enabled', value=True)" in text
    assert "SetParameter(name='g1_follow_enabled', value=False)" in text
    assert '/drive/safe' not in text and 'remappings' not in text
    node = (ROOT/'src/arena_autonomy/arena_autonomy/local_pursuit.py').read_text()
    assert "declare_parameter('r0_plus_enabled', False)" in node
    assert 'self.g1_enabled and self.r0_plus_enabled' in node


def test_start_demo_default_stays_same_and_candidates_are_exclusive(monkeypatch):
    import validate_basic_autonomy as basic
    monkeypatch.setattr(basic.subprocess, 'Popen', lambda args, **kw: args)
    assert basic.start_demo_process(None)[4] == 'demo.launch.py'
    assert basic.start_demo_process(None, g1_follow=True)[4] == 'g1_follow.launch.py'
    assert basic.start_demo_process(None, r0_plus=True)[4] == 'r0_plus.launch.py'
    with pytest.raises(ValueError):
        basic.start_demo_process(None, g1_follow=True, r0_plus=True)
