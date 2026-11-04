"""기억 없는 낮은 속도 재획득 후보. 저장 정체 장면은 원시 기록이 있을 때만 재생한다."""
from dataclasses import asdict, replace
import json
from pathlib import Path

import pytest

from arena_autonomy import local_follow as lf
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT/'artifacts/validation/2026-11-02/official_ops_v1/a5/follow_decisions.jsonl'
stalled = pytest.mark.skipif(not RUN.is_file(), reason='원시 판단 기록은 Git에서 제외된다')


def test_default_off_and_typed():
    assert lf.FollowConfig().standing_wall_reacquire is False
    assert asdict(lf.FollowConfig(standing_wall_reacquire=True))['standing_wall_reacquire'] is True
    with pytest.raises(ValueError):
        lf.FollowConfig(standing_wall_reacquire=1)


def test_parameter_is_declared_off_and_launch_passes_it():
    source = (ROOT/'src/arena_autonomy/arena_autonomy/local_pursuit.py').read_text(encoding='utf-8')
    assert "standing_wall_reacquire=bool(self.declare_parameter('g1_standing_wall_reacquire',False).value)" in source
    launch = (ROOT/'src/arena_bringup/launch/g1_follow.launch.py').read_text(encoding='utf-8')
    assert "DeclareLaunchArgument('g1_standing_wall_reacquire', default_value='false'" in launch


def stalled_calls():
    header, calls = None, []
    with RUN.open(encoding='utf-8') as stream:
        for line in stream:
            try:
                event = json.loads(line)
            except ValueError:
                continue
            if event['kind'] == 'header':
                header = event['data']
            elif event['kind'] == 'call' and 44.9 <= (event['data']['input'].get('reference_s') or 0.) <= 46.:
                calls.append(event['data']['input'])
    return header, calls


def replay(enabled):
    header, calls = stalled_calls()
    config = {key: value for key, value in header['follow_config'].items()}
    motion = MotionConfig(**header['motion_config'])
    follow = lf.LocalFollow(replace(lf.FollowConfig(**config), standing_wall_reacquire=enabled))
    return [replay_call(follow, call, motion) for call in calls]


@stalled
def test_saved_merge_stall_stays_held_when_off():
    outputs = replay(False)
    assert len(outputs) > 20 and all(speed == 0. for speed, _, _ in outputs)
    assert {detail['reason'] for _, _, detail in outputs} == {'follow_boundary_unconfirmed'}


@stalled
def test_saved_merge_stall_reacquires_preferred_wall_after_confirmation_and_still_checks_clearance():
    # 재생의 직전 조향은 기록값에 고정된다. 제자리 조향이 진행돼 출발하는지는 폐루프 시험에서 본다.
    outputs = replay(True)
    assert outputs[0][2]['reason'] == 'follow_boundary_unconfirmed' and outputs[0][2]['bilateral_reference'] == 'standing_preferred_wall'
    later = outputs[1:]
    assert later and all(detail['path_wall'] == 'right' for _, _, detail in later)
    assert {detail['reason'] for _, _, detail in later} == {'follow_steer_in_place'}
    assert all(speed == 0. for speed, _, _ in later)
