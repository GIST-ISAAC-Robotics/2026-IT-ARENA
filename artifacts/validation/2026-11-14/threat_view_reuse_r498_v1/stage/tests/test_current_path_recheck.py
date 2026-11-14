"""현재 경로 검사 공통 진입점. 기본 실행/명령 발행을 추가하지 않는다."""
import ast
import copy
import importlib
import json
from functools import lru_cache
from pathlib import Path

import pytest

from arena_autonomy import current_path_recheck as check
from arena_autonomy.follow_trace import restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene

ROOT = Path(__file__).resolve().parents[1]
SCOPES = ('solo_baseline', 'solo_or_guard_parallel', 'absent_guard_baseline',
          'released_baseline', 'present_guard_baseline', 'released_parallel')


@lru_cache(maxsize=1)
def samples():
    found = {}
    trace = ROOT/'artifacts/validation/2026-11-12/released_runtime_r168_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as stream:
        header = json.loads(next(stream))['data']
        for event in map(json.loads, stream):
            if event['kind'] != 'call': continue
            item = event['data']; selected = check.scope(item['output'])
            if selected is not None and selected not in found:
                found[selected] = (header, item)
    assert set(found) == set(SCOPES)
    return found


def fixture_for(selected):
    header, item = copy.deepcopy(samples()[selected])
    out = item['output']; old = item['input']; now = copy.deepcopy(old)
    _, scan, history = restore_call(now, MotionConfig(**header['motion_config']))
    scene = motion_scene(scan, history, now['reference_s'], timing_verified=now['timing_verified'],
                         epoch=now['epoch'], rear_half_angle_deg=now['rear_half_angle_deg'])
    return out, old, now, scene, history, FollowConfig(**header['follow_config'])


@pytest.mark.parametrize('selected', SCOPES)
def test_dispatch_recalculates_and_keeps_original_source_and_inputs(selected):
    args = fixture_for(selected); before = copy.deepcopy(args[:3])
    assert check.scope(args[0]) == selected
    result = check.rebase(*args)
    assert result is not None and 0 <= result['speed_mps'] <= args[0][0]
    assert result['path_source_s'] == args[0][2]['normal']['path_source_s']
    assert result['reference_s'] == args[2]['reference_s']
    assert result['independent_guard_required'] and not result['runtime_authorized']
    assert args[:3] == before


@pytest.mark.parametrize('selected', SCOPES)
@pytest.mark.parametrize('change', ('epoch', 'steering', 'expired_path'))
def test_current_epoch_steering_or_source_mismatch_stops(selected, change):
    args = fixture_for(selected)
    if change == 'epoch': args[2]['epoch'] += 1
    elif change == 'steering': args[2]['previous_steering_rad'] += .1
    else: args[0][2]['normal']['path_source_s'] -= 1.
    assert check.rebase(*args)['speed_mps'] == 0.


@pytest.mark.parametrize('selected', SCOPES)
def test_zero_and_stop_cannot_be_revived_by_recheck(selected):
    out = fixture_for(selected)[0]
    out[0] = 0.
    assert not check.supported(out) and check.rebase(out, None, None, None, None, None) is None
    out[0] = .15; out[2]['handoff_state'] = 'STOP'
    assert not check.supported(out) and check.rebase(out, None, None, None, None, None) is None


def test_import_has_no_installer_ros_or_artifact_loading():
    import arena_autonomy.pass_release_trial as selector
    before = (selector.same_path_risk, selector.threat_view)
    importlib.reload(check)
    assert (selector.same_path_risk, selector.threat_view) == before
    for path in Path(check.__file__).parent.glob('*.py'):
        tree = ast.parse(path.read_text(encoding='utf8'))
        imports = [n.module or '' for n in ast.walk(tree) if isinstance(n, ast.ImportFrom)]
        imports += [alias.name for n in ast.walk(tree) if isinstance(n, ast.Import) for alias in n.names]
        assert not any(name.startswith(('rclpy', 'importlib', 'subprocess')) for name in imports)
        assert not any(isinstance(n, ast.FunctionDef) and n.name in ('install', 'worker_entry') for n in tree.body)
