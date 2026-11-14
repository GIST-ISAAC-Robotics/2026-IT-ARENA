"""실제 노드 메서드의 OFF 동등성/ON 현재 기록 연결. 센서/차량 프로세스 없음."""
import copy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace as NS
import zipfile

import pytest

pytest.importorskip('rclpy')
from arena_autonomy import current_path_recheck
from arena_autonomy.follow_trace import equal_output, freeze, restore_call
from arena_autonomy.g1_g2_node import G1G2Node
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig

ROOT = Path(__file__).resolve().parents[1]
ART = ROOT/'artifacts/validation/2026-11-12'


@pytest.fixture(scope='module')
def records():
    trace = ART/'released_parallel_runtime_r174_v1/two_actor_run1/run1/follow_decisions.jsonl'
    with trace.open() as stream:
        header = json.loads(next(stream))['data']
        calls = [e['data'] for e in map(json.loads, stream) if e['kind'] == 'call']
    return header, calls


@pytest.fixture(scope='module')
def original(tmp_path_factory):
    path = tmp_path_factory.mktemp('r177_original')/'node.py'
    with zipfile.ZipFile(ART/'current_recheck_node_r177_v1/before_sources.zip') as z:
        path.write_bytes(z.read('src/arena_autonomy/arena_autonomy/g1_g2_node.py'))
    spec = importlib.util.spec_from_file_location('r177_original_node', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    return module.G1G2Node


def make_node(header, call, enabled):
    current = call.get('current_rebase', {}).get('input', call['input'])
    _, scan, history = restore_call(current, MotionConfig(**header['motion_config']))
    trace = []; resets = []
    item = {**call['delivery'], 'input': copy.deepcopy(call['input']),
            'output': copy.deepcopy(call['output']),
            'result': {**call['compute'], 'output': copy.deepcopy(call['output'])}}
    node = NS(
        decision_delivery=NS(poll=lambda *a, **kw:copy.deepcopy(item), counts={'received':1}),
        current_path_recheck=current_path_recheck if enabled else None,
        now_s=lambda:current['reference_s'], last_steering=current['previous_steering_rad'],
        side=current['side'], scan=scan, rear_blind_half_angle=current['rear_half_angle_deg'],
        decision_output_dt_s=current['dt_s'], decision_command_stamp_s=None,
        motion=NS(counts={'resets':current['epoch']}, history=history,
                  last_meta={'reference_stamp_s':current['reference_s']}, verified=current['timing_verified']),
        follow=NS(config=FollowConfig(**header['follow_config']), reset=lambda:resets.append(True)),
        follow_trace=NS(emit=lambda kind,data:trace.append((kind,freeze(data)))),
        prepared_output=lambda output:output)
    return node, trace, resets


def test_off_keeps_all_473_outputs_stamps_and_trace_identical(records, original):
    header, calls = records
    for call in calls:
        old, old_trace, old_resets = make_node(header, call, False)
        new, new_trace, new_resets = make_node(header, call, False)
        left = original.command(old, []); right = G1G2Node.command(new, [])
        assert equal_output(freeze(left), freeze(right))
        assert equal_output(old_trace, new_trace)
        assert old_resets == new_resets
        assert old.decision_command_stamp_s == new.decision_command_stamp_s


def test_on_reproduces_193_actual_current_outputs_and_keeps_evidence(records):
    header, calls = records; count = 0
    for call in calls:
        if 'current_rebase' not in call: continue
        node, trace, _ = make_node(header, call, True)
        target, steering, detail = G1G2Node.command(node, [])
        expected = call['current_rebase']['output']
        assert (target, steering) == pytest.approx((expected['speed_mps'], expected['steering_rad']))
        assert node.decision_command_stamp_s == expected['reference_s']
        recorded = trace[-1][1]
        assert equal_output(recorded['output'], call['output'])
        assert equal_output(recorded['current_rebase']['input'], call['current_rebase']['input'])
        assert equal_output(recorded['current_rebase']['output'], expected)
        assert detail['current_rebase']['independent_guard_required']
        assert 'path_points' not in detail['current_rebase']
        count += 1
    assert count == 193


@pytest.mark.parametrize('kind', ('pending', 'fault', 'discarded', 'unsupported'))
def test_unavailable_or_unhandled_results_do_not_gain_fresh_stamp(records, kind):
    header, calls = records
    if kind == 'discarded':
        call = next(c for c in calls if c['delivery']['kind'] == 'discarded')
    elif kind == 'unsupported':
        call = next(c for c in calls if c['delivery']['kind'] == 'candidate'
                    and not current_path_recheck.supported(c['output']))
    else: call = calls[0]
    node, trace, resets = make_node(header, call, True)
    if kind == 'pending': node.decision_delivery.poll = lambda *a,**kw:None
    elif kind == 'fault': node.decision_delivery.poll = lambda *a,**kw:{'kind':'fault','reason':'test_fault'}
    result = G1G2Node.command(node, [])
    if kind == 'unsupported':
        assert node.decision_command_stamp_s == call['delivery']['reference_s']
        assert 'current_rebase' not in trace[-1][1]
    else:
        assert node.decision_command_stamp_s is None
        assert result is None if kind != 'fault' else result[0] == 0.
    if kind == 'discarded': assert resets and trace[-1][0] == 'reset'


def test_current_motion_failure_records_zero_instead_of_stale_permission(records):
    header, calls = records
    call = next(c for c in calls if 'current_rebase' in c)
    node, trace, _ = make_node(header, call, True)
    node.motion.history.gyro.times.clear(); node.motion.history.gyro.values.clear()
    result = G1G2Node.command(node, [])
    assert result[0] == 0. and result[2]['reason'] == 'rebase_input_stop'
    assert trace[-1][1]['current_rebase']['output']['independent_guard_required']


@pytest.mark.parametrize('selection', ['off', 'selected_v1'])
def test_launch_keeps_off_as_string_instead_of_yaml_false(selection):
    from launch import LaunchContext
    from launch.actions import DeclareLaunchArgument
    from launch_ros.actions import SetParameter
    path = ROOT/'src/arena_bringup/launch/g1_follow.launch.py'
    spec = importlib.util.spec_from_file_location('r177_launch', path)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations['g1_current_path_recheck'] = selection
    for group in module.generate_launch_description().entities:
        for action in group.get_sub_entities():
            if isinstance(action, (DeclareLaunchArgument, SetParameter)):
                action.execute(context)
            values = dict(context.launch_configurations.get('global_params', []))
            if 'g1_current_path_recheck' in values:
                assert type(values['g1_current_path_recheck']) is str
                assert values['g1_current_path_recheck'] == selection
                return
    pytest.fail('current path recheck launch argument was not set')
