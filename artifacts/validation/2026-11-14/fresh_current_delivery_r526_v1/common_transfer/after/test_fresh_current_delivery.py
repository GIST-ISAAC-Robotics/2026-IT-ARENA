"""공통 current 전달의 시각·조향·소비 계약. ROS 발행/물리 성능 시험은 아니다."""
from pathlib import Path
import ast
from types import SimpleNamespace
import copy
import pytest

from test_runtime_dual_contract import setup_case


def case(angle=.15):
    d, w = setup_case()
    d.last_application = 1.08
    w.ready['output']['steering_rad'] = angle
    return d, w


def poll(d, **changes):
    state = dict(now_s=1.12, previous_steering_rad=.1, side='left', motion_epoch=0,
                 sensor_epoch=0, output_dt_s=.035)
    return d.poll(**{**state, **changes})


def test_changed_angle_early_publication_preserves_source_and_computed_command():
    d, w = case()
    original = copy.deepcopy(w.ready)
    result = poll(d)
    assert result['kind'] == 'candidate' and result['output'][1] == .15
    assert result['command_reference_s'] == 1.1 and w.ready == original
    assert 1.12-d.last_application < .1
    ack = d.acknowledge_publication(published=True, reference_s=1.1, speed_mps=.4,
                                   steering_rad=.15, now_s=1.12)
    assert ack['retained'] and w.ready is None
    assert w.retained['output'] == original['plan']['output']
    assert w.retained['input'] == original['plan']['input']
    assert d.last_application == w.last_application == 1.12
    assert poll(d) == dict(kind='needs_recheck')
    assert d.acknowledge_publication(published=True, reference_s=1.1, speed_mps=.4,
        steering_rad=.15, now_s=1.13)['reason'] == 'no_current_candidate'


@pytest.mark.parametrize('angle,dt', [(.22,.035), (.15,.01), (.15,.05), (.15,None)])
def test_rate_excess_or_unknown_actual_dt_keeps_original_interval(angle, dt):
    d, w = case(angle)
    original = copy.deepcopy(w.ready)
    assert poll(d, output_dt_s=dt) is None
    assert w.ready == original and d.pending_publication is None
    assert poll(d, now_s=1.181, output_dt_s=.1)['kind'] == 'candidate'


def test_even_unchanged_angle_cannot_publish_faster_than_existing_control_period():
    d, w = case(.1)
    d.last_application = 1.105
    w.ready['input']['reference_s'] = w.ready['output']['reference_s'] = 1.11
    assert poll(d, output_dt_s=.014) is None
    assert w.ready is not None and d.pending_publication is None


def test_zero_command_is_not_delayed_by_positive_interval():
    d, w = case()
    w.ready['output']['speed_mps'] = 0.
    w.ready['output']['reason'] = 'current_gap_hold'
    assert poll(d, output_dt_s=0.)['output'][0] == 0.


@pytest.mark.parametrize('change,reason', [
    (dict(now_s=1.31), 'current_source_expired'),
    (dict(previous_steering_rad=.11), 'current_previous_request_changed'),
    (dict(motion_epoch=1), 'current_input_epoch_or_side_changed'),
    (dict(side='right'), 'current_input_epoch_or_side_changed'),
])
def test_existing_freshness_epoch_side_and_previous_checks_still_reject(change, reason):
    d, w = case()
    result = poll(d, **change)
    assert result['kind'] == 'discarded' and result['reason'] == reason
    assert w.ready is None


def test_retained_proof_is_not_invented():
    d, w = case()
    w.ready['route_check']['retained'] = True
    assert poll(d)['reason'] == 'new_current_route_proof_required'


def test_stop_cancels_unpublished_early_candidate():
    d, w = case()
    assert poll(d)['kind'] == 'candidate'
    d.invalidate()
    assert d.pending_publication is None and w.ready is None
    assert d.acknowledge_publication(published=True, reference_s=1.1, speed_mps=.4,
        steering_rad=.15, now_s=1.12)['reason'] == 'no_current_candidate'


def test_nonretained_plan_does_not_gain_early_permission():
    d, w = case()
    plan = copy.deepcopy(w.ready['plan'])
    plan['input']['previous_steering_rad'] = .1
    plan['input']['reference_s'] = 1.1
    plan['input']['scan']['measurements']['acquired_at_s'] = [1.]
    w.saved_plan, w.ready = plan, None
    w.supported = lambda output: False
    assert poll(d) is None and w.saved_plan == plan


def test_existing_100ms_delivery_result_is_unchanged():
    baseline, _ = setup_case()
    baseline.last_application = 1.08
    candidate, _ = case(.22)
    state = dict(now_s=1.181,previous_steering_rad=.1,side='left',motion_epoch=0,sensor_epoch=0)
    assert baseline.poll(**state) == candidate.poll(**state,output_dt_s=.1)


@pytest.mark.parametrize('dual', [False, True])
def test_actual_node_passes_output_dt_only_to_runtime_dual(dual):
    # ROS 모듈을 위조하지 않고 실제 command 메서드의 조기 return 경로를 실행한다.
    path = Path(__file__).parents[1]/'src/arena_autonomy/arena_autonomy/g1_g2_node.py'
    node = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body
                if isinstance(n, ast.ClassDef) and n.name == 'G1G2Node')
    command = next(n for n in node.body if isinstance(n, ast.FunctionDef) and n.name == 'command')
    env = {}
    exec(compile(ast.fix_missing_locations(ast.Module(body=[command], type_ignores=[])),
                 str(path), 'exec'), env)
    calls = []
    class LegacyDelivery:
        def poll(self, now, *, previous_steering_rad, side, motion_epoch, sensor_epoch):
            calls.append(dict(now=now))
            return None
    class DualDelivery:
        def poll(self, now, *, previous_steering_rad, side, motion_epoch, sensor_epoch, output_dt_s=None):
            calls.append(dict(now=now, output_dt_s=output_dt_s))
            return None
    owner = SimpleNamespace(decision_delivery=DualDelivery() if dual else LegacyDelivery(),
        runtime_dual_selected=dual, decision_output_dt_s=.035, now_s=lambda:1.12,
        last_steering=.1, side='left', motion=SimpleNamespace(counts={'resets':0}),
        scan=SimpleNamespace(source_epoch=0))
    assert env['command'](owner, None) is None
    assert calls == [dict(now=1.12, **({'output_dt_s':.035} if dual else {}))]
