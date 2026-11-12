import copy
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

A = Path(__file__).resolve().parent
sys.path.insert(0, str(A.parent/'current_deceleration_r334_v1'))
from deceleration import CurrentDecelerationProcess
spec = importlib.util.spec_from_file_location('r334_test_records', A.parent/'current_deceleration_r334_v1/test_deceleration.py')
fixture = importlib.util.module_from_spec(spec)
spec.loader.exec_module(fixture)
from ros_delivery import Delivery


def setup_case():
    old, new, current = fixture.records()
    old['output'][2]['normal']['path_source_s'] = .9
    current['plan'] = copy.deepcopy(old)
    current['route_check'] = dict(revision=8, retained=False)
    current['input'] = copy.deepcopy(old['input'])
    current['input'].update(reference_s=1.1, previous_steering_rad=.1)
    current['input']['scan']['measurements']['acquired_at_s'] = [1.]
    w = fixture.worker(old)
    w.ready, w.current_input, w.retained = current, None, None
    w.current = SimpleNamespace(inflight=None)
    w.last_application = -float('inf')
    w.geometry_age_limit = .5
    d = Delivery(w, lambda: 1.12)
    d.collect = lambda: None
    return d, w


def poll(d):
    return d.poll(1.12, previous_steering_rad=.1, side='left', motion_epoch=0, sensor_epoch=0)


def ack(d, **changes):
    args = dict(published=True, reference_s=1.1, speed_mps=.4, steering_rad=.22, now_s=1.12)
    args.update(changes)
    return d.acknowledge_publication(**args)


def test_poll_does_not_claim_publication_or_discard_ready():
    d, w = setup_case()
    result = poll(d)
    assert result['kind'] == 'candidate'
    assert w.ready is not None and w.retained is None
    assert d.pending_publication is not None
    assert w.last_application == -float('inf')


def test_checked_publication_only_then_retains_original_route():
    d, w = setup_case(); old = copy.deepcopy(w.ready['plan'])
    poll(d); result = ack(d)
    assert result['retained']
    assert w.ready is None and w.retained['input'] == old['input']
    assert w.retained['output'] == old['output']
    assert w.last_application == d.last_application == 1.12
    assert 'route_check' not in w.retained
    assert ack(d)['reason'] == 'no_current_candidate'


@pytest.mark.parametrize('changes', [dict(published=False), dict(speed_mps=0.),
    dict(speed_mps=.6), dict(reference_s=1.12), dict(steering_rad=.21), dict(speed_mps=float('nan'))])
def test_unpublished_or_different_command_cannot_retain_route(changes):
    d, w = setup_case(); poll(d)
    assert not ack(d, **changes)['retained']
    assert w.ready is None and w.retained is None


def test_late_ack_does_not_renew_expired_proof():
    d, w = setup_case(); poll(d)
    result = ack(d, now_s=1.31)
    assert not result['retained'] and result['reason'].startswith('post_publication_')
    assert w.ready is None and w.retained is None


def test_duplicate_poll_cannot_apply_unacknowledged_candidate():
    d, _ = setup_case(); poll(d)
    with pytest.raises(RuntimeError, match='acknowledgement'): poll(d)


def test_stop_between_selection_and_publish_cancels_pending_ack():
    d, w = setup_case(); poll(d)
    def invalidated():
        w.ready = w.retained = w.saved_plan = None
        w.revision += 1
    w.invalidate = invalidated
    d.invalidate()
    assert d.pending_publication is None and not ack(d)['retained']


def test_retained_path_requests_fresh_current_check_not_old_command():
    d, w = setup_case(); poll(d); ack(d)
    assert poll(d) == dict(kind='needs_recheck')
