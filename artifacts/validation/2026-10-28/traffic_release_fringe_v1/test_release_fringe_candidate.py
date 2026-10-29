"""상태 분기는 주입하고 실제 후반 무반환 검사는 유지하는 연결 반례.

차체 경로/센서 인식을 증명하지 않는다. 기존 bounded_rear_fringe와 RELEASED의
연결, 현재 점유/입력/정지 거부가 동일하게 적용되는지만 확인한다.
"""
from dataclasses import replace
import math
import numpy as np
import pytest
import arena_autonomy.pass_release_trial as ps
from test_pass_release_trial import config, output, view
from test_rear_fringe_trial import case


def execute(monkeypatch, *, enabled=True, retained=True, point=(-.047, .0753),
            value=math.inf, neighbor=4., missing=(451, 452), mismatch=False,
            present=False, no_path=False, target=.8, speed=.2,
            front_mask=0, tail_extent=0., baseline_reason='local_follow'):
    _, raw, scene = case(point=point, value=value, neighbor=neighbor,
                         missing=missing, mismatch=mismatch)
    cfg = replace(config(), parallel_hold_trial=True, pass_release_trial=True,
                  guard_curve_trial=True, traffic_recovery_trial=enabled)
    selector = ps.PassReleaseSelector(cfg)
    selector.sides['left'].update(seen=True, passed=True, clear=2, vanish='benign')
    monkeypatch.setattr(ps, 'threat_view', lambda *args: None if no_path else view(present, -.1))
    monkeypatch.setattr(ps, 'band_check', lambda *args: dict(free=not retained))
    monkeypatch.setattr(ps, 'side_extent', lambda *args: 0.)
    monkeypatch.setattr(ps, 'lateral_excursion', lambda *args: .04)
    def region(*args, context=None):
        if context is not None:
            context.update(queries=np.array([point]), free=np.array([False]),
                           masked=np.array([False]), padding=np.full(500, .10))
        return dict(unobserved=1, masked_left=0, masked_right=int(bool(front_mask or tail_extent)),
                    masked_front_samples=front_mask, masked_tail_extent_m=tail_extent)
    monkeypatch.setattr(ps, 'motion_region', region)
    baseline = output(target, .2, baseline_reason)
    # 평행 경로 자체는 진행 불가다. 새 옆차 반례가 GUARD 후보로 우회하지 않도록
    # 여기서는 기존 기하 실패 조건에 해당하지 않는 일반 HOLD를 쓴다.
    parallel = output(0., reason='parallel_hold_wait')
    return selector, selector.select(scene, baseline, parallel, speed, .2, .02, raw_scan=raw)


def test_retained_pass_uses_existing_rear_fringe_and_keeps_unknown(monkeypatch):
    ctl, result = execute(monkeypatch)
    assert ctl.mode == 'RELEASED' and result[0] == pytest.approx(.35)
    detail = result[2]
    assert detail['selector_reason'] == 'released_bounded_rear_fringe'
    assert detail['selector_bounded_rear_fringe_risk_accepted']
    assert detail['selector_rear_fringe']['accepted']
    assert detail['selector_rear_fringe']['residual_hidden_object_risk']
    assert detail['selector_region']['unobserved'] == 1
    assert not detail['selector_unknown_filled']


@pytest.mark.parametrize('kwargs', [
    {'point': (-.047, .076001)}, {'point': (.001, .0753)},
    {'missing': tuple(range(449, 455))}, {'neighbor': .19}, {'neighbor': .4},
    {'value': math.nan}, {'value': -math.inf}, {'value': .02},
    {'mismatch': True}, {'front_mask': 1}, {'tail_extent': .01001},
])
def test_geometric_input_and_mask_counterexamples_still_hold(monkeypatch, kwargs):
    assert execute(monkeypatch, **kwargs)[1][0] == 0.


def test_old_option_off_does_not_get_new_permission(monkeypatch):
    ctl, result = execute(monkeypatch, enabled=False)
    assert ctl.mode == 'GUARD' and result[0] == 0.


def test_recent_clear_pass_does_not_get_retained_exception(monkeypatch):
    _, result = execute(monkeypatch, retained=False, target=.35)
    assert result[0] == 0.


@pytest.mark.parametrize('kwargs', [
    {'present': True}, {'no_path': True, 'target': 0., 'baseline_reason': 'follow_no_path'},
    {'target': 0., 'baseline_reason': 'follow_gap_hold'}, {'speed': -.01},
    {'speed': math.nan}, {'baseline_reason': 'follow_scene_stale', 'target': 0.},
])
def test_reappearance_path_loss_stop_and_fault_are_not_overridden(monkeypatch, kwargs):
    assert execute(monkeypatch, **kwargs)[1][0] == 0.
