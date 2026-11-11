"""통과 확인 뒤 현재 경로의 기존 1mm 검사를 재사용하며 점유/결측 범위는 넓히지 않는다."""
import math
import pytest

import test_traffic_release_fringe as linked


def execute(monkeypatch, *, current=True, proof=True, **kwargs):
    original = linked.output
    def output(*args, **kw):
        v, a, detail = original(*args, **kw)
        detail = {**detail, 'partial_observation': dict(allowed=proof, reason='fully_observed')}
        if current:
            # 공통 합성 scene은 첫 광선0.5초·판단0.6초다.
            detail.update(boundary_from_memory=False, path_source_s=.5)
        return v, a, detail
    monkeypatch.setattr(linked, 'output', output)
    return linked.execute(monkeypatch, retained=False, **kwargs)


@pytest.mark.parametrize('speed', [0., .2, .57])
def test_confirmed_pass_can_use_same_fringe_without_erasing_unknown(monkeypatch, speed):
    ctl, result = execute(monkeypatch, speed=speed)
    detail = result[2]
    assert ctl.mode == 'RELEASED' and result[0] == .35
    assert detail['selector_reason'] == 'released_bounded_rear_fringe'
    assert detail['selector_bounded_rear_fringe_risk_accepted']
    check = detail['selector_confirmed_fringe_recheck']
    assert check['accepted'] and check['measured_speed_mps'] == speed
    assert check['region']['unobserved'] == detail['selector_region']['unobserved'] == 1
    assert check['fringe']['maximum_exterior_m'] == .001
    assert not check['unknown_filled'] and check['fringe']['residual_hidden_object_risk']


@pytest.mark.parametrize('kwargs', [
    {'current': False}, {'proof': False}, {'enabled': False}, {'present': True},
    {'point': (-.047, .076001)}, {'point': (.001, .0753)},
    {'missing': tuple(range(449, 455))}, {'neighbor': .19}, {'neighbor': .4},
    {'value': math.nan}, {'value': -math.inf}, {'value': .02}, {'mismatch': True},
    {'front_mask': 1}, {'tail_extent': .01001}, {'speed': -.01},
    {'target': 0., 'baseline_reason': 'follow_gap_hold'},
    {'target': 0., 'no_path': True, 'baseline_reason': 'follow_no_path'},
])
def test_current_threat_stop_uncertain_path_and_wider_region_still_stop(monkeypatch, kwargs):
    assert execute(monkeypatch, **kwargs)[1][0] == 0.


def test_lower_original_target_does_not_accelerate(monkeypatch):
    assert execute(monkeypatch, target=.2)[1][0] == .2
