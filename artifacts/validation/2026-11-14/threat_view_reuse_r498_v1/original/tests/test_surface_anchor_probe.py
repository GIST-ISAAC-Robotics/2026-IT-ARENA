import numpy as np
import pytest
from unittest.mock import patch
import probe_surface_anchor as module
from probe_surface_anchor import SurfaceAnchorProbe
from test_partial_surface_alignment import room


def stationary(points,h,source,reference):
    return np.asarray(points).copy()


def started(retained=None):
    points,ids=room();probe=SurfaceAnchorProbe()
    if retained is None:retained=points[[30,130]]
    probe.start(retained,points,ids,1.,.9)
    return probe,points,ids


def test_two_new_sources_and_no_input_mutation():
    probe,p,ids=started();before=p.copy()
    with patch.object(module,'transport_points',stationary):
        first=probe.observe(p,ids,None,1.1,1.)
        duplicate=probe.observe(p,ids,None,1.12,1.)
        second=probe.observe(p,ids,None,1.2,1.1)
    assert not first['matched'] and not duplicate['matched']
    assert duplicate['consecutive_fresh_sources']==1
    assert second['matched'] and second['unknown_filled'] is False
    np.testing.assert_array_equal(p,before)


def test_lost_object_is_not_replaced_by_visible_wall():
    probe,p,ids=started(np.array([[.35,.02]]))
    with patch.object(module,'transport_points',stationary):
        for t in (1.1,1.2,1.3):
            result=probe.observe(p,ids,None,t,t-.1)
            assert result['fit']['accepted'] and not result['matched']
            assert result['nearest_distances_m'][0]>.3


def test_failed_scan_does_not_replace_anchor_or_accumulate_rejected_shift():
    probe,p,ids=started();original=probe.anchor
    rejected=dict(accepted=False,reason='weak_improvement')
    with patch.object(module,'transport_points',stationary),patch.object(module,'surface_translation',return_value=rejected):
        assert probe.observe(p+.01,ids,None,1.1,1.)['reason']=='surface_rejected'
    assert probe.anchor is original
    with patch.object(module,'transport_points',stationary):
        assert probe.observe(p,ids,None,1.2,1.1)['consecutive_fresh_sources']==1


def test_local_surface_and_missing_adjacency_reject():
    for p,ids in [(room()[0][:30],room()[1][:30]),(room()[0],room()[1]*2)]:
        probe,_,_=started()
        with patch.object(module,'transport_points',stationary):
            result=probe.observe(p,ids,None,1.1,1.)
        assert not result['matched'] and result['reason']=='surface_rejected'


@pytest.mark.parametrize('now,source',[(.99,.95),(1.1,.89),(1.1,1.2),(float('nan'),1.),(1.1,float('inf'))])
def test_bad_time_cannot_confirm(now,source):
    probe,p,ids=started()
    assert not probe.observe(p,ids,None,now,source)['matched']
    assert probe.ended


def test_history_expiry_is_failure_not_release():
    probe,p,ids=started()
    assert probe.observe(p,ids,None,1.31,1.2)['reason']=='diagnostic_history_expired'
    assert not probe.observe(p,ids,None,1.32,1.21)['matched']


def test_current_invalid_points_terminate_probe():
    probe,p,ids=started();p[0,0]=np.nan
    assert probe.observe(p,ids,None,1.1,1.)['reason']=='invalid_input'
    assert probe.ended


def test_motion_unavailable_does_not_substitute_zero_motion():
    probe,p,ids=started()
    with patch.object(module,'transport_points',side_effect=module.MotionUnavailable('gap')):
        assert probe.observe(p,ids,None,1.1,1.)['reason']=='motion_unavailable'


def test_multiple_co_moving_surfaces_remain_an_explicit_ambiguity():
    # 모두 움직이는 물체의 관측과 자차 이동 오차는 이 함수만으로 구별할 수 없다.
    # 이것이 운영 해제기로 채택하지 않는 이유이며 안전 통과를 뜻하지 않는다.
    probe,p,ids=started()
    with patch.object(module,'transport_points',stationary):
        probe.observe(p+[.025,.01],ids,None,1.1,1.)
        result=probe.observe(p+[.025,.01],ids,None,1.2,1.1)
    assert result['matched'] and result['static_background_assumption']
    assert result['scope']=='diagnostic_reobservation_only'
