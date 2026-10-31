from dataclasses import replace
import numpy as np
import pytest
from candidate import OverspeedHoldGuide
from test_bypass_signed_hold_trial import inputs,active
from arena_autonomy.bypass_corridor_trial import CorridorGuide
from arena_autonomy.lidar_motion import MotionConfig,MotionHistory


def guide():
    result=OverspeedHoldGuide(stationary_outer_boundary=True,wall_reanchor=True,tracked_wall_fit=True)
    result.__dict__.update(active().__dict__)
    return result


def newer(scene,delta):
    rays=tuple(replace(r,acquired_at_s=r.acquired_at_s+delta) for r in scene.rays)
    return replace(scene,t_ref_s=scene.t_ref_s+delta,rays=rays,
        source_first_s=scene.source_first_s+delta,source_last_s=scene.source_last_s+delta)


def test_overspeed_holds_without_clipping_and_transports_actual_history():
    scene,history=inputs();g=guide();old=g.path.copy()
    result=g.evaluate(scene,history,1.021,0.,.02)
    assert result['selected'] is None and result['reason']=='entry_overspeed_motion_hold'
    assert result['measured_speed_mps']==1.021 and g.path is not None and g.reverse_wait
    # 이 fixture의 운동 이력은 음수다. 속도 인수로 이동량을 새로 만들거나 자르지 않고 실제 이력대로 이송하는지 본다.
    assert np.all(g.path[:,0]>old[:,0])
    assert g.config.max_speed_mps==1 and g.forward_confirmations==0


@pytest.mark.parametrize('speed,previous,dt',[(float('nan'),0.,.02),(True,0.,.02),(1.02,float('nan'),.02),(1.02,0.,0.),(1.02,1.,.02)])
def test_invalid_measurement_still_resets(speed,previous,dt):
    scene,history=inputs();g=guide()
    assert g.evaluate(scene,history,speed,previous,dt)['selected'] is None
    assert g.path is None


def test_epoch_missing_history_and_stale_geometry_reset():
    scene,history=inputs()
    bad=replace(scene,source_epoch=1,rays=tuple(replace(r,source_epoch=1) for r in scene.rays))
    for current,motion in ((bad,history),(scene,MotionHistory(MotionConfig()))):
        g=guide();assert g.evaluate(current,motion,1.02,0.,.02)['selected'] is None
        assert g.path is None
    g=guide();g.reference_s=.1
    assert g.evaluate(scene,history,1.02,0.,.02)['reason']=='entry_overspeed_retention_expired'
    assert g.path is None


def test_sustained_overspeed_expires_in_existing_memory_ttl():
    scene,history=inputs();g=guide()
    for delta in (0.,.1,.2):
        assert g.evaluate(newer(scene,delta),history,1.02,0.,.02)['reason']=='entry_overspeed_motion_hold'
    assert g.evaluate(newer(scene,.31),history,1.02,0.,.02)['reason']=='entry_overspeed_retention_expired'
    assert g.path is None


def test_return_to_range_needs_current_candidate_and_two_distinct_scans(monkeypatch):
    scene,history=inputs();g=guide();g.evaluate(scene,history,1.02,0.,.02)
    permitted=dict(reason='candidate',selected=dict(speed_mps=.1,steering_rad=0.),
        reference=dict(reason='current_outer_wall_rechecked_for_remembered_guide'),outer_wall_fresh_confirmations=2)
    monkeypatch.setattr(CorridorGuide,'evaluate',lambda *a:permitted)
    assert g.evaluate(scene,history,0.,0.,.02)['selected'] is None
    assert g.evaluate(newer(scene,.1),history,0.,0.,.02)['selected'] is None
    assert g.evaluate(newer(scene,.1),history,0.,0.,.02)['selected'] is None
    assert g.evaluate(newer(scene,.2),history,0.,0.,.02)['signed_hold_recovered']
    g=guide();g.evaluate(scene,history,1.02,0.,.02)
    monkeypatch.setattr(CorridorGuide,'evaluate',lambda *a:dict(reason='occupied',selected=None))
    assert g.evaluate(newer(scene,.1),history,0.,0.,.02)['selected'] is None
    assert g.forward_confirmations==0
