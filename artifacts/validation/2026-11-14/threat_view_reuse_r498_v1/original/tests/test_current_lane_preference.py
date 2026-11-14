"""Policy counterexamples; injected parent output is not sensor recognition proof."""
from types import SimpleNamespace
import numpy as np
import pytest
import arena_autonomy.current_lane_preference as mod
from arena_autonomy.local_follow import FollowConfig


@pytest.fixture
def setup(monkeypatch):
    cfg = FollowConfig(partial_observation_policy='bounded_local_v4',
        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate',
        parallel_hold_trial=True, pass_release_trial=True, guard_curve_trial=True,
        traffic_recovery_trial=True, traffic_path_recovery_trial=True, static_side_release_trial=True,
        corridor_fallback_enabled=True, side_departure_trial=True, current_lane_preference_trial=True,
        current_lane_headroom_mps=np.pi*.065/(28*(2.5*61/17))/.02)
    state = dict(side='right', seen=True, forward=False)
    def parent(self, scene, baseline, parallel, speed, previous, dt, **kwargs):
        evidence = {state['side']:dict(present=state['seen'], world_fixed_return=state['seen'] and not state['forward'],
            rearmost_ds_m=.3 if state['forward'] else -.2)}
        return baseline[0], baseline[1], {**baseline[2], 'selector_mode':'SOLO',
            'selector_choice':'baseline', 'selector_evidence':evidence}
    monkeypatch.setattr(mod.SuspendedPassageSelector, '_select', parent)
    monkeypatch.setattr(mod, 'faster_proposal', lambda *a, **k:dict(eligible=False,reason='test_unchanged'))
    ctl = mod.CurrentLaneSelector(cfg)
    path = np.array([[x, 0.] for x in np.linspace(0.,1.,11)])
    def call(angle=-.2, target=.8, parallel_speed=.35, reason='local_follow', context='parallel_current_position',
             wall='left', verified=True, points=()):
        scene = SimpleNamespace(t_ref_s=1., source_first_s=.9, occupied_endpoints_xy_m=points,
                                rays=[SimpleNamespace(acquired_at_s=.999)])
        base = target, angle, dict(reason=reason,path_points=path.tolist(),preferred_wall=wall)
        par = parallel_speed, 0., dict(reason='parallel_hold_follow',parallel_hold_selected=verified,
            parallel_hold_trial=dict(reason=context,path_points=path.tolist(),maximum_padding_m=.1,clearance_m=2.))
        return ctl._select(scene,base,par,.3,0.,.02)
    return ctl,state,call


@pytest.mark.parametrize('side,angle',[('right',-.2),('left',.2)])
def test_current_parallel_lane_before_return_toward_previous_side(setup,side,angle):
    ctl,state,call=setup; state['side']=side
    out=call(angle=angle)
    assert out[:2] == (.35,0.)
    assert out[2]['current_lane_preference']['applied']
    assert not out[2]['current_lane_preference']['unknown_filled']


def test_zero_center_wait_can_continue_on_verified_parallel(setup):
    ctl,state,call=setup
    assert call(target=0.,reason='follow_release_wait')[:2] == (.35,0.)


@pytest.mark.parametrize('angle',[0.,.2])
def test_no_return_toward_right_keeps_original_speed(setup,angle):
    ctl,state,call=setup
    assert call(angle=angle)[:2] == (.8,angle)


@pytest.mark.parametrize('kwargs',[
    dict(parallel_speed=0.),dict(parallel_speed=.1),dict(verified=False),
    dict(reason='follow_boundary_unconfirmed'),dict(context='reference_not_straight'),
    dict(context='partial_observation_outside_trial'),dict(points=[[.2,.16]])])
def test_unusable_parallel_or_current_obstacle_does_not_invent_new_stop(setup,kwargs):
    ctl,state,call=setup
    out=call(**kwargs)
    assert out[:2] == (.8,-.2)
    assert not out[2]['current_lane_preference']['applied']


def test_forward_pass_ends_side_preference(setup):
    ctl,state,call=setup
    call(); state['forward']=True
    assert call()[:2] == (.8,-.2)
    assert not ctl.lane_sides


def test_wall_switch_and_reset_end_context(setup):
    ctl,state,call=setup
    call(); state['seen']=False
    assert call(wall='right')[:2] == (.8,-.2)
    assert not ctl.lane_sides
    state['seen']=True; call(wall='right')
    ctl.invalidate('input_discontinuity'); state['seen']=False
    assert call(wall='right')[:2] == (.8,-.2)


def test_current_motor_gear_ratio_for_speed_quantum(setup):
    ctl,_,_=setup
    assert ctl.speed_quantum == pytest.approx(np.pi*.065/(28*(2.5*61/17))/.02)


def test_default_off_and_explicit_requirements():
    from dataclasses import replace
    from arena_autonomy.local_follow import LocalFollow
    cfg=FollowConfig(partial_observation_policy='bounded_local_v4', boundary_evidence_mode='corner_distance_candidate',
        occupancy_timing_mode='per_ray_candidate', parallel_hold_trial=True, pass_release_trial=True,
        guard_curve_trial=True, traffic_recovery_trial=True, traffic_path_recovery_trial=True,
        static_side_release_trial=True, corridor_fallback_enabled=True, side_departure_trial=True,
        current_lane_preference_trial=True, current_lane_headroom_mps=.04)
    assert not FollowConfig().current_lane_preference_trial
    assert type(LocalFollow(cfg).pass_selector) is mod.CurrentLaneSelector
    for update in (dict(side_departure_trial=False),dict(static_side_release_trial=False),
                   dict(current_lane_preference_trial=1),dict(current_lane_headroom_mps=0.),
                   dict(current_lane_headroom_mps=float('nan')),dict(current_lane_headroom_mps=True),
                   dict(rear_lane_keep_trial=True)):
        with pytest.raises(ValueError): replace(cfg,**update)
    assert type(LocalFollow(replace(cfg,current_lane_preference_trial=False)).pass_selector) is mod.SuspendedPassageSelector


def test_explicit_launch_option_and_recorded_header(tmp_path):
    import json
    import g1_handoff_eval as ev
    from pathlib import Path
    root=Path(__file__).resolve().parents[1]
    launch=json.loads((root/'config/tests/official_traffic_v1.json').read_bytes())['launch']
    args=ev.launch_arguments('g1g2','paced',launch,True,observed_motion=True,side_departure=True,current_lane=True)
    assert 'evaluation_current_lane_preference_trial:=true' in args
    for kw in (dict(side_departure=False,observed_motion=True),dict(side_departure=True,observed_motion=False)):
        with pytest.raises(ValueError): ev.launch_arguments('g1g2','paced',launch,True,current_lane=True,**kw)
    path=tmp_path/'header.jsonl'
    path.write_text(json.dumps(dict(kind='header',data=dict(follow_config={},execution_config={}))))
    result=ev.inspect_header(path,'paced',current_lane=True)
    assert not result['checks']['current_lane'] and not result['checks']['lane_headroom']
