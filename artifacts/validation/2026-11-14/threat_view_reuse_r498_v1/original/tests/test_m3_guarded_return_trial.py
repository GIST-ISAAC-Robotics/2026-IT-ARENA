"""출력 관문과 해제 상태의 분리. 순수 합성/상태 전이이며 운영 실행 아님."""
import pytest
from arena_autonomy.m3_guarded_return_trial import GuardedReturnTrial
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, ExecutionConfig
from test_m3_baseline_release import car_x, D0
from test_m3_observed_follow import timed_scene
from test_lateral_intent import car
from test_local_follow import history


def sequence():
    return ([(round(.4+.1*k, 3), []) for k in range(3)] +
            [(round(.7+.1*k, 3), [car(car_x(1.+.1*k), .13, D0)]) for k in range(20)])


def make(enabled=True):
    return GuardedReturnTrial(execution=ExecutionConfig(release_to_baseline=True),
                              enabled=enabled, allow_simulation_trial=True)


def test_option_requires_explicit_bool_and_default_off():
    assert not GuardedReturnTrial(allow_simulation_trial=True).enabled
    with pytest.raises(ValueError): make(1)
    with pytest.raises(ValueError): GuardedReturnTrial(enabled=True)


def test_off_entire_output_identical_to_original_controller():
    got, ref = make(False), M3ExecutionFollow(execution=ExecutionConfig(release_to_baseline=True), allow_simulation_trial=True)
    h=history(v=0.)
    from arena_autonomy.follow_trace import freeze
    for t, extra in sequence():
        scene,_=timed_scene(t,D0,extra=extra)
        assert freeze(got.command(scene,h,0.,'left'))==freeze(ref.command(scene,h,0.,'left'))


def test_unobserved_release_preserves_records_and_uses_existing_candidates():
    c=make();h=history(v=0.);rejected=[]
    for t, extra in sequence():
        scene,_=timed_scene(t,D0,extra=extra)
        out=c.command(scene,h,0.,'left')
        gate=(out[2].get('m3_lateral') or {}).get('release_gate') or {}
        if gate.get('memory_confirmed'):
            rejected.append(out)
            assert not gate['release'] and not gate['command_review']['accepted']
            assert c.inner.lateral.active and c.inner.lateral.release_records
            assert not c.audit.ever_released
            if out[0] > 0:
                assert out[2]['m3_envelope']['observation_ok']
                assert out[2]['m3_envelope']['geometry_ok']
    assert rejected and any(out[0]>0 for out in rejected)


def fixed_baseline(c, *, speed=.35, steering=0., release=True):
    def inner(scene,*args,**kwargs):
        _,path=timed_scene(scene.t_ref_s,0.)
        return speed,steering,{'reason':'local_follow','behavior':'CRUISE','path_points':path.tolist(),
            'm3_lateral':{'mode':'BASELINE','released_to_baseline':release},'runtime_authorized':False}
    c.inner.command=inner


def test_recovery_needs_two_new_scans_and_repeated_scan_cannot_count():
    c=make();h=history(v=0.);fixed_baseline(c,steering=.06)
    scene,_=timed_scene(1.)
    assert c.command(scene,h,0.,'left')[2]['reason']=='m3_return_guard_hold'
    fixed_baseline(c,release=False)
    new,_=timed_scene(1.1)
    assert c.command(new,h,0.,'left')[2]['reason']=='m3_return_guard_wait'
    assert c.command(new,h,0.,'left')[2]['reason']=='m3_return_guard_wait'
    newer,_=timed_scene(1.2)
    assert c.command(newer,h,0.,'left')[0] == .35


def test_input_reset_preserves_guard_but_session_reset_is_explicit():
    c=make();c.audit.observe_release({'m3_lateral':{'released_to_baseline':True}})
    c.reset()
    assert c.audit.ever_released and c.guard_held
    c.reset_session()
    assert not c.audit.ever_released and not c.guard_held


def test_parent_input_stop_not_replaced_or_delayed():
    c=make();c.audit.observe_release({'m3_lateral':{'released_to_baseline':True}})
    original=(0.,0.,{'reason':'follow_input_stop','behavior':'HOLD'})
    c.inner.command=lambda *a,**kw:original
    scene,_=timed_scene(1.)
    assert c.command(scene,history(v=0.),0.,'left') is original
