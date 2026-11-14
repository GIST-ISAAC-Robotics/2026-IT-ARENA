from dataclasses import replace
from types import SimpleNamespace
import pytest
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from test_current_wall_reacquire import CFG, sensor
from test_local_follow import history


def call(reason='partial_history_warmup', enabled=True, fresh=True, base_reason='follow_release_wait'):
    cfg=replace(CFG, known_structure_policy='off', known_structure_sectors_json='', oblique_gap_trial=False,
                traffic_recovery_trial=True) if enabled else FollowConfig(partial_observation_policy='bounded_local_v4')
    owner=LocalFollow(cfg)
    scene,raw=sensor(.7,closed=True)
    owner.partial_epoch=scene.source_epoch
    owner.release_count=1
    owner.hold_reason='follow_boundary_unconfirmed'
    owner.partial_policy=SimpleNamespace(oblique_metric_enabled=False,
        requested_speed=lambda scan,yaw,speed,explained:speed,
        evaluate=lambda *args,**kwargs:dict(allowed=False,reason=reason,speed_cap_mps=0.))
    owner._path_command=lambda *args,**kwargs:(0.,.1,dict(reason=base_reason,
        boundary_from_memory=not fresh, path_source_s=scene.source_first_s,
        target_speed_before_hold_mps=.5))
    output=owner._partial_command(scene,history(),0.,'left',0.,.1,raw)
    return owner,output


def test_current_release_wait_stays_zero_but_keeps_confirmation():
    owner,out=call()
    assert out[0]==0 and out[1]==.1
    assert owner.release_count==1 and owner.hold_reason=='follow_boundary_unconfirmed'
    assert out[2]['reason']=='follow_release_wait' and out[2]['partial_release_wait_preserved']


@pytest.mark.parametrize('options',[
    dict(reason='known_obstacle_latched'),dict(enabled=False),dict(fresh=False),
    dict(base_reason='follow_gap_hold')])
def test_other_stops_keep_existing_hold_reset(options):
    owner,out=call(**options)
    assert out[0]==0 and owner.release_count==0
    assert not out[2].get('partial_release_wait_preserved',False)
