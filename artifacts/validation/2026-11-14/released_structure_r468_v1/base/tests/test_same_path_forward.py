import math
import numpy as np
import pytest
from arena_autonomy.same_path_gap_trial import bounded_same_path_gap
import types,zipfile
from pathlib import Path
_snapshot=Path(__file__).resolve().parents[1]/'artifacts/validation/2026-10-31/g1_forward_same_path_connection_v1/before_connection.zip'
with zipfile.ZipFile(_snapshot) as _archive:
    _source=_archive.read('src/arena_autonomy/arena_autonomy/same_path_gap_trial.py')
_legacy=types.ModuleType('preserved_side_only_gap')
exec(compile(_source,str(_snapshot),'exec'),_legacy.__dict__)
original=_legacy.bounded_same_path_gap
from arena_autonomy.scene_motion import motion_scene
from test_same_path_gap_probe import fixture
from test_local_follow import history


def forward(extra=None):
    raw,_,proof,command=fixture()
    raw.ranges[375]=8.;raw.ranges[250]=math.inf
    for i,v in (extra or {}).items():raw.ranges[i]=v
    scene=motion_scene(raw,history(),.6,timing_verified=True)
    proof.update(reason='bounded_forward_missing',total_gap_deg=.72,
        forward_neighbor_required_m=1.,minimum_gap_neighbor_m=8.,history_distance_m=.6,
        speed_cap_mps=.2)
    command['target']=.2
    return scene,raw,np.array([[.14,0.]]),np.full(500,.1),.2,.15,proof,command


def test_current_small_forward_proof_is_reused_without_filling():
    args=forward();r=bounded_same_path_gap(*args)
    assert original(*args)['reason']=='partial_not_allowed'
    assert r['accepted'],r
    assert not r['unknown_filled'] and r['residual_hidden_object_risk']
    assert r['minimum_neighbor_range_m']==1.


@pytest.mark.parametrize('update',[dict(allowed=False),dict(reason='known_obstacle_latched'),
    dict(total_gap_deg=1.51),dict(minimum_gap_neighbor_m=.99),dict(forward_neighbor_required_m=.9),
    dict(history_distance_m=1.1),dict(speed_cap_mps=.21),dict(total_gap_deg=float('nan'))])
def test_forward_proof_must_keep_its_existing_limits(update):
    args=list(forward());args[-2].update(update)
    assert not bounded_same_path_gap(*args)['accepted']


@pytest.mark.parametrize('update',[dict(now=.61),dict(steering=.21),dict(previous=.21),
    dict(target=.201),dict(speed=.36),dict(speed=-.001)])
def test_changed_motion_cannot_borrow_proof(update):
    args=list(forward());args[-1].update(update)
    assert not bounded_same_path_gap(*args)['accepted']


@pytest.mark.parametrize('extra',[{249:.08},{249:.9},{250:float('nan')},
    {250:-math.inf},{250:0.},{249:math.inf,251:math.inf},{391:math.inf}])
def test_real_occupancy_invalid_or_large_missing_stays_rejected(extra):
    assert not bounded_same_path_gap(*forward(extra))['accepted']


def test_different_raw_scan_and_outside_sweep_rejected():
    args=list(forward());args[1].header.frame_id='other'
    assert bounded_same_path_gap(*args)['reason']=='raw_scene_mismatch'
    args=list(forward());args[2]=np.array([[1.,0.]])
    assert bounded_same_path_gap(*args)['reason']=='outside_checked_sweep'


@pytest.mark.parametrize('degrees',[-90,-60,-54,-30,0,30,54,90,101])
def test_existing_side_proof_keeps_same_result(degrees):
    from test_same_path_side_scope import case
    raw,scene,proof,command,queries=case(degrees)
    args=(scene,raw,queries,np.full(500,.1),.2,.15,proof,command)
    assert bounded_same_path_gap(*args)==original(*args)


def test_rear_counterexample_is_unmasked_unknown():
    args=forward({391:math.inf});ray=args[0].rays[391]
    assert ray.status=='unknown'
    assert abs(math.atan2(ray.direction_xy[1],ray.direction_xy[0]))>math.radians(100)
    assert bounded_same_path_gap(*args)['reason']=='unknown_outside_forward_proof'
