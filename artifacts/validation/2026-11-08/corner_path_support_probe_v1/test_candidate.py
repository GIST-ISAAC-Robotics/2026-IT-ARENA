from pathlib import Path
import json
import numpy as np
from arena_autonomy.local_follow import FollowConfig
from candidate import propose
A=Path(__file__).resolve().parent
def data():
 row=next(r for r in json.loads((A/'snapshots.json').read_bytes())['captures'] if abs(r['t']-34.62)<.001)
 return np.asarray(row['points']),np.asarray(row['carried']),row['old_wall'],FollowConfig(**row['config']),row['t']-row['old_path_source']

def test_actual_first_fit_loss_is_supported_without_removing_occupancy():
 args=data();before=args[0].copy();path,e=propose(*args)
 assert path is not None and e['accepted'] and e['geometry_intersection_is_unobserved']
 assert not e['free_space_certified'] and not e['driving_authorized']
 assert e['added_sensor_points']==0 and np.array_equal(args[0],before)
 assert e['unobserved_approach_gap_m']>.9 and len(e['occluder_indices'])>=3

def test_old_geometry_alone_does_not_refresh():
 args=list(data());args[0]=args[0][args[0][:,0]<.6]
 assert propose(*args)[0] is None

def test_expired_reference_cannot_be_renewed():
 args=list(data());args[-1]=.301
 assert propose(*args)[0] is None

def test_shifted_front_face_conflicts_with_previous_corner():
 args=list(data());p=args[0].copy();p[p[:,0]>1.,0]+=.35;args[0]=p
 assert propose(*args)[0] is None

def test_without_current_occluder_no_connection_claim():
 args=list(data());p=args[0];args[0]=p[~((p[:,1]>-.47)&(p[:,1]<-.08)&(p[:,0]<1.))]
 assert propose(*args)[0] is None

def test_changed_reference_direction_rejected():
 args=list(data());args[1]=args[1]*[1.,-1.]
 assert propose(*args)[0] is None


def test_mirrored_scene_supports_left_corner_with_same_geometry():
 args=list(data());path,e=propose(*args)
 args[0]=args[0]*[1.,-1.];args[1]=args[1]*[1.,-1.];args[2]='left'
 reflected,info=propose(*args)
 assert reflected is not None and info['accepted']
 assert np.allclose(path*[1.,-1.],reflected,atol=1e-8)


def test_straight_reference_does_not_claim_previously_seen_corner():
 args=list(data());args[1]=np.column_stack((np.linspace(-.35,2.,36),np.zeros(36)))
 assert propose(*args)[0] is None
