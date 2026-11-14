import math

import numpy as np
import pytest
from shapely.geometry import MultiLineString, box

from audit_grid_departure_motion import audit, interval_bound
from g2_surface_geometry import footprint_at
from g2_tracking_probe import bicycle_step
from surface_contact_eval import VehicleGeometry


@pytest.mark.parametrize('steer',[-.3,0.,.3])
def test_interval_bound_contains_full_body_and_wheel_outline_displacement(steer):
    g=VehicleGeometry();before,_=footprint_at(0.,0.,0.,steer,g)
    rear=bicycle_step(-g.wheelbase_m/2,0.,0.,.35,steer,.001,g.wheelbase_m)
    after,_=footprint_at(rear[0]+g.wheelbase_m/2*math.cos(rear[2]),
                        rear[1]+g.wheelbase_m/2*math.sin(rear[2]),rear[2],steer,g)
    actual=max(float(np.linalg.norm(np.asarray(a.exterior.coords)-np.asarray(b.exterior.coords),axis=1).max())
               for a,b in zip(before.components,after.components))
    assert actual<=interval_bound(.35,steer,.001,g)+1e-15


def test_initial_three_cm_gap_included_before_first_controller_call():
    slots=[dict(index=5,x=0.,y=0.,yaw_rad=0.),dict(index=2,x=.2,y=.18,yaw_rad=0.)]
    calls=[dict(t_s=.62,truth_pose=[0.,0.,0.],safe_mps=0.,safe_steering=0.)]
    case=dict(summary=dict(rounded=False,fault=False),calls=calls)
    result=audit(case,slots,MultiLineString([[[-1,-1],[2,-1]],[[-1,1],[2,1]]]),box(-2,-2,3,2),VehicleGeometry())
    assert .0294<=result['minimum_peer_gap_lower_m']<=.03
    assert result['minimum_peer_location']['t_s']<.5
    assert result['passed']


def test_selected_ego_is_not_assumed_to_be_grid_five():
    slots=[dict(index=2,x=0.,y=0.,yaw_rad=0.),dict(index=5,x=-.2,y=.18,yaw_rad=0.)]
    case=dict(summary=dict(rounded=True,fault=False),
        calls=[dict(t_s=5.99,truth_pose=[0.,0.,0.],safe_mps=0.,safe_steering=0.)])
    walls=MultiLineString([[[-1,-1],[2,-1]],[[-1,1],[2,1]]]);road=box(-2,-2,3,2)
    result=audit(case,slots,walls,road,VehicleGeometry(),ego_index=2)
    assert .0294<=result['minimum_peer_gap_lower_m']<=.03
    assert result['minimum_peer_location']['peer_slot']==5
    with pytest.raises(ValueError,match='first pose'):
        audit(case,slots,walls,road,VehicleGeometry(),ego_index=5)
    with pytest.raises(ValueError,match='unique slots'):
        audit(case,slots+slots,walls,road,VehicleGeometry(),ego_index=2)
