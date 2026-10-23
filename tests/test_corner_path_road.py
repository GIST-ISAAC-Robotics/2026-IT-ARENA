import math
import numpy as np
import pytest
from shapely.geometry import box

from audit_corner_path_road import exact_pose,path_footprints,bracketing_poses,summary_metrics


def test_only_exact_unique_pose_not_interpolation_or_nearest():
    rows=[[1.,2.,3.,0.],[1.002,2.,3.,0.]]
    np.testing.assert_array_equal(exact_pose(rows,1.),[2.,3.,0.])
    with pytest.raises(ValueError):exact_pose(rows,1.001)
    with pytest.raises(ValueError):exact_pose(rows+[rows[0]],1.)


def test_bracket_preserves_both_raw_poses_and_does_not_interpolate_yaw():
    rows=[[1.,2.,3.,math.pi-.01],[1.002,2.001,3.001,-math.pi+.01]]
    assert bracketing_poses(rows,1.001)==rows
    with pytest.raises(ValueError):bracketing_poses(rows,.999)
    with pytest.raises(ValueError):bracketing_poses(rows+[rows[1]],1.001)
    with pytest.raises(ValueError):bracketing_poses([rows[0],[1.01,2.,3.,0.]],1.001)


def test_no_evaluated_rows_cannot_be_reported_as_zero_violations():
    report=summary_metrics([dict(evaluated=False)])
    assert report['evaluated']==0 and not report['metric_available']
    assert report['outside_samples'] is None and report['wall_overlap_samples'] is None
    assert report['sensitivity_outside_samples'] is None


def test_straight_path_and_footprint_not_just_center():
    p=np.column_stack((np.linspace(0.,1.,20),np.zeros(20)))
    road=box(-.2,-.225,1.2,.225)
    result=path_footprints(p,[0.,0.,0.],road)
    assert result['outside_over_1cm2']==0
    crossed=path_footprints(p+[0,.20],[0.,0.,0.],road)
    assert crossed['outside_over_1cm2']>0
    assert not result['continuous_sweep_verified'] and not result['tracking_dynamics_verified']
    assert not result['driving_authorized']


def test_rotation_translation_and_wall_overlap():
    p=np.column_stack((np.linspace(0.,1.,20),np.zeros(20)))
    road=box(1.775,2.8,2.225,4.2)
    result=path_footprints(p,[2.,3.,math.pi/2],road,[box(1.9,3.4,2.1,3.5)])
    assert result['outside_over_1cm2']==0
    assert result['wall_overlap_over_1mm2']>0
    assert result['samples'][-1]['world_y_m']==pytest.approx(4.)


@pytest.mark.parametrize('p,pose',[(np.zeros((3,2)),[0.,0.,0.]),([],[]),
    ([[0,0],[1,0],[math.nan,0]],[0,0,0])])
def test_bad_path_is_not_fitted_or_filled(p,pose):
    with pytest.raises(ValueError):path_footprints(p,pose,box(-1,-1,1,1))
