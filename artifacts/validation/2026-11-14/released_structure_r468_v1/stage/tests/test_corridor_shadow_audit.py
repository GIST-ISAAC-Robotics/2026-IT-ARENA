"""사후 평가용 교점/외곽 판정. 정답 형상을 제어 입력으로 전달하지 않는다."""
import math
import numpy as np
import pytest
from audit_corridor_shadow import rectangle_first_hit, static_case, saved_path_audit
from shapely.geometry import box


def test_rectangle_first_entry_and_axis_parallel_miss():
    rays=np.array([[1.,0.],[-1.,0.],[0.,1.],[0.,-1.]])
    values=rectangle_first_hit((0.,0.), rays, (1.,0.), (.1,.075))
    assert values[0] == pytest.approx(.9)
    assert np.all(np.isinf(values[1:]))


def test_rectangle_exit_for_origin_inside_and_tangent():
    assert rectangle_first_hit((1.,0.), [[1.,0.]], (1.,0.), (.1,.075))[0] == pytest.approx(.1)
    assert rectangle_first_hit((0.,.075), [[1.,0.]], (1.,0.), (.1,.075))[0] == pytest.approx(.9)


@pytest.mark.parametrize('direction', [[[0.,0.]], [[2.,0.]], [[math.nan,0.]]])
def test_invalid_ray_not_silently_evaluated(direction):
    with pytest.raises(ValueError):
        rectangle_first_hit((0.,0.), direction, (1.,0.), (.1,.075))


@pytest.mark.parametrize('side', ['left','right'])
@pytest.mark.parametrize('kind', ['center_block','front_unknown','missing_inner'])
def test_clear_block_or_unknown_is_not_preliminary_motion(side, kind):
    row=static_case(.15, side, kind, 'none')
    assert not row['shadow']['preliminary_motion_supported'], row
    assert not row['driving_authorized']


def test_saved_exact_miss_stays_miss_even_when_neighbors_fit():
    path=np.column_stack((np.linspace(-.1,.8,20), np.zeros(20))).tolist()
    report={'rows':[{'seq':1,'time_s':1.,'shadow':{'geometry_supported':True,
        'preliminary_motion_supported':True,'geometry':{'path':path}}}]}
    rows=saved_path_audit(report, [[.999,0.,0.,0.],[1.001,0.,0.,0.]], box(-2,-2,2,2), [])
    assert len(rows)==1 and rows[0]['evaluated'] is False
    assert rows[0]['sensitivity_evaluated'] is True
    assert len(rows[0]['bracket_evaluations'])==2
    assert rows[0]['contemporaneous_pose_verified'] is False


def test_saved_pose_gap_is_not_interpolated():
    path=np.column_stack((np.linspace(-.1,.8,20), np.zeros(20))).tolist()
    report={'rows':[{'seq':1,'time_s':1.,'shadow':{'geometry_supported':True,
        'preliminary_motion_supported':True,'geometry':{'path':path}}}]}
    rows=saved_path_audit(report, [[.9,0.,0.,0.],[1.1,0.,0.,0.]], box(-2,-2,2,2), [])
    assert rows[0]['evaluated'] is False
    assert rows[0]['sensitivity_evaluated'] is False
