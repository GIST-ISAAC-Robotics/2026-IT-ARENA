import math

import pytest

from g2_visibility_probe import local_pose, pose_at, sequential_scene


def static_rows():
    return [dict(t_s=i*.02,rear_x_m=-.0675,rear_y_m=0.,yaw_rad=0.,
                 applied_speed_mps=0.,applied_steering_rad=0.) for i in range(21)]


def test_local_pose_inverse_and_rear_to_body():
    assert local_pose((1.,2.,math.pi/2),(1.,1.,math.pi/2)) == pytest.approx((1.,0.,0.))
    assert pose_at(static_rows(),.15)==pytest.approx((0.,0.,0.))


@pytest.mark.parametrize('height,visible',[(.12,True),(.06,False)])
def test_sequential_stamps_and_invisible_low_target(height,visible):
    scene,labels,reference=sequential_scene(static_rows(),.3,height)
    returns=sum(label=='target' and ray.status=='return' for label,ray in zip(labels,scene.rays))
    assert (returns>0)==visible
    assert len(scene.rays)==500
    assert scene.source_last_s-scene.source_first_s==pytest.approx(.0998)
    assert scene.rays[0].acquired_at_s != scene.t_ref_s
    assert any(ray.status=='masked' for ray in scene.rays)


def test_out_of_trace_is_not_clamped_to_known_pose():
    with pytest.raises(ValueError):
        pose_at(static_rows(),1.)
