import math
from probe_shape import open_parts,common,render_open
from g0_geometry import TruthBox,ray_box_distance,Footprint


def test_all_six_parts_and_projected_full_body():
    t=TruthBox(1.2,-.15,.14)
    p=open_parts(t)
    assert len(p)==6 and sum(x.label=='target_post' for x in p)==4
    assert p[0].footprint==p[1].footprint==Footprint()
    assert p[0].bottom_m==0 and p[1].bottom_m==.12
    for part in p:
        assert abs(part.x_m-t.x_m)+part.footprint.length_m/2<=.1+1e-12
        assert abs(part.y_m-t.y_m)+part.footprint.width_m/2<=.075+1e-12


def test_scan_plane_passes_opening_and_hits_post():
    p=open_parts(TruthBox(1.2,0.,.14))
    def hit(y,z):return [d for box in p if (d:=ray_box_distance((0,y,z),(1,0,0),box,0,12)) is not None]
    assert hit(0,.095)==[]
    assert math.isclose(min(hit(.055,.095)),1.11)
    assert math.isclose(min(hit(0,.01)),1.1)
    assert math.isclose(min(hit(0,.13)),1.1)


def test_sensor_shape_does_not_leak_into_physical_evaluation():
    old=common.probe.evaluation_boxes
    rows=[dict(t=0.,rear_pose=[-.0675,0.,0.],speed=0.,steering=0.),dict(t=.1,rear_pose=[-.0675,0.,0.],speed=0.,steering=0.)]
    before=common.probe.evaluation_metrics((-.0675,0.,0.),0.,.1,-.15,False)
    scan=render_open(rows,.1,-.15,False)
    assert len(scan.ranges)==500 and common.probe.evaluation_boxes is old
    assert common.probe.evaluation_metrics((-.0675,0.,0.),0.,.1,-.15,False)==before
