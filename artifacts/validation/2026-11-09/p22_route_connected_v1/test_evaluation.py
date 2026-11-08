import importlib.util
from pathlib import Path
import pytest
import math

spec=importlib.util.spec_from_file_location('p22_evaluation',Path(__file__).with_name('evaluate_live.py'))
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def test_real_progress_and_slow_intervals_are_independent_of_commands():
    poses=[(0.,0.,0.,0.),(.02,.01,0.,0.),(.04,.01,0.,0.),(.06,.01,0.,0.),(.08,.02,0.,0.)]
    slow,progress=m.motion_metrics(poses,.02)
    assert slow==pytest.approx(.04) and progress==pytest.approx(.01)
    assert m.motion_metrics(poses,None)[1]==0.
    with pytest.raises(ValueError):m.motion_metrics([poses[0],poses[-1]],None)


def test_missing_or_dropped_footer_never_passes():
    f=dict(accepted=9,written=9,dropped=0,failed=0,pending=0,capture_errors=0,worker_alive=False)
    assert m.trace_integrity(f,10)
    assert not m.trace_integrity(None,10)
    assert not m.trace_integrity(f|{'capture_errors':1},10)
    assert not m.trace_integrity(f,11)


def test_body_crossing_is_not_only_center_crossing():
    corners=m.outline(5.60,4.5,math.pi)
    assert max(p[0] for p in corners)>5.635
    assert m.heading_near(-math.pi,math.pi)
