import importlib.util
from pathlib import Path


spec = importlib.util.spec_from_file_location('registered_reapproach_eval', Path(__file__).with_name('evaluate.py'))
ev = importlib.util.module_from_spec(spec)
spec.loader.exec_module(ev)


def sample(t, hits, blind=False, x=-.3):
    return dict(stamp=t, hits=hits, whole_rear_mask=blind, right=True, relative_x_m=x)


def test_rear_center_is_not_enough_if_near_corner_remains_visible():
    assert ev.wholly_in_rear_mask([(-1., -.23, 0.)]*5)
    assert not ev.wholly_in_rear_mask([(-.2, -.1, 0.)]*5)
    assert not ev.wholly_in_rear_mask([(-1., -.23, 0.), (-.2, -.1, 0.)])


def test_geometry_only_or_wrong_order_cannot_be_reacquisition():
    initial = [sample(0., 5), sample(.1, 4)]
    blind = sample(.3, 0, True, -1.)
    returned = [sample(.5, 4, x=-.5), sample(.6, 5, x=-.4)]
    assert ev.phases(initial+[blind]+returned)['established']
    assert not ev.phases(initial+returned+[blind])['established']
    assert not ev.phases(initial+[blind]+[sample(.5, 0), sample(.6, 0)])['established']
    assert not ev.phases(initial+[blind]+[sample(.5, 4, x=-1.), sample(.6, 4, x=-1.)])['established']
