import numpy as np
import pytest

from arena_autonomy.local_follow import path_agrees
from probe_follow_overlap import legacy_path_agrees, shared_path_agrees


def line(start=.05, finish=.65, offset=0., count=80):
    return np.column_stack((np.linspace(start, finish, count), np.full(count, offset)))


def test_truncated_but_shared_path_is_not_shape_disagreement():
    a, b = line(), line(.28, .65, .01)
    assert not legacy_path_agrees(a, b, .08)
    assert shared_path_agrees(a, b, .08)
    assert shared_path_agrees(b, a, .08)


@pytest.mark.parametrize('a,b', [(line(.05, .33), line(.30, .65)),
                               (line(), line(.5, .65)), (line(), line(.28, .65, count=4))])
def test_insufficient_common_extent_or_samples_still_rejected(a, b):
    assert not shared_path_agrees(a, b, .08)


@pytest.mark.parametrize('offset', [-.15, -.081, .081, .15])
def test_real_lateral_disagreement_is_not_relaxed(offset):
    assert not shared_path_agrees(line(), line(.28, .65, offset), .08)


def test_crossing_paths_not_just_one_intersection():
    a, b = line(), line(.28, .65)
    b[:, 1] = 2*(b[:, 0]-.45)
    assert not shared_path_agrees(a, b, .08)


def test_existing_full_shared_domain_is_same():
    for offset in (0., .03, .079, .081, .15):
        a, b = line(count=36), line(offset=offset, count=36)
        assert shared_path_agrees(a, b, .08) == legacy_path_agrees(a, b, .08)


def test_runtime_is_the_tested_common_domain_implementation():
    assert shared_path_agrees is path_agrees


def test_curved_paths_keep_lateral_difference_and_common_extent_contract():
    a, b = line(), line(.28, .65)
    for p in (a, b):
        p[:, 1] = .3*p[:, 0]**2
    assert path_agrees(a, b, .08)
    b[:, 1] += .16
    assert not path_agrees(a, b, .08)
