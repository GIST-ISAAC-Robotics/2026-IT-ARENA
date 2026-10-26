import numpy as np
import pytest
from arena_autonomy.lateral_intent import PathFrame
from projection_candidate import project_xy


@pytest.mark.parametrize('seed', [0, 1, 29, 499])
@pytest.mark.parametrize('kind', ['straight', 'curved', 'loop', 'duplicate', 'translated'])
def test_exact_projection_preserves_all_segments_ties_and_end_extensions(seed, kind):
    rng = np.random.default_rng(seed)
    x = np.linspace(0., 2., 61)
    if kind == 'straight': p = np.column_stack((x, x*.17))
    elif kind == 'curved': p = np.column_stack((x, .4*np.sin(2*x)))
    elif kind == 'loop': p = np.column_stack((np.cos(x*3), np.sin(x*3)))
    else: p = np.cumsum(rng.normal(size=(61, 2))*.1, axis=0)
    if kind == 'duplicate': p = np.repeat(p, 2, axis=0)
    if kind == 'translated': p += [1e5, -1e5]
    frame = PathFrame(p)
    points = np.vstack((p, (frame.p[1:]+frame.p[:-1])/2,
                        rng.normal(size=(501, 2))*2+frame.p.mean(axis=0)))
    for q in (points, points[:0], points[:1], points[:128], points[:129]):
        expected, actual = frame.project(q), project_xy(frame, q)
        assert all(np.array_equal(a, b, equal_nan=True) for a, b in zip(expected, actual))


def test_segment_tie_and_noncontiguous_inputs_keep_first_segment():
    p = np.array([[0., 0.], [1., 0.], [1., 1.], [0., 1.], [0., 0.]])
    frame = PathFrame(p)
    points = np.array([[.5, .5], [0., 0.], [.5, 0.], [1., 1.], [2., -2.], [-2., 2.]])
    for q in (points, points[::-1], points[::2]):
        assert all(np.array_equal(a, b) for a, b in zip(frame.project(q), project_xy(frame, q)))
