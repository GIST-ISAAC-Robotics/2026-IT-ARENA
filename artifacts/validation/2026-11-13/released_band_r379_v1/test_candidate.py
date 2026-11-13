"""실제 광선의 띠/창 분류와 Python fallback·함수 복구만 제한 검증한다."""
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
sys.path[:0] = [str(A), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
import numpy as np
import pytest
import candidate
from arena_autonomy import numeric_kernels
from arena_autonomy import pass_release_trial as ps
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.local_scene import scene_from_measurements


class Frame:
    def project(self, q):
        return q[:, 0], q[:, 1]


@pytest.fixture(autouse=True)
def restore_backend():
    numeric_kernels.configure('python')
    yield
    numeric_kernels.configure('python')


@pytest.mark.parametrize('backend', ['python', 'native_v1'])
@pytest.mark.parametrize('count,direction', [(500, 1), (508, -1)])
@pytest.mark.parametrize('ranges_kind', ['return', 'unknown', 'mixed'])
def test_actual_band_and_window_full_arrays(backend, count, direction, ranges_kind):
    if backend == 'native_v1':
        # 선택 native 검증에서 모듈 부재를 skip 성공으로 감추지 않는다.
        from arena_autonomy import _numeric_kernels
        assert hasattr(_numeric_kernels, 'classes')
    numeric_kernels.configure(backend)
    angles = direction*(-np.pi+np.arange(count)*2*np.pi/count)
    ranges = np.full(count, 3.)
    if ranges_kind == 'unknown':
        ranges[:] = np.inf
    elif ranges_kind == 'mixed':
        ranges[::23] = np.inf
        ranges[::71] = np.nan
        ranges[::53] = .08
    poses = np.column_stack((np.linspace(-.03, 0, count), np.linspace(.01, 0, count), np.linspace(-.1, 0, count)))
    scene = scene_from_measurements(ranges, angles_rad=angles,
        acquired_at_s=np.linspace(.8, .9, count), t_ref_s=1., range_min=.05, range_max=12.,
        acquisition_to_ref_se2=poses, rear_half_angle_deg={'sectors_deg': [[170, 12], [-90, 5]]},
        preserve_sampling=True)
    cfg, sel = ps.lateral_config(FollowConfig()), ps.SelectorConfig()
    for sign in (-1., 1.):
        for longitudinal in (None, (-cfg.band_behind_m, cfg.band_ahead_m)):
            expected = ps.band_check(scene, Frame(), sign, cfg, sel, longitudinal)
            actual = candidate.band_check(scene, Frame(), sign, cfg, sel, longitudinal, verify=True)
            assert actual == expected


def test_empty_wall_clipped_band_never_classifies(monkeypatch):
    class OutsideFrame:
        def project(self, q):
            return q[:, 0], np.full(len(q), 999.)
    monkeypatch.setattr(ps, 'depth_and_masked_support', lambda *a: pytest.fail('empty band classified'))
    cfg, sel = ps.lateral_config(FollowConfig()), ps.SelectorConfig()
    assert candidate.band_check(None, OutsideFrame(), 1., cfg, sel) == dict(
        free=False, samples=0, observed_free=0, masked=0, unobserved=0)


def test_context_restores_function_on_exception():
    original = ps.band_check
    with pytest.raises(RuntimeError, match='deliberate'):
        with candidate.enabled():
            raise RuntimeError('deliberate')
    assert ps.band_check is original
