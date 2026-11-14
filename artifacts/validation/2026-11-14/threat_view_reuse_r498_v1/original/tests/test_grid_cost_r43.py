"""r43 의미 보존 최적화 대조: 원래 계산(r43_reference_oracle)과 격자·장면·위협 결과가 정확히 같은지.

허용 오차 없이 dtype·shape·값을 비교한다. 성능 수치는 이 시험의 대상이 아니다.
"""
from dataclasses import replace
import math
from types import SimpleNamespace

import numpy as np
import pytest

import arena_autonomy.lateral_intent as li
import arena_autonomy.local_scene as ls
import arena_autonomy.pass_release_trial as ps
from arena_autonomy.lateral_intent import LateralBudget, LateralIntentConfig, ObservationGrid
from arena_autonomy.local_scene import LocalScene, SensorRay

import r43_reference_oracle as ref
from test_pass_release_trial import (AHEAD, BESIDE, HIDDEN, MERGED_BEHIND, PASSED, PASSING, STRAIGHT, box_scene,
                                     config)

GRASS = LateralIntentConfig(enabled=True, surface_policy='two_wheel_grass')
STEP = GRASS.grid_resolution_m/2
BIAS = GRASS.budget.range_bias_m


def ray_scene(segments, statuses=None, t_ref=2.):
    """주어진 원점→끝점 반환 광선 장면. statuses로 masked/unknown 광선을 섞는다(끝점 없음)."""
    rays = []
    statuses = statuses or ['return']*len(segments)
    for index, ((o, h), status) in enumerate(zip(segments, statuses)):
        dx, dy = h[0]-o[0], h[1]-o[1]
        n = math.hypot(dx, dy)
        direction = (dx/n, dy/n) if n > 0 else (1., 0.)
        rays.append(SensorRay(tuple(map(float, o)), tuple(map(float, h)) if status == 'return' else None,
                              t_ref-.05+index*1e-5, status, 'laser_frame', 0, index, direction))
    if not rays:
        return LocalScene(t_ref, None, None, 'laser_frame', 0, ())
    return LocalScene(t_ref, rays[0].acquired_at_s, rays[-1].acquired_at_s, 'laser_frame', 0, tuple(rays))


def assert_same_grid(new, old):
    a, b = ref.grid_fields(new), ref.grid_fields(old)
    for key in ('state', 'returns', 'ages', 'compact', 'observed'):
        x, y = getattr(a, key), getattr(b, key)
        assert x.dtype == y.dtype and x.shape == y.shape and np.array_equal(x, y), key
    assert a.components == b.components and a.dims == b.dims


def kept_samples(grid_cfg, scene):
    """실제 후보가 만드는 표본 수와 원래 표본 수(보고용)."""
    g = ObservationGrid.__new__(ObservationGrid)
    g.r = grid_cfg.grid_resolution_m
    g.x0, g.x1 = grid_cfg.grid_x_m
    g.y0, g.y1 = grid_cfg.grid_y_m
    g.nx, g.ny = int(round((g.x1-g.x0)/g.r)), int(round((g.y1-g.y0)/g.r))
    rays = [r for r in scene.rays if r.status == 'return']
    if not rays:
        return 0, 0
    o = np.asarray([r.ray_origin_xy_m for r in rays], dtype=float)
    vec = np.asarray([r.hit_xy_m for r in rays], dtype=float)-o
    dist = np.linalg.norm(vec, axis=1)
    free = np.maximum(0., dist-grid_cfg.budget.range_bias_m)
    step = g.r/2
    counts = np.ceil(free/step).astype(int)
    first, kept = g._free_sample_window(o, vec, dist, free, counts, step)
    assert np.all(first >= 0) and np.all(kept >= 0) and np.all(first+kept <= counts)
    return int(np.sum(kept)), int(np.sum(counts))


X0, X1 = GRASS.grid_x_m
Y0, Y1 = GRASS.grid_y_m
E = 1e-12
CASES = {
    'horizontal_inside_out': [((0., .3), (5., .3))],
    'horizontal_crossing': [((-3., .3), (4., .3)), ((4., -.2), (-3., -.2))],
    'horizontal_missing': [((-3., 1.), (4., 1.)), ((-3., -1.2), (4., -1.2))],
    'tangent_y_edges': [((-3., y), (4., y)) for y in (Y0, Y1, Y0-E, Y0+E, Y1-E, Y1+E, Y0-.01, Y1+.01, Y1-.01)],
    'vertical': [((x, -3.), (x, 3.)) for x in (.5, X0, X1, X0-E, X0+E, X1-E, X1+E, X0-.01, X1+.01)],
    'diagonal': [((-3., -3.), (4., 4.)), ((4., -3.), (-3., 4.)), ((1., -.2), (3., 1.8)), ((-2., -1.8), (0., .2))],
    'corner_graze': [((X1-1., Y1-1.), (X1+1., Y1+1.)), ((X0-1., Y0+1.), (X0+1., Y0-1.)),
                     ((X1+.01-1., Y1-1.), (X1+.01+1., Y1+1.))],
    'enter_end_inside': [((-5., 0.), (.5, .1)), ((0., -4.), (.2, .3)), ((6., 3.), (1.5, .7))],
    'leave_from_inside': [((.06, 0.), (.06+8*math.cos(a), 8*math.sin(a))) for a in np.linspace(-2.6, 2.6, 27)],
    'pointing_away': [((3., 0.), (8., 0.)), ((-2., -2.), (-6., -5.)), ((0., 1.), (0., 7.))],
    'sample_boundaries': [((0., .1), (BIAS+k*STEP+e, .1)) for k in (0, 1, 2, 100, 399, 400, 401)
                          for e in (-E, 0., E)]
                         + [((X0-.5-k*STEP, .2), (X1+.3, .2)) for k in range(6)]
                         + [((X1+BIAS+k*STEP, -.1), (X0-.2, -.1)) for k in range(4)],
    'near_zero': [((.5, .5), (.5, .5)), ((.5, .5), (.51, .5)), ((.5, .5), (.5+BIAS, .5)),
                  ((.5, .5), (.5+BIAS+E, .5)), ((.5, .5), (.5+BIAS+1e-7, .5)), ((5., 5.), (5., 5.)),
                  ((X1, Y1), (X1+BIAS+STEP*.5, Y1)), ((X0-1e-3, 0.), (X0+BIAS, 0.))],
    'large_finite': [((0., 0.), (60., .1)), ((-40., -30.), (40., 30.)), ((100., 100.), (-100., -100.)),
                     ((1e4, .3), (.5, .3)), ((.3, -2e3), (.3, 2e3))],
}


@pytest.mark.parametrize('name', sorted(CASES))
def test_pruned_raster_equals_full_sample_reference(name):
    scene = ray_scene(CASES[name])
    assert_same_grid(ObservationGrid(scene, GRASS), ref.reference_grid(scene, GRASS))


def test_masked_unknown_and_empty_rays_leave_same_grid():
    segments = [((0., 0.), (1., .2)), ((0., 0.), (0., 0.)), ((0., 0.), (-1., 0.)), ((.06, 0.), (3., -.4))]
    for statuses in (['masked', 'return', 'unknown', 'return'], ['masked', 'unknown', 'masked', 'unknown']):
        scene = ray_scene(segments, statuses)
        assert_same_grid(ObservationGrid(scene, GRASS), ref.reference_grid(scene, GRASS))
    empty = ray_scene([])
    assert_same_grid(ObservationGrid(empty, GRASS), ref.reference_grid(empty, GRASS))


def test_long_rays_discard_only_samples_outside_grid():
    scene = ray_scene(CASES['large_finite']+CASES['leave_from_inside'])
    kept, total = kept_samples(GRASS, scene)
    assert 0 < kept < total
    old = ref.reference_grid(scene, GRASS)
    assert kept >= old.reference_inside_samples


def test_missing_and_zero_length_rays_keep_no_samples():
    scene = ray_scene([((5., 5.), (5., 5.)), ((-3., 1.), (4., 1.)), ((5., 5.), (5.2, 5.)), ((.5, .5), (.5, .5))])
    kept, total = kept_samples(GRASS, scene)
    assert kept == 0 < total
    assert_same_grid(ObservationGrid(scene, GRASS), ref.reference_grid(scene, GRASS))


def test_sub_nanometre_rays_without_range_bias_match_reference():
    cfg = replace(GRASS, budget=LateralBudget(range_bias_m=0.))
    scene = ray_scene([((.5, .5), (.5+1e-10, .5)), ((.5, .3), (.5, .3+5e-10)), ((5., 5.), (5.+1e-10, 5.)),
                       ((X1-1e-11, .2), (X1+1e-11, .2)), ((.5, .1), (.5, .1))])
    kept, total = kept_samples(cfg, scene)
    assert kept >= 3 and total == 4
    assert_same_grid(ObservationGrid(scene, cfg), ref.reference_grid(scene, cfg))


def test_extreme_coordinates_use_original_full_samples():
    scene = ray_scene([((2e10, 0.), (2e10+1., 0.)), ((0., 0.), (1., 0.))])
    kept, total = kept_samples(GRASS, scene)
    assert kept == total > 0
    assert_same_grid(ObservationGrid(scene, GRASS), ref.reference_grid(scene, GRASS))


def test_nonfinite_duck_scene_fails_like_original():
    ray = SimpleNamespace(status='return', ray_origin_xy_m=(0., 0.), hit_xy_m=(math.inf, 0.), acquired_at_s=1.,
                          direction_xy=(1., 0.), source_index=0)
    scene = SimpleNamespace(rays=(ray,), t_ref_s=1.)
    errors = []
    for build in (ObservationGrid, ref.reference_grid):
        with np.errstate(all='ignore'), pytest.raises(Exception) as caught:
            build(scene, GRASS)
        errors.append((type(caught.value), str(caught.value)))
    assert errors[0] == errors[1]


def random_cfg(rng):
    r = float(rng.choice([.01, .02, .013, .05, .007]))
    x0 = float(rng.uniform(-3., 0.))
    y0 = float(rng.uniform(-2., 0.))
    extent_x = max(float(rng.uniform(.5, 4.)), 3*r)
    extent_y = max(float(rng.uniform(.4, 3.)), 3*r)
    while math.ceil(extent_x/r)*math.ceil(extent_y/r) > 1000000:
        extent_x, extent_y = extent_x*.7, extent_y*.7
    budget = LateralBudget(range_bias_m=float(rng.choice([0., .005, .03, .061, float(rng.uniform(0., .1))])))
    return replace(GRASS, grid_resolution_m=r, grid_x_m=(x0, x0+extent_x), grid_y_m=(y0, y0+extent_y), budget=budget)


def random_scan_scene(rng, *, oracle):
    n = int(rng.integers(0, 400))
    values = rng.uniform(.0, 14., n)
    special = rng.random(n)
    ranges = [math.inf if p < .05 else (math.nan if p < .08 else float(v)) for p, v in zip(special, values)]
    increment = float(rng.uniform(.002, .03))
    poses = None
    if rng.random() < .7:
        dx, dy, turn = rng.normal(0., .15, n), rng.normal(0., .1, n), rng.normal(0., .3, n)
        poses = [(float(a), float(b), float(c)) for a, b, c in zip(dx, dy, turn)]
    spec = float(rng.choice([0., 30., 60., 179.]))
    if rng.random() < .4:
        sectors = [[float(rng.uniform(-180, 180)), float(rng.uniform(1., 40.))] for _ in range(int(rng.integers(0, 3)))]
        spec = {'rear_half_angle_deg': spec, 'sectors_deg': sectors if rng.random() < .5 else tuple(sectors)}
    args = dict(angle_min=float(rng.uniform(-math.pi, 0.)), angle_increment=increment, range_min=.05,
                range_max=12., acquired_at_s=[1.+i*1e-4 for i in range(n)], t_ref_s=1.+n*1e-4+.02,
                sensor_xy_m=(float(rng.uniform(-.1, .2)), float(rng.uniform(-.05, .05))),
                sensor_yaw_rad=float(rng.uniform(-.3, .3)), acquisition_to_ref_se2=poses, rear_half_angle_deg=spec)
    return (ref.scene_from_scan if oracle else ls.scene_from_scan)(ranges, **args)


@pytest.mark.parametrize('seed', range(24))
def test_randomized_scans_cfg_and_transforms_match_reference(seed):
    rng = np.random.default_rng(4300+seed)
    cfg = random_cfg(rng)
    state = rng.bit_generator.state
    scene = random_scan_scene(rng, oracle=False)
    rng.bit_generator.state = state
    assert scene == random_scan_scene(rng, oracle=True)
    assert_same_grid(ObservationGrid(scene, cfg), ref.reference_grid(scene, cfg))


@pytest.mark.parametrize('seed', range(16))
def test_randomized_segments_match_reference(seed):
    rng = np.random.default_rng(9100+seed)
    cfg = random_cfg(rng)
    n = int(rng.integers(1, 120))
    origins = rng.uniform(-6., 6., (n, 2))
    origins[: n//3] = rng.uniform(-.5, .5, (n//3, 2))
    angles = rng.uniform(-math.pi, math.pi, n)
    axis = rng.random(n) < .2
    angles[axis] = rng.choice([0., math.pi/2, math.pi, -math.pi/2], int(np.sum(axis)))
    length = rng.uniform(0., 10., n)
    snap = rng.random(n) < .3
    step = cfg.grid_resolution_m/2
    length[snap] = cfg.budget.range_bias_m+np.round(length[snap]/step)*step
    segments = [((float(o[0]), float(o[1])), (float(o[0]+d*math.cos(a)), float(o[1]+d*math.sin(a))))
                for o, a, d in zip(origins, angles, length)]
    scene = ray_scene(segments)
    assert_same_grid(ObservationGrid(scene, cfg), ref.reference_grid(scene, cfg))


# ------------------------------------------------------------------ C3: blind_spec 1회 파싱
SPECS = [30., 0, 179.5, {'rear_half_angle_deg': 30., 'sectors_deg': [[90., 10.], [-45., 5.]]},
         {'sectors_deg': ((170., 20.),)}, {}, {'rear_half_angle_deg': 12, 'sectors_deg': []}]


@pytest.mark.parametrize('spec', SPECS, ids=range(len(SPECS)))
def test_scene_from_scan_masks_match_original(spec):
    n = 500
    args = dict(angle_min=-math.pi, angle_increment=math.tau/n, range_min=.05, range_max=12.,
                acquired_at_s=[1.+i*1e-4 for i in range(n)], t_ref_s=1.2, rear_half_angle_deg=spec)
    ranges = [1.+(i % 7)*.3 for i in range(n)]
    assert ls.scene_from_scan(ranges, **args) == ref.scene_from_scan(ranges, **args)
    for angle in np.linspace(-7., 7., 301):
        assert ls.rear_masked(float(angle), spec) == ref.rear_masked(float(angle), spec)


def test_one_shot_sector_iterable_keeps_original_per_ray_parsing():
    def spec():
        return {'rear_half_angle_deg': 0., 'sectors_deg': (s for s in [[0., 10.]])}
    args = dict(angle_min=-.05, angle_increment=.01, range_min=.05, range_max=12.,
                acquired_at_s=[1.]*11, t_ref_s=1.1)
    new = ls.scene_from_scan([1.]*11, rear_half_angle_deg=spec(), **args)
    old = ref.scene_from_scan([1.]*11, rear_half_angle_deg=spec(), **args)
    assert new == old and [r.status for r in new.rays][:2] == ['masked', 'return']


def outcome(build, ranges, **args):
    try:
        return ('ok', build(ranges, **args))
    except Exception as error:  # 원래 함수와 같은 예외 종류·문구·순서인지 비교한다.
        return (type(error), str(error))


BAD = {'key': {'rear': 1.}, 'half': 180., 'width': {'sectors_deg': [[0., 95.]]}, 'text': 'abc',
       'nan': math.nan, 'pair': {'sectors_deg': [[1., 2., 3.]]}}


@pytest.mark.parametrize('bad', sorted(BAD))
@pytest.mark.parametrize('poses', [None, [(0., 0.)]*2, [(0., math.inf, 0.)]*2, [(0., 0., 0.), (0., 0.)],
                                   [(0., 0., 0.), (math.nan, 0., 0.)]], ids=range(5))
def test_invalid_spec_errors_keep_first_ray_order(bad, poses):
    args = dict(angle_min=0., angle_increment=.1, range_min=.05, range_max=12., acquired_at_s=[1., 1.],
                t_ref_s=1.1, acquisition_to_ref_se2=poses, rear_half_angle_deg=BAD[bad])
    new, old = outcome(ls.scene_from_scan, [1., 1.], **args), outcome(ref.scene_from_scan, [1., 1.], **args)
    assert new == old and new[0] != 'ok'


def test_empty_scan_does_not_validate_spec_and_angle_overflow_order_is_kept():
    for bad in BAD.values():
        args = dict(angle_min=0., angle_increment=.1, range_min=.05, range_max=12., acquired_at_s=[], t_ref_s=1.,
                    rear_half_angle_deg=bad)
        assert outcome(ls.scene_from_scan, [], **args) == outcome(ref.scene_from_scan, [], **args)
        assert outcome(ls.scene_from_scan, [], **args)[0] == 'ok'
    for spec in (30., BAD['key']):
        args = dict(angle_min=1e308, angle_increment=1e308, range_min=.05, range_max=12., acquired_at_s=[1., 1.],
                    t_ref_s=1.1, rear_half_angle_deg=spec)
        assert outcome(ls.scene_from_scan, [1., 1.], **args) == outcome(ref.scene_from_scan, [1., 1.], **args)


def test_public_rear_masked_errors_unchanged():
    for angle in (0., math.nan, math.inf, True):
        for spec in [30.]+list(BAD.values()):
            assert outcome(lambda a, **k: ls.rear_masked(a, spec), angle) == \
                outcome(lambda a, **k: ref.rear_masked(a, spec), angle)


# ------------------------------------------------------------------ C2: 같은 threat_view 호출 안 재사용
SCENES = {'none': (), 'beside': (BESIDE,), 'merged': (MERGED_BEHIND,), 'passing': (PASSING,),
          'passed': (PASSED,), 'ahead': (AHEAD,), 'hidden': (HIDDEN,), 'two': (BESIDE, AHEAD)}


def comparable(view):
    if view is None:
        return None
    return {k: v for k, v in view.items() if k != 'frame'}


@pytest.mark.parametrize('name', sorted(SCENES))
def test_threat_view_matches_original(name, monkeypatch):
    scene = box_scene(1., SCENES[name])
    cfg = ps.lateral_config(config())
    calls = []
    original = li.compact_return_mask

    def counted(*a):
        calls.append(a[0])
        return original(*a)
    old = ref.threat_view(scene, STRAIGHT, cfg)
    monkeypatch.setattr(li, 'compact_return_mask', counted)
    monkeypatch.setattr(ps.mx, 'compact_return_mask', counted)
    new = ps.threat_view(scene, STRAIGHT, cfg)
    assert comparable(new) == comparable(old)
    assert len(calls) == 1 and calls[0] is scene  # 생성되는 ObservationGrid도 같은 호출의 compact를 재사용한다


def test_same_call_reuse_copies_and_rejects_other_or_changed_inputs():
    cfg = ps.lateral_config(config())
    scene, other = box_scene(1., (BESIDE,)), box_scene(1.3, (PASSED,))
    light = ps.mx.ReturnGrid(scene, cfg)
    fresh = ref.reference_grid(scene, cfg)
    grid = ObservationGrid(scene, cfg, _same_call_returns=(scene, cfg, light))
    assert_same_grid(grid, fresh)
    assert grid.compact_returns is not light.compact_returns
    assert not np.shares_memory(grid.compact_returns, light.compact_returns)
    assert grid.return_components is not light.return_components
    assert all(a is not b for a, b in zip(grid.return_components, light.return_components))
    before = light.compact_returns.copy(), [dict(c) for c in light.return_components]
    grid.compact_returns[:] = ~grid.compact_returns
    grid.return_components[0]['points'] = -1
    assert np.array_equal(light.compact_returns, before[0]) and light.return_components == before[1]

    forged = SimpleNamespace(returns=light.returns, ages=light.ages, compact_returns=~light.compact_returns,
                             return_components=[])
    stale = ps.mx.ReturnGrid(other, cfg)
    for token in ((other, cfg, stale), (scene, replace(cfg), light), (other, cfg, forged),
                  (scene, cfg, SimpleNamespace(returns=stale.returns, ages=stale.ages,
                                               compact_returns=stale.compact_returns,
                                               return_components=stale.return_components)),
                  (scene, cfg, SimpleNamespace(returns=light.returns, ages=light.ages+1e-9,
                                               compact_returns=~light.compact_returns, return_components=[])),
                  (scene, cfg, SimpleNamespace(returns=light.returns, ages=light.ages,
                                               compact_returns=light.compact_returns.astype(int),
                                               return_components=[]))):
        assert_same_grid(ObservationGrid(scene, cfg, _same_call_returns=token), fresh)
