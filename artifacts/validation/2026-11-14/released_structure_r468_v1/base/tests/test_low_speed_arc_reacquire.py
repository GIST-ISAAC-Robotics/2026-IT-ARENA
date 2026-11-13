"""낮은 측정 속도의 양측 호길이 재획득 경계. 저장 정지 장면은 원시 기록이 있을 때만 확인한다."""
import ast
import copy
from dataclasses import asdict, replace
import importlib.util
import json
import math
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from arena_autonomy import local_follow as lf
from arena_autonomy.follow_trace import replay_call, restore_call
from arena_autonomy.lidar_motion import MotionConfig, MotionHistory
from arena_autonomy.lidar_observation import audit_front_observation
from arena_autonomy.local_path import braking_speed, mask_scan, swept_limit
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.traffic_recovery_trial import RecoveryPolicy

ROOT = Path(__file__).resolve().parents[1]
RUN = ROOT/'artifacts/validation/2026-11-01/qualifying_profile_v1/run1/follow_decisions.jsonl'
TOL, STOP_S, END_S = .08, 46.30, 47.12
SOLO = dict(signed_motion_policy='forward_drive', partial_observation_policy='bounded_local_v4', parallel_hold_trial=True,
            pass_release_trial=True, guard_curve_trial=True, traffic_recovery_trial=True, traffic_path_recovery_trial=True,
            corridor_fallback_enabled=True, short_boundary_prefix_enabled=True, boundary_evidence_mode='corner_distance_candidate',
            occupancy_timing_mode='per_ray_candidate', wheelbase_m=.135, wall_offset_m=.425, max_steering_rad=.3710583448410034,
            open_route_speed_mps=10/3.6, blend_open_speed_by_length=True, preview_turn_speed=True, lateral_acceleration_mps2=3.)
MOTION = MotionConfig(wheel_radius_m=.0325, rear_axle_x_m=-.0675, lidar_x_m=.06)
saved_scene = pytest.mark.skipif(not RUN.is_file(), reason='원시 판단 기록은 Git에서 제외된다')
PREPARED = None


def polyline(*segments, step=.02):
    """(길이, 각도) 구간을 이어 붙인 경로. 차체 원점 바로 뒤에서 시작한다."""
    points, position = [np.array([-.04, 0.])], np.array([-.04, 0.])
    for length, angle in segments:
        for _ in range(int(round(length/step))):
            position = position+step*np.array([math.cos(angle), math.sin(angle)])
            points.append(position)
    return np.array(points)


# ── 호길이 비교: 범위 요구와 분기 ────────────────────────────────────────────

def test_dense_short_paths_do_not_prove_common_range():
    # 점이 아무리 많아도 선택 호길이가 0.25 m 미만이면 비교 범위를 증명하지 못한다.
    path = polyline((.19, 0.), step=.001)
    assert len(path) > 150 and not lf.path_agrees_arc(path, path, TOL)


@pytest.mark.parametrize('length,expected', [(.33, False), (.35, True)])
def test_quarter_metre_boundary(length, expected):
    # 경로는 x=-0.04에서 시작하고 선택은 호길이 0.05부터다. 끝 호길이는 길이-0.04다.
    path = polyline((length, 0.))
    assert lf.path_agrees_arc(path, path, TOL) is expected


def test_common_range_uses_shorter_path():
    assert not lf.path_agrees_arc(polyline((1.2, 0.)), polyline((.25, 0.)), TOL)


def test_parallel_walls_keep_same_tolerance():
    base = polyline((1.2, 1.2))
    normal = np.array([-math.sin(1.2), math.cos(1.2)])
    assert lf.path_agrees_arc(base, base+.06*normal, TOL)
    assert not lf.path_agrees_arc(base, base+.15*normal, TOL)


@pytest.mark.parametrize('turn', [.5, 1.2])
@pytest.mark.parametrize('shared', [.25, .5])
def test_early_fork_is_rejected_in_any_heading(shared, turn):
    # 좌우 가지가 호길이 0.5 m 안에서 갈라지면 차체 방향과 무관하게 다른 경로다.
    assert not lf.path_agrees_arc(polyline((shared, turn), (1.2, turn+.8)), polyline((shared, turn), (1.2, turn-.8)), TOL)


def test_reversed_path_order_is_rejected():
    path = polyline((1.2, 1.2))
    assert not lf.path_agrees_arc(path, path[::-1], TOL)


def test_known_limit_late_fork_is_not_distinguished():
    """공통 80%를 넘긴 뒤 갈라지면 구별하지 못한다. 벽 정체 인증이 아니라 선호 측 재획득이다."""
    forward = polyline((.85, 0.), (1., .8)), polyline((.85, 0.), (1., -.8))
    assert lf.path_agrees(*forward, TOL) and lf.path_agrees_arc(*forward, TOL)
    sideways = polyline((.85, 1.3), (1., 2.1)), polyline((.85, 1.3), (1., .5))
    assert not lf.path_agrees(*sideways, TOL) and lf.path_agrees_arc(*sideways, TOL)


# ── 설정과 연결 ─────────────────────────────────────────────────────────

def launch_module():
    spec = importlib.util.spec_from_file_location('arc_reacquire_launch', ROOT/'src/arena_bringup/launch/simulation.launch.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_default_is_off_and_requires_solo_route_profile():
    assert lf.FollowConfig().low_speed_arc_reacquire is False
    assert lf.FollowConfig(**SOLO, low_speed_arc_reacquire=True).low_speed_arc_reacquire is True
    with pytest.raises(ValueError):
        lf.FollowConfig(low_speed_arc_reacquire=True)
    with pytest.raises(ValueError):
        lf.FollowConfig(**{**SOLO, 'low_speed_arc_reacquire': 1})


def test_registered_profile_adds_one_key_and_traffic_stays_unconnected():
    profiles = launch_module().SPEED_PROFILES
    assert profiles['g1_open_10kmh_turn15_arc'] == {**profiles['g1_open_10kmh_turn15'], 'g1_low_speed_arc_reacquire': True}
    assert [k for k, v in profiles.items() if 'g1_low_speed_arc_reacquire' in v] == ['g1_open_10kmh_turn15_arc', 'g1_open_10kmh_solo', 'g1_open_10kmh_solo_gap05', 'g1_open_10kmh_solo_gap05_arc', 'g1_open_10kmh_solo_gap05_arc_ray', 'g1_open_10kmh_solo_gap05_arc_preview', 'g1_open_10kmh_solo_gap05_arc_preview_bridge', 'g1_open_10kmh_solo_gap05_arc_preview_bridge_ray']
    for name in ('validate_g1_traffic.py', 'g1_handoff_eval.py', 'g1_traffic_eval.py'):
        text = (ROOT/'scripts'/name).read_text(encoding='utf-8')
        assert 'turn15_arc' not in text and 'low_speed_arc' not in text


def test_parameter_reaches_recorded_header_field():
    source = (ROOT/'src/arena_autonomy/arena_autonomy/local_pursuit.py').read_text(encoding='utf-8')
    assert "low_speed_arc_reacquire=bool(self.declare_parameter('g1_low_speed_arc_reacquire',False).value)" in source
    assert asdict(lf.FollowConfig(**SOLO, low_speed_arc_reacquire=True))['low_speed_arc_reacquire'] is True


def test_imu_sample_is_evaluation_record_only():
    tree = ast.parse((ROOT/'scripts/validate_basic_autonomy.py').read_text(encoding='utf-8'))
    node = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'imu_sample')
    namespace = {}
    exec(compile(ast.Module([node], []), 'imu_sample', 'exec'), namespace)
    imu_sample = namespace['imu_sample']
    vector = lambda *v: SimpleNamespace(x=v[0], y=v[1], z=v[2])
    message = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=3, nanosec=500000000)),
                              linear_acceleration=vector(.1, .2, 9.8), angular_velocity=vector(0., 0., .3))
    assert imu_sample(None) is None
    assert imu_sample(message) == {'stamp_s': 3.5, 'linear_acceleration_mps2': [.1, .2, 9.8], 'angular_velocity_rad_s': [0., 0., .3]}
    for path in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'):
        assert 'imu_sample' not in path.read_text(encoding='utf-8')


# ── 합성 정지 장면 ──────────────────────────────────────────────────────

ANGLE_MIN, COUNT, STEP = -math.pi, 500, math.tau/500
ROOM = [(-6., -6., 6., -6.), (6., -6., 6., 6.), (6., 6., -6., 6.), (-6., 6., -6., -6.)]


def ranges_for(segments, blind=()):
    origin = np.array([MOTION.lidar_x_m, MOTION.lidar_y_m])
    angles = ANGLE_MIN+np.arange(COUNT)*STEP
    out = np.full(COUNT, np.inf)
    ux, uy = np.cos(angles), np.sin(angles)
    for ax, ay, bx, by in segments:
        a, d = np.array([ax, ay], dtype=float)-origin, np.array([bx-ax, by-ay], dtype=float)
        det = ux*(-d[1])+d[0]*uy
        safe = np.where(np.abs(det) < 1e-12, np.nan, det)
        t = (a[0]*(-d[1])+d[0]*a[1])/safe
        s = (ux*a[1]-uy*a[0])/safe
        out = np.minimum(out, np.nan_to_num(np.where((t > .05) & (s >= 0) & (s <= 1), t, np.inf), nan=np.inf))
    for lo, hi in blind:
        out[(angles >= lo) & (angles <= hi)] = np.inf
    return out.tolist()


def play_scene(segments, flag, *, speed=0., blind=(), steps=30, config_changes=None):
    config = lf.FollowConfig(**SOLO, low_speed_arc_reacquire=flag)
    follow = lf.LocalFollow(replace(config, **(config_changes or {})))
    history = MotionHistory(MOTION)
    t0, previous, rows = 10., 0., []
    for k in range(300):
        history.wheels.times.append(t0-2.+k*.01), history.wheels.values.append(0.)
    for k in range(600):
        history.gyro.times.append(t0-2.+k*.005), history.gyro.values.append(0.)
    for k in range(steps):
        now = round(t0+k*.02, 6)
        first = round(math.floor((now-.102)*10)/10+.002, 6)
        stamp = SimpleNamespace(sec=int(first), nanosec=int(round((first-int(first))*1e9)))
        scan = mask_scan(SimpleNamespace(header=SimpleNamespace(stamp=stamp, frame_id='laser_frame'),
                                         ranges=ranges_for(segments, blind), angle_min=ANGLE_MIN, angle_increment=STEP,
                                         range_min=.05, range_max=12., time_increment=.0002), 30.)
        scene = motion_scene(scan, history, now, timing_verified=True, epoch=0, rear_half_angle_deg=30.)
        v, s, detail = follow.command(scene, history, speed, 'right', previous, .02, raw_scan=scan)
        previous = max(previous-.06, min(previous+.06, s))
        rows.append(SimpleNamespace(v=v, s=s, reason=detail.get('reason'), bilateral=detail.get('bilateral_reference')))
    return rows


def corridor(extra=(), half=.425, length=3.):
    return [(-1., -half, length, -half), (-1., half, length, half), (-1., -half, -1., half), *extra, *ROOM]


def fork(start_x, half=.425, spread=math.radians(45), arm=2.5):
    dx, dy = arm*math.cos(spread), arm*math.sin(spread)
    nose = start_x+half/math.sin(spread)+.45
    return [(-1., -half, start_x, -half), (start_x, -half, start_x+dx, -half-dy), (-1., half, start_x, half),
            (start_x, half, start_x+dx, half+dy), (nose, 0., nose+dx, -dy), (nose, 0., nose+dx, dy),
            (-1., -half, -1., half), *ROOM]


SIDE_GAP = [(-1., -.425, 3., -.425), (-1., .425, .2, .425), (1.3, .425, 3., .425), (-1., -.425, -1., .425), *ROOM]
AMBIGUOUS = {'early_fork_0p3': fork(.3), 'early_fork_0p6': fork(.6), 'side_gap': SIDE_GAP,
             'observed_block': corridor(((.38, -.425, .38, .425),))}


@pytest.mark.parametrize('name', sorted(AMBIGUOUS))
def test_new_branch_adds_no_motion_in_ambiguous_scene(name):
    off, on = play_scene(AMBIGUOUS[name], False), play_scene(AMBIGUOUS[name], True)
    assert [r.v for r in on] == [r.v for r in off]
    assert not any(r.bilateral == 'low_speed_arc' for r in on)


def test_front_no_return_stays_unconfirmed():
    off = play_scene(corridor(), False, blind=((-.3, .3),))
    on = play_scene(corridor(), True, blind=((-.3, .3),))
    assert [r.v for r in on] == [r.v for r in off] and all(r.v == 0 for r in on)


# ── 저장 정지 장면: 실제 정지 직전까지 재생해 얻은 상태에서 이어 받는다 ──────

def guard(policy, data, speed, steering, previous, motion, config):
    """lidar_safety.control의 양수 속도 결정 순서를 노드 없이 그대로 계산한다."""
    data, scan, history = restore_call(data, motion)
    now = data['reference_s']
    masked = mask_scan(scan, data['rear_half_angle_deg'])
    observation = audit_front_observation(masked.ranges, masked.angle_min, masked.angle_increment,
                                          masked.range_min, masked.range_max, history.config.lidar_yaw_rad)
    if observation.reason not in ('ok', 'front_unobserved'):
        return 0., observation.reason, 0.
    first = masked.header.stamp.sec+masked.header.stamp.nanosec*1e-9
    points, _, _ = history.project(masked.ranges, masked.angle_min, masked.angle_increment, masked.range_min,
                                   masked.range_max, first, masked.time_increment, now, mode='both', timing_verified=True)
    measured = float(history.wheels.available(now)[1][-1])
    partial = policy.evaluate(masked, points, history, now, max(speed, abs(measured)), steering, previous,
                              config.wheelbase_m, config.length_m, config.width_m)
    if not partial['allowed']:
        return 0., partial['reason'], 0.
    if partial['speed_cap_mps'] is not None:
        speed = min(speed, partial['speed_cap_mps'])
    clearance = swept_limit(points, steering, config.wheelbase_m, length=config.length_m, width=config.width_m)
    cap = braking_speed(clearance, now-first)
    return float(min(speed, cap)), 'clear' if cap >= speed else 'lidar_braking', float(clearance)


def prepared():
    global PREPARED
    if PREPARED is not None:
        return PREPARED
    follow = policy = motion = None
    calls, mismatch = [], 0
    with RUN.open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            kind, data = row['kind'], row['data']
            if kind == 'header':
                motion = MotionConfig(**data['motion_config'])
                follow = lf.LocalFollow(replace(lf.FollowConfig(**data['follow_config']), low_speed_arc_reacquire=True))
                policy = RecoveryPolicy(allow_narrow_forward=True, surface_translation_enabled=True, lateral_metric_enabled=True)
            elif kind == 'reset':
                follow.reset()
            elif kind == 'call':
                now = data['input']['reference_s']
                if now >= END_S:
                    break
                if now >= STOP_S:
                    calls.append(data['input'])
                    continue
                v, s, _ = replay_call(follow, data['input'], motion)
                mismatch += abs(v-data['output'][0]) > 1e-9 or abs(s-data['output'][1]) > 1e-9
                if now >= STOP_S-3.:
                    guard(policy, data['input'], data['output'][0], data['output'][1],
                          data['input']['previous_steering_rad'], motion, follow.config)
    PREPARED = SimpleNamespace(motion=motion, follow=follow, policy=policy, calls=calls, mismatch=mismatch)
    return PREPARED


def play(flag, speed=None, inject=None, **changes):
    state = prepared()
    follow, previous, rows = copy.deepcopy(state.follow), 0., []
    follow.config = replace(follow.config, low_speed_arc_reacquire=flag, **changes)
    for call in state.calls:
        data = copy.deepcopy(call)
        data['previous_steering_rad'] = previous
        if speed is not None:
            data['speed_mps'] = speed
        if inject is not None:
            inject(data)
        v, s, detail = replay_call(follow, data, state.motion)
        previous = max(previous-.06, min(previous+.06, s))
        rows.append(SimpleNamespace(v=v, s=s, published=previous, detail=detail, reason=detail.get('reason'),
                                    bilateral=detail.get('bilateral_reference'), fresh=detail.get('fresh_boundary_accepted')))
    return rows


def block_left_arc(data, distance=.34):
    """저장 스캔의 왼쪽 앞 조향 호에 실제 반환을 넣는다. 무반환을 자유 공간으로 바꾸지 않는다."""
    scan = data['scan']
    for i in range(len(scan['ranges'])):
        angle = scan['angle_min']+i*scan['angle_increment']
        if 0 <= angle <= math.radians(70):
            value = scan['ranges'][i]
            scan['ranges'][i] = min(distance, value) if isinstance(value, float) else distance


@saved_scene
def test_prefix_replay_matches_recorded_decisions():
    assert prepared().mismatch == 0


@saved_scene
def test_option_off_keeps_recorded_stop():
    rows = play(False)
    assert all(r.v == 0 and r.s == 0 for r in rows)
    assert {r.reason for r in rows} == {'follow_boundary_unconfirmed'}


@saved_scene
def test_option_on_restarts_only_after_existing_release():
    rows = play(True)
    first = next(i for i, r in enumerate(rows) if r.v > 0)
    assert rows[first].reason == 'local_follow' and rows[first].published > .3
    assert sum(1 for r in rows[:first] if r.fresh) >= 2
    assert any(r.bilateral == 'low_speed_arc' for r in rows)
    held = [r for r in rows if r.v == 0 and (r.detail.get('swept_clearance_m') or 1.) <= .2]
    assert held and {r.reason for r in held} == {'follow_steer_in_place'}


@saved_scene
@pytest.mark.parametrize('speed,allowed', [(-.5, False), (-.11, False), (-.10, True), (-.08, True), (0., True),
                                           (.08, True), (.10, True), (.11, False), (.5, False)])
def test_condition_is_measured_magnitude(speed, allowed):
    rows = play(True, speed=speed)
    assert any(r.bilateral == 'low_speed_arc' for r in rows) is allowed
    assert any(r.v > 0 for r in rows) is allowed


@saved_scene
def test_legacy_hold_negative_feedback_still_holds():
    rows = play(True, speed=-.08, signed_motion_policy='legacy_hold')
    assert all(r.v == 0 for r in rows) and {r.reason for r in rows} == {'follow_reverse_motion'}


@saved_scene
def test_low_measured_speed_does_not_skip_two_fresh_scans():
    state = prepared()
    follow, previous, sources = copy.deepcopy(state.follow), 0., set()
    for call in state.calls:
        data = copy.deepcopy(call)
        data['previous_steering_rad'], data['speed_mps'] = previous, .05
        v, s, detail = replay_call(follow, data, state.motion)
        previous = max(previous-.06, min(previous+.06, s))
        sources.add(detail.get('source_first_s'))
        if v > 0:
            break
    assert v > 0 and len(sources) >= 3


@saved_scene
def test_observed_obstacle_on_steered_arc_keeps_zero():
    assert all(r.v == 0 for r in play(True, inject=block_left_arc))


@saved_scene
def test_independent_guard_recomputes_forward_permission():
    state = prepared()

    def series(distance):
        policy, values = copy.deepcopy(state.policy), []
        for call in state.calls[:30]:
            data = copy.deepcopy(call)
            if distance is not None:
                block_left_arc(data, distance)
            values.append(guard(policy, data, .5, .371, .371, state.motion, state.follow.config))
        return values
    clear, near, close = series(None), series(.34), series(.16)
    assert all(v[0] == .5 and v[1] == 'clear' and v[2] > .9 for v in clear)
    assert max(v[2] for v in near) < .5
    assert max(v[2] for v in close) < .2 and max(v[0] for v in close) < .35
