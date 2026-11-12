"""명시 시뮬 보호 프로세스용 r164 위험 분류. 보호 출력 경계는 유지한다."""
import importlib.util
from pathlib import Path
import math
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.partial_observation import PartialObservationPolicy
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.scan_input import scan_first_stamp
import arena_autonomy.pass_release_trial as ps

A = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('r165_geometry', A.parent/'path_side_gap_r164_v1/candidate.py')
body = importlib.util.module_from_spec(spec)
spec.loader.exec_module(body)
old_evaluate = old_control = None


def evaluate(policy, scan, points, history, now, speed, steering=0., previous_steering=0.,
             wheelbase=.135, length=.20, width=.15):
    result = old_evaluate(policy, scan, points, history, now, speed, steering,
                          previous_steering, wheelbase, length, width)
    context = getattr(policy, '_r165_context', None)
    if (context is None or not context['verified'] or result.get('allowed') is not False
            or result.get('reason') != 'unbounded_missing' or not 0 <= speed <= .15):
        return result
    try:
        scene = motion_scene(scan, history, now, timing_verified=context['verified'],
            epoch=context['epoch'], rear_half_angle_deg=context['blind'])
        cfg = FollowConfig(wheelbase_m=wheelbase, length_m=length, width_m=width)
        detail = {}
        region = ps.motion_region(scene, speed, .15, steering, previous_steering, .02,
                                  cfg, ps.lateral_config(cfg), context=detail)
        if not detail or not region['unobserved']:
            return result
        queries = detail['queries'][~detail['free'] & ~detail['masked']]
        proof = body.body_gap(scene, scan, queries, detail['padding'], length, width, speed, .15)
        memory = body.history_check(policy, scene, history, queries, points)
        report = {**result, 'independent_body_side_gap': dict(region=region, geometry=proof, history=memory)}
        if (not proof['accepted'] or not memory['accepted'] or
                ps.region_blocked(region, ps.SelectorConfig(), bounded_unknown_risk=True) is not None):
            return report
        source = scan_first_stamp(scan)
        policy._remember(points, now, source, source != policy.last_source)
        return {**report, 'allowed': True, 'reason': body.REASON, 'speed_cap_mps': .15,
                'unknown_filled': False, 'independent_current_scan': True}
    except (MotionUnavailable, ValueError, FloatingPointError) as error:
        return {**result, 'independent_body_side_gap': dict(accepted=False, error=str(error))}


def control(node):
    policy = getattr(node, 'partial_policy', None)
    if policy is None:
        return old_control(node)
    policy._r165_context = dict(verified=getattr(node.motion, 'verified', False),
        epoch=getattr(node.motion, 'counts', {}).get('resets', 0),
        blind=getattr(node, 'rear_blind_half_angle', 30.))
    try:
        return old_control(node)
    finally:
        policy._r165_context = None


def install():
    global old_evaluate, old_control
    from arena_autonomy.lidar_safety import LidarSafety
    if LidarSafety.control is control:
        return
    old_evaluate, old_control = PartialObservationPolicy.evaluate, LidarSafety.control
    PartialObservationPolicy.evaluate, LidarSafety.control = evaluate, control


def uninstall():
    from arena_autonomy.lidar_safety import LidarSafety
    PartialObservationPolicy.evaluate, LidarSafety.control = old_evaluate, old_control
