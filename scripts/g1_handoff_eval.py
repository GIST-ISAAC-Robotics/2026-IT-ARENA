"""센서 기반 G1/G2 앞차 비교의 실제 시작 구성과 판단 기록 확인 (평가 전용)."""
import json


def launch_arguments(mode, reserve, launch, recorded, *, progress=False, rear_sweep=False, observed_motion=False, side_departure=False, current_lane=False):
    if mode != 'g1g2':
        if reserve is not None or progress or rear_sweep or observed_motion or side_departure or current_lane:
            raise ValueError('reserve comparison is only defined for the G1/G2 trial')
        return []
    if (reserve not in ('fixed', 'moving','closing','recent','low_motion','paced') or not recorded or
            any(launch.get(k) != v for k,v in {
                'actuation_mode':'virtual_mcu', 'autonomy_mode':'local_pursuit', 'speed_bump':False,
                'lidar_rate_hz':10., 'lidar_compensation':'both', 'lidar_acquisition':'sequential',
                'speed_profile':'g1_open_1p5', 'control_rate_hz':50.}.items())):
        raise ValueError('G1/G2 pair requires an explicit reserve, recorded 1.5 m/s lap profile and sequential C1')
    if current_lane and not (side_departure and observed_motion):
        raise ValueError('current lane preference requires side departure and observed motion')
    extra = ['evaluation_handoff_yield_progress:=true'] if progress else []
    if rear_sweep:extra.append('evaluation_handoff_rear_sweep:=true')
    if observed_motion:extra.append('evaluation_handoff_observed_motion:=true')
    if side_departure:extra.append('evaluation_side_departure_trial:=true')
    if current_lane:extra.append('evaluation_current_lane_preference_trial:=true')
    return extra + [f'evaluation_handoff_reserve:={reserve}',
            'g1_boundary_evidence_mode:=corner_distance_candidate', 'g1_occupancy_timing_mode:=per_ray_candidate',
            'partial_observation_policy:=bounded_local_v4', 'g1_corridor_fallback_enabled:=true',
            'g1_short_boundary_prefix_enabled:=true', 'g1_parallel_hold_trial:=true', 'g1_pass_release_trial:=true',
            'g1_guard_curve_trial:=true', 'g1_traffic_recovery_trial:=true', 'g1_traffic_path_recovery_trial:=true']


def inspect_header(path, reserve, *, progress=False, rear_sweep=False, observed_motion=False, side_departure=False, current_lane=False):
    with path.open(encoding='utf-8') as stream:
        first = json.loads(next(stream))
    header = first['data']; f = header['follow_config']; e = header['execution_config']
    checks = dict(header=first['kind']=='header', mode=header.get('controller_mode')=='g1_g2_handoff_trial',
        normal_path=all(f.get(k) is True for k in ('corridor_fallback_enabled', 'short_boundary_prefix_enabled',
            'parallel_hold_trial','pass_release_trial','guard_curve_trial','traffic_recovery_trial','traffic_path_recovery_trial')),
        normal_speed=f.get('open_route_speed_mps')==1.5, occupancy=f.get('occupancy_timing_mode')=='per_ray_candidate',
        boundary=f.get('boundary_evidence_mode')=='corner_distance_candidate',
        observation=f.get('partial_observation_policy')=='bounded_local_v4',
        rear_sweep=f.get('released_rear_sweep_trial',False) is rear_sweep,
        static_side=f.get('static_side_release_trial',False) is observed_motion,
        side_departure=f.get('side_departure_trial',False) is side_departure,
        current_lane=f.get('current_lane_preference_trial',False) is current_lane,
        lane_headroom=(not current_lane or 0 < f.get('current_lane_headroom_mps',0.) < f.get('max_speed_mps',0.)),
        entry_motion=e.get('entry_motion_gate',False) is observed_motion,
        observed_forward_pace=e.get('observed_forward_pace',False) is (observed_motion and reserve=='paced'),
        observed_rejoin=(e.get('reacquire_policy') == 'observed_rejoin_v2') is observed_motion,
        motor_policy=f.get('signed_motion_policy')=='forward_drive', entry=e.get('follow_entry_space_enabled') is True,
        reserve=e.get('moving_follow_reserve') is (reserve in ('moving','closing','recent','low_motion','paced')),
        closing=e.get('closing_follow_reserve',False) is (reserve in ('closing','recent','low_motion','paced')),
        recent=e.get('recent_forward_reserve',False) is (reserve=='recent'),
        low_motion=e.get('low_motion_reserve',False) is (reserve in ('low_motion','paced')),
        pace_floor=e.get('moving_pace_floor',False) is (reserve=='paced'),
        policy=e.get('relative_yield_policy')==('observed_progress_yield_v3' if progress else 'bounded_relative_yield_v2'))
    return {'passed':all(checks.values()),'checks':checks,'reserve':reserve, 'execution':e}
