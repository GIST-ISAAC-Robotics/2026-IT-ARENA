"""기본 OFF 선택형: 정지 측방 반환 뒤 현재 관측 직선에서 불필요한 중앙 복귀를 줄인다.

기억은 어느 쪽에 반환이 있었는지만 담으며 과거 점/자유 공간을 추가하지 않는다.
평행 경로의 현재 점유/관측/운동 검사는 유지한다. 현재 경로가 진행할 수 없거나
코너/벽 선택이 바뀌면 원래 판단으로 돌아가며 새 정지 조건을 만들지 않는다.
후방 차량의 부재나 향후 횡이동을 보장하는 기능은 아니다.
"""
import numpy as np

from arena_autonomy.traffic_path_recovery_trial import SuspendedPassageSelector
from arena_autonomy.observed_lane_preference_trial import front_extent, StraightLanePreferenceSelector
from arena_autonomy.pass_release_trial import SIDES, NORMAL_BASELINE, extrema_arc, motion_region, region_blocked
from arena_autonomy.m3_observed_follow import command_arc
import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.fresh_parallel_speed_trial import proposal as faster_proposal


class CurrentLaneSelector(SuspendedPassageSelector):
    def __init__(self, follow_config, config=None):
        self.speed_quantum = follow_config.current_lane_headroom_mps
        super().__init__(follow_config, config)

    def reset(self):
        self.lane_sides = set()
        self.lane_wall = None
        super().reset()

    def invalidate(self, reason):
        self.lane_sides = set()
        self.lane_wall = None
        return super().invalidate(reason)

    def _select(self, scene, baseline, parallel, speed, previous, dt, *, raw_scan=None):
        out = super()._select(scene, baseline, parallel, speed, previous, dt, raw_scan=raw_scan)
        detail = out[2]
        evidence = detail.get('selector_evidence') or {}
        for name, _ in SIDES:
            e = evidence.get(name) or {}
            if e.get('world_fixed_return') and e.get('present'):
                self.lane_sides.add(name)
            elif (e.get('present') and e.get('rearmost_ds_m') is not None
                  and e['rearmost_ds_m'] >= self.follow_config.length_m/2+self.config.pass_front_margin_m):
                self.lane_sides.discard(name)
        p_detail = parallel[2]
        trial = p_detail.get('parallel_hold_trial') or {}
        wall = baseline[2].get('preferred_wall')
        changed_wall = self.lane_wall is not None and wall is not None and wall != self.lane_wall
        self.lane_wall = wall
        lost = (trial.get('reason') in ('reference_not_straight','short_reference',
                'partial_observation_outside_trial','baseline_unavailable','invalid_motion','occupancy_invalid'))
        if changed_wall or lost:
            self.lane_sides.clear()
        info = dict(sides=sorted(self.lane_sides), applied=False, unknown_filled=False,
                    context_lost=lost, changed_wall=changed_wall, hidden_rear_risk_unresolved=True)

        def original(reason):
            info['reason'] = reason
            return out[0], out[1], {**detail, 'current_lane_preference': info}

        if (not self.lane_sides or detail.get('selector_mode') not in ('SOLO','RELEASED')
                or baseline[2].get('reason') not in NORMAL_BASELINE
                or detail.get('selector_choice') != 'baseline'):
            return original('not_current_straight_baseline')
        # Existing verified positive parallel output retains fresh confirmation,
        # current occupancy, side departure geometry and measured motion checks.
        p_speed, p_steer = parallel[:2]
        if not p_speed > 0 or p_detail.get('parallel_hold_selected') is not True:
            return original('parallel_not_moving')
        if p_speed < min(out[0], self.config.guard_speed_cap_mps)-1e-6:
            return original('parallel_slower_than_existing_guard_cap')
        try:
            frame = mx.ExecutionFrame(np.asarray(baseline[2]['path_points'], dtype=float).reshape(-1,2))
        except (KeyError, TypeError, ValueError):
            return original('no_current_frame')
        age = scene.t_ref_s-scene.source_first_s
        distance = max(.01, mx.envelope_distance(max(speed,out[0]), age, dt, mx.ExecutionConfig()))
        poses = np.vstack([np.vstack((command_arc(a, distance, self.lat), extrema_arc(a,distance,self.lat)))
                           for a in sorted({float(out[1]), float(previous)})])
        toward = [name for name,sign in SIDES if name in self.lane_sides
                  and front_extent(frame, poses, sign, self.lat) > front_extent(frame, [[0.,0.,0.]], sign, self.lat)+1e-9]
        info['toward_sides'] = toward
        if out[0] > 0 and not toward:
            return original('not_toward_previous_side')
        # Do not trade a currently observed opposite obstacle for hidden history.
        near = StraightLanePreferenceSelector._opposite_near(self, scene, p_detail, sorted(self.lane_sides))
        info['opposite_near'] = near
        if near is None or near['count']:
            self.lane_sides.clear()
            return original('opposite_current_occupancy')
        if out[0] > p_speed:
            request = min(out[0], max(0., self.follow_config.max_speed_mps-self.speed_quantum))
            faster = faster_proposal(scene, baseline, speed, previous, dt, self.follow_config, request)
            info['faster'] = {k:faster.get(k) for k in ('eligible','reason','target_speed_mps','clearance_m')}
            if faster.get('eligible') and faster['target_speed_mps'] > p_speed:
                region = motion_region(scene, speed, faster['target_speed_mps'], faster['steering_rad'],
                                       previous, dt, self.follow_config, self.lat)
                blocked = region_blocked(region, self.config)
                info['faster']['blocked'] = blocked
                if blocked is None:
                    p_speed, p_steer = faster['target_speed_mps'], faster['steering_rad']
                    p_detail = {**p_detail, 'parallel_hold_trial': faster,
                                'path_points': faster['path_points'], 'swept_clearance_m': faster['clearance_m'],
                                'target_speed_before_hold_mps': p_speed}
        selector = {k:v for k,v in detail.items() if k.startswith('selector_') or k in (
            'pass_release_trial','retained_passage','same_path_gap','suspended_passage_history',
            'restored_passage_history','restored_history_is_current_free_space')}
        info.update(applied=True, reason='prefer_current_verified_lane', previous_output=out[:2],
                    speed_quantum_mps=self.speed_quantum)
        return p_speed, p_steer, {**p_detail, **selector, 'selector_choice':'parallel',
            'selector_reason':'static_side_current_lane', 'current_lane_preference':info}
