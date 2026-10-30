"""선택형 시험, 기본 미사용: 시간 대신 현재 관측 직선에서의 차선 유지 선호를 비교한다.

ROS에서 rear_lane_keep_policy=observed_measured_v1을 명시한 시험 전용이며 운영 채택이 아니다(10/29 오프라인 v4). 후방 사각 부재를 추정하지 않는다. 현재 양의 RELEASED 출력에서
소실 쪽으로 꺾을 때 기존 평행 후보를 택하며 새 정지를 만들지 않는다. 곡선/경로/관측 맥락 상실,
벽 선택 변경, 현재 반대쪽 점유, 앞쪽 통과에는 원래 판단으로 돌아간다. 현재 관측/제동 검사를
통과한 평행 경로의 목표속도는 기존 G1 이하이며, 시험용 20 ms 엔코더 한 카운트분 여유를 둔다.
anchor_m은 이전 후보와의 진단 비교용이고 새 정책의 판단에는 현재 프레임의 현재 모서리만 쓴다.
"""
import math

from arena_vehicle_interface.drive_feedback import MotorEncoderSpec

# 현재 합성/MCU 시험의 20 ms 캡처 가정. control_dt와 같은 의미가 아니다.
ENCODER_CAPTURE_INTERVAL_S = .02
ENCODER_SPEED_QUANTUM_MPS = MotorEncoderSpec(28, 15., .0325).mean_wheel_travel_m_per_count / ENCODER_CAPTURE_INTERVAL_S

import numpy as np

import arena_autonomy.m3_execution_trial as mx
from arena_autonomy.m3_observed_follow import command_arc
from arena_autonomy.pass_release_trial import SIDES, extrema_arc

# parallel_lane_trial.proposal의 swept_limit(horizon=2.5)와 같은 범위. 새 근거값이 아니다.
SWEPT_HORIZON_M = 2.5
from arena_autonomy.traffic_path_recovery_trial import SuspendedPassageSelector


def front_extent(frame, poses, sign, cfg):
    """경로 좌표에서 차체 앞 두 모서리의 한쪽 방향 최대 횡위치. 표본 자세 사이 연속 극값 인증은 아니다."""
    p = np.asarray(poses, dtype=float).reshape(-1, 3)
    corner = np.array([[cfg.length_m/2, y] for y in (-cfg.width_m/2, cfg.width_m/2)])
    c, s = np.cos(p[:, 2])[:, None], np.sin(p[:, 2])[:, None]
    points = np.stack((p[:, :1]+c*corner[None, :, 0]-s*corner[None, :, 1],
                       p[:, 1:2]+s*corner[None, :, 0]+c*corner[None, :, 1]), axis=-1).reshape(-1, 2)
    _, lateral = frame.project(points)
    return float(np.max(sign*lateral))


class StraightLanePreferenceSelector(SuspendedPassageSelector):
    def _opposite_near(self, scene, p_detail, toward):
        """유지할 평행 경로의 반대쪽 옆면 밖 근접 관측 반환 수. 경로가 없으면 None."""
        trial = p_detail.get('parallel_hold_trial') or {}
        path, padding = trial.get('path_points'), trial.get('maximum_padding_m')
        if path is None or len(path) < 2 or padding is None or not math.isfinite(padding):
            return None
        try:
            frame = mx.ExecutionFrame(np.asarray(path, dtype=float).reshape(-1, 2))
        except (ValueError, TypeError):
            return None
        points = np.asarray(scene.occupied_endpoints_xy_m, dtype=float).reshape(-1, 2)
        span = scene.rays[-1].acquired_at_s-scene.source_first_s
        margin = float(padding)+self.follow_config.object_motion_bound_mps*max(0., span)
        result = {'count': 0, 'margin_m': margin, 'horizon_m': SWEPT_HORIZON_M}
        if not len(points):
            return result
        s, l = frame.project(points)
        half, length = self.follow_config.width_m/2, self.follow_config.length_m
        window = (s >= -length/2) & (s <= SWEPT_HORIZON_M)
        inside = s[window & (np.abs(l) <= half)]
        for name in toward:
            opposite = -dict(SIDES)[name]
            exterior = opposite*l-half
            mask = window & (exterior > 1e-9) & (exterior < margin)
            if len(inside) and np.any(mask):
                # 통로 안쪽 반환과 같은 종방향 위치의 가장자리는 통로를 가로막는 물체(추종 대상)의 일부로 본다.
                blocked = np.min(np.abs(s[mask][:, None]-inside[None, :]), axis=1) <= length
                mask[np.flatnonzero(mask)[blocked]] = False
            result['count'] += int(np.sum(mask))
        return result

    def reset(self):
        self.lane_keep = {}
        self.last_direction = {}
        self.preference_wall = None
        super().reset()

    def invalidate(self, reason):
        # 시간 연결을 잃으면 원래 선택기가 양쪽을 unknown으로 둔다. 새 해제 때 기준을 다시 잡는다.
        self.lane_keep = {}
        sides = getattr(self, 'sides', {})
        # 원래 선택기와 같이 위협을 본 적이 있으면 양쪽 모두 방향을 모르는 소실로 둔다.
        self.last_direction = ({name: 'unknown' for name in sides}
                               if any(side['seen'] for side in sides.values()) else {})
        return super().invalidate(reason)

    def _select(self, scene, baseline, parallel, speed, previous, dt, *, raw_scan=None):
        released_before = {name: side['seen'] and side['passed'] and not side['present']
                           for name, side in self.sides.items()}
        present_before = {name: side['present'] for name, side in self.sides.items()}
        output = super()._select(scene, baseline, parallel, speed, previous, dt, raw_scan=raw_scan)
        front = self.follow_config.length_m/2+self.config.pass_front_margin_m
        for name, side in self.sides.items():
            if side['present']:
                self.last_direction[name] = None
            elif present_before[name] and side['seen']:
                if side['last_present'] is None:
                    # 시간 단절(invalidate)로 사라졌다. 방향을 모른다.
                    self.last_direction[name] = 'unknown'
                    continue
                # 원래 선택기와 같은 기준으로 이번 소실 방향만 따로 기록한다(원래 vanish 기록은 바꾸지 않음).
                rear = side['last_present'].get('rearmost_ds_m')
                self.last_direction[name] = 'forward' if rear is not None and rear >= front else 'rear'
        now = scene.t_ref_s
        frame = None
        path = baseline[2].get('path_points')
        try:
            if path is not None and len(path) >= 2:
                frame = mx.ExecutionFrame(np.asarray(path, dtype=float).reshape(-1, 2))
        except (ValueError, TypeError):
            frame = None
        for name, sign in SIDES:
            side = self.sides[name]
            released = side['seen'] and side['passed'] and not side['present']
            if (not released or side['vanish'] not in ('rear', 'unknown')
                    or self.last_direction.get(name) not in ('rear', 'unknown')):
                self.lane_keep.pop(name, None)
                continue
            if name not in self.lane_keep and not released_before[name] and frame is not None:
                # 이번 호출에서 처음 해제됐다. 해제 시 차체 모서리의 경로 횡위치를 기준으로 둔다.
                self.lane_keep[name] = {'released_s': now, 'vanish': side['vanish'],
                                        'direction': self.last_direction.get(name),
                                        'anchor_m': front_extent(frame, [[0., 0., 0.]], sign, self.lat)}
        # 시간 경과가 후방 부재 증거는 아니다. 현재 같은 직선 맥락이 사라질 때만 선호를 끝낸다.
        preferred = baseline[2].get('preferred_wall')
        context_reason = (parallel[2].get('parallel_hold_trial') or {}).get('reason')
        wall_changed = (self.preference_wall is not None and preferred is not None
                        and preferred != self.preference_wall)
        self.preference_wall = preferred
        context_lost = context_reason in ('reference_not_straight', 'short_reference',
            'partial_observation_outside_trial', 'baseline_unavailable', 'invalid_motion', 'occupancy_invalid')
        if wall_changed or context_lost or frame is None:
            self.lane_keep.clear()
        info = {'active_sides': sorted(self.lane_keep), 'hold_s': None,
                'release_policy': 'current_straight_context', 'context_lost': context_lost,
                'wall_changed': wall_changed,
                'unknown_filled': False, 'hidden_rear_risk_unresolved': True, 'applied': False}
        detail = output[2]
        if (not self.lane_keep or detail.get('selector_mode') != 'RELEASED'
                or detail.get('selector_choice') != 'baseline' or not output[0] > 0):
            return output[0], output[1], {**detail, 'rear_lane_keep': info}
        if frame is None:
            info['reason'] = 'no_baseline_frame'
            return output[0], output[1], {**detail, 'rear_lane_keep': info}
        age = scene.t_ref_s-scene.source_first_s
        v = max(speed, output[0])
        distance = mx.envelope_distance(v, age, dt, mx.ExecutionConfig())
        poses = np.vstack([np.vstack((command_arc(a, max(distance, .01), self.lat),
                                      extrema_arc(a, max(distance, .01), self.lat)))
                           for a in sorted({float(output[1]), float(previous)})])
        toward = []
        for name, sign in SIDES:
            record = self.lane_keep.get(name)
            if record is None:
                continue
            current = front_extent(frame, [[0., 0., 0.]], sign, self.lat)
            predicted = front_extent(frame, poses, sign, self.lat)
            info.setdefault('sides', {})[name] = {**record, 'current_m': current, 'predicted_m': predicted}
            if predicted > current+1e-9:
                toward.append(name)
        info['toward_sides'] = toward
        if not toward:
            info['reason'] = 'not_toward_recent_rear_side'
            return output[0], output[1], {**detail, 'rear_lane_keep': info}
        p_speed, p_steer, p_detail = parallel
        info.update(baseline_clearance_m=baseline[2].get('swept_clearance_m'),
                    parallel_clearance_m=(p_detail.get('parallel_hold_trial') or {}).get('clearance_m'),
                    parallel_speed_mps=p_speed, parallel_reason=p_detail.get('reason'),
                    parallel_trial_reason=(p_detail.get('parallel_hold_trial') or {}).get('reason'))
        if not p_speed > 0:
            # 현재 위치 유지가 진행할 수 없으면 기존 판단을 쓴다(새 정지 금지).
            info['reason'] = 'parallel_not_moving_keep_original'
            return output[0], output[1], {**detail, 'rear_lane_keep': info}
        if p_speed < min(output[0], self.config.guard_speed_cap_mps)-1e-6:
            # 유지한 차선 쪽 관측 점유가 더 가깝다. 이 해제의 보류를 끝낸다(다음 해제 때 새 기준).
            for name in toward:
                self.lane_keep.pop(name, None)
            info.update(reason='parallel_slower_end_hold', ended_sides=toward)
            return output[0], output[1], {**detail, 'rear_lane_keep': info}
        near = self._opposite_near(scene, p_detail, toward)
        info['opposite_near_returns'] = near
        if near is None or near['count']:
            for name in toward:
                self.lane_keep.pop(name, None)
            info.update(reason='opposite_near_occupancy_end_hold' if near else 'parallel_path_unavailable_end_hold',
                        ended_sides=toward)
            return output[0], output[1], {**detail, 'rear_lane_keep': info}
        info.update(applied=True, reason='straight_lane_preference', replaced_speed_mps=output[0],
                    replaced_steering_rad=output[1])
        selector = {k: v for k, v in detail.items() if k.startswith('selector_') or k in (
            'pass_release_trial', 'retained_passage', 'same_path_gap', 'suspended_passage_history',
            'restored_passage_history', 'restored_history_is_current_free_space')}
        # RELEASED 현재직선만: 기존G1속도를상한으로 같은 평행경로의 점유/제동/횡변위/관측을 재계산한다.
        from arena_autonomy.measured_parallel_speed_trial import proposal as speed_proposal
        from arena_autonomy.pass_release_trial import motion_region, region_blocked
        request_cap = min(output[0], max(0., self.follow_config.max_speed_mps-ENCODER_SPEED_QUANTUM_MPS))
        faster = speed_proposal(scene, baseline, speed, previous, dt, self.follow_config, request_cap)
        speed_info = {'eligible': faster.get('eligible', False), 'applied': False,
                      'reason': faster.get('reason'), 'original_cap_mps': p_speed,
                      'baseline_cap_mps': output[0], 'request_cap_mps': request_cap,
                      'encoder_speed_quantum_mps': ENCODER_SPEED_QUANTUM_MPS,
                      'encoder_capture_interval_s': ENCODER_CAPTURE_INTERVAL_S,
                      'measured_speed_unclipped_mps': speed}
        if (not detail.get('retained_passage') and faster.get('eligible')
                and faster['target_speed_mps'] > p_speed+1e-9):
            target, steering = faster['target_speed_mps'], faster['steering_rad']
            region = faster.get('measured_region')
            if region is None:
                region = motion_region(scene, speed, target, steering, previous, dt,
                                       self.follow_config, self.lat)
            blocked = region_blocked(region, self.config)
            speed_info.update(region=region, blocked=blocked,
                              proposed_speed_mps=target, proposed_steering_rad=steering)
            if blocked is None:
                p_speed, p_steer = target, steering
                p_detail = {**p_detail, 'parallel_hold_trial': faster,
                            'path_points': faster['path_points'],
                            'swept_clearance_m': faster['clearance_m'],
                            'target_speed_before_hold_mps': target}
                speed_info.update(applied=True, reason='current_path_observed',
                                  masked_tail_risk_accepted=bool(region['masked_left'] or region['masked_right']))
        info['observed_speed'] = speed_info
        return p_speed, p_steer, {**p_detail, **selector, 'selector_choice': 'parallel',
                                  'selector_reason': 'released_straight_lane_preference', 'rear_lane_keep': info}
