"""저속 G1/G2 전환 비교. 운영 코드/ROS 연결이 아닌 명시 시험용 후보."""
import math
import numpy as np

from arena_autonomy.bypass_entry_follow import BypassEntryFollow
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points


def normal_config():
    return FollowConfig(partial_observation_policy='bounded_local_v4', parallel_hold_trial=True,
        pass_release_trial=True, guard_curve_trial=True, traffic_recovery_trial=True,
        traffic_path_recovery_trial=True, corridor_fallback_enabled=True,
        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate')


def front_patch(scene):
    """좁은 전방의 연속 반환 조각. 물체 ID/전체 외곽/정지의 증명이 아니다."""
    groups, current, previous = [], [], None
    for ray in scene.rays:
        p = ray.hit_xy_m
        eligible = p is not None and .20 <= p[0] <= 1.60 and abs(p[1]) <= .24
        if not eligible or (previous is not None and ray.source_index != previous + 1):
            if current:
                groups.append(np.asarray(current)); current = []
        if eligible:
            current.append(p)
        previous = ray.source_index
    if current:
        groups.append(np.asarray(current))
    faces = []
    for group in groups:
        if len(group) < 3 or not np.any(abs(group[:, 1]) < .15):
            continue
        front = group[group[:, 0] <= np.quantile(group[:, 0], .1) + .08]
        if len(front) >= 3:
            faces.append(front)
    faces.sort(key=lambda p: float(np.median(p[:, 0])))
    if len(faces) > 1 and np.median(faces[1][:, 0]) - np.median(faces[0][:, 0]) < .20:
        return None, 'multiple_near_front_patches'
    return (faces[0], 'current_front_patch') if faces else (None, 'no_front_patch')


class BlockingPersistence:
    """0.8초 동안 낮은 이동을 보인 반환 조각. 차량 정지/신원 판정기는 아니다.

    60 mm는 ±30 mm 거리 비교를 위한 시험값이다. 느린 물체·가림 교체·모터축
    슬립은 정지처럼 보일 수 있으므로 우회 허가/미관측 해제에는 사용하지 않는다.
    """
    def __init__(self):
        self.reset()

    def reset(self):
        self.rows = []; self.key = None; self.last_ref = None
        self.result = dict(ready=False, observed=False, fresh=False, reason='waiting')

    def update(self, scene, history):
        now, source = scene.t_ref_s, scene.source_first_s
        if not source_valid(scene) or source is None or not 0 <= now-source < .30:
            self.reset()
            return dict(self.result, reason='invalid_or_stale_source')
        key = (scene.source_frame_id, scene.source_epoch, source)
        if self.key is not None and (key[:2] != self.key[:2] or source < self.key[2]
                or now < self.last_ref or source-self.key[2] > .25):
            self.reset()
        self.last_ref = now
        if key == self.key:
            return dict(self.result, fresh=False)
        self.key = key
        patch, reason = front_patch(scene)
        if patch is None:
            self.rows = []
            self.result = dict(ready=False, observed=False, fresh=True, reason=reason)
            return self.result
        deviations = []
        for old in self.rows:
            try:
                projected = transport_points(old['patch'], history, old['time'], now)
            except ValueError:
                self.rows = []; deviations = []; break
            deviations.append(float(np.linalg.norm(np.median(patch, axis=0)
                                                   - np.median(projected, axis=0))))
        if deviations and max(deviations) > .06:
            self.rows = []
        self.rows.append(dict(source=source, time=now, patch=patch.copy()))
        # 판정 창을 처음부터 다시 누적하지 않고 최소 0.8초의 연속 관측을 유지한다.
        while len(self.rows) > 2 and source-self.rows[1]['source'] >= .8:
            self.rows.pop(0)
        span = source-self.rows[0]['source']
        self.result = dict(ready=len(self.rows) >= 6 and span >= .8-1e-9,
            observed=True, fresh=True, reason='low_motion_patch_candidate',
            sample_count=len(self.rows), source_span_s=span,
            max_compensated_displacement_m=max(deviations, default=0.),
            front_median_xy=np.median(patch, axis=0).tolist(),
            identity_or_stationarity_proven=False)
        return self.result


class HandoffPrototype:
    def __init__(self):
        self.normal = LocalFollow(normal_config())
        self.bypass = BypassEntryFollow(allow_simulation_trial=True)
        self.monitor = BlockingPersistence()
        self.mode = 'FOLLOW'; self.last_key = None
        self.reacquire = 0

    def reset(self):
        self.normal.reset(); self.bypass.reset(); self.monitor.reset()
        self.mode = 'FOLLOW'; self.last_key = None; self.reacquire = 0

    def command(self, scene, history, speed, side, previous, dt, *, raw_scan):
        key = (scene.source_epoch, scene.source_first_s)
        if self.last_key is not None and (key[0] != self.last_key[0]
                or key[1] is None or key[1] < self.last_key[1]):
            self.reset()
            return 0., previous, dict(mode=self.mode, reason='handoff_source_reset')
        # G1의 옆차/소실 이력은 G2 중에도 관측한다. 모드 전환만으로 지우지 않는다.
        normal = self.normal.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        persistence = self.monitor.update(scene, history)
        fresh = key != self.last_key
        self.last_key = key
        detail = dict(mode_before=self.mode, normal=normal[2], persistence=persistence,
                      scope='explicit_simulation_handoff_prototype')
        fault = normal[2].get('selector_choice') == 'hold_fault'
        if fault or not source_valid(scene) or not math.isfinite(speed) or speed < 0:
            self.bypass.reset(); self.mode = 'FOLLOW'; self.reacquire = 0
            return 0., previous, dict(detail, mode=self.mode, reason='handoff_input_fault')
        if self.mode == 'FOLLOW':
            # 확실한 현재 전방 반환을 보면서 관측 시간을 확보한다. 반환이 없어지면
            # G1을 따르며, 무반환 자체를 장애물/우회 신호로 사용하지 않는다.
            target = min(normal[0], .35) if persistence['observed'] else normal[0]
            # .35는 출력 상한이다. 28카운트/15:1의 20 ms 속도 양자화로 측정값은
            # .340/.365를 오가므로 이를 입력 허용선으로 쓰면 관측을 매번 버린다.
            # 측정 속도는 그대로 G2에 전달하여 기존 반응/제동 영역에서 검사한다.
            if (persistence['ready'] and speed <= self.bypass.config.max_speed_mps
                    and abs(previous) <= .08):
                candidate = self.bypass.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
                chosen = candidate[2].get('selected')
                detail['bypass'] = candidate[2]
                if chosen is not None and chosen['offset_m'] != 0. and candidate[0] > 0:
                    selected_side = 'left' if chosen['offset_m'] > 0 else 'right'
                    threat = normal[2].get('selector_sides', {}).get(selected_side)
                    # 미관측 뒤차 부재 증명은 아니며 기존 소실/측방 위협을 보존한다.
                    if threat is not None and not threat.get('present') and not threat.get('seen'):
                        self.mode = 'BYPASS'
                        return candidate[0], candidate[1], dict(detail, mode=self.mode, reason='handoff_entry')
                    self.bypass.reset()
            else:
                self.bypass.reset()
            return target, normal[1], dict(detail, mode=self.mode, reason='handoff_follow')
        candidate = self.bypass.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        detail['bypass'] = candidate[2]
        centered = candidate[2].get('entry_phase') == 'centered'
        if centered:
            self.mode = 'REACQUIRE'
            self.reacquire = self.reacquire + 1 if fresh and normal[0] > 0 else self.reacquire
            if normal[0] <= 0:
                self.reacquire = 0
            if self.reacquire >= 2:
                self.mode = 'FOLLOW'; self.bypass.reset(); self.monitor.reset(); self.reacquire = 0
                return min(normal[0], .35), normal[1], dict(detail, mode=self.mode, reason='handoff_return')
        else:
            self.reacquire = 0
            self.mode = 'BYPASS'
        return candidate[0], candidate[1], dict(detail, mode=self.mode, reason='handoff_bypass')
