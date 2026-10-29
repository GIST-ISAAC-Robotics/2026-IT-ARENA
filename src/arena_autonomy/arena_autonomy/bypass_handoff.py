"""명시 시뮬레이션 G1/G2 전환. 낮은 이동 반환은 정지/물체 신원 증명이 아니다."""
from dataclasses import dataclass, field
import math
import numpy as np

from arena_autonomy.bypass_entry_follow import BypassEntryFollow, BypassExecutionConfig
from arena_autonomy.bypass_entry_trial import straight_reference, _compare_entries
from arena_autonomy.lidar_motion import MotionUnavailable
from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.local_path import swept_limit
from arena_autonomy.observation_support import source_valid
from arena_autonomy.scene_motion import transport_points

# front_patch_v0: 현재 전방 조각이 보이면 곧바로 준비 감속(10/29 run3까지의 기록, 기본).
# observed_entry_relevance_v1: 연속 낮은 이동·이동 증거·G1 원호·G2 두 벽 기준을 함께 본다.
# 멈추는 앞차의 진입 간격이 줄고 복귀가 미해결이므로 명시 선택할 때만 쓴다.
# observed_entry_space_v1: v1 준비 조건에 더해, 다음 새 스캔 직전 여유로도 경로가 비는
# 진입만 허용하고 준비 완료 전에는 가장 긴 등록 전이의 그런 후보가 남아 있으면 멈춰 기다린다.
PREPARATION_POLICIES = ('front_patch_v0', 'observed_entry_relevance_v1', 'observed_entry_space_v1')
PERSISTENCE_WINDOWS = ('compare_then_trim_v0', 'trim_before_compare_v1')
FRONT_PATCH_POLICIES = ('contiguous_three_v0', 'sparse_returns_v1')


def handoff_baseline():
    return FollowConfig(partial_observation_policy='bounded_local_v4', parallel_hold_trial=True,
        pass_release_trial=True, guard_curve_trial=True, traffic_recovery_trial=True,
        traffic_path_recovery_trial=True, corridor_fallback_enabled=True,
        boundary_evidence_mode='corner_distance_candidate', occupancy_timing_mode='per_ray_candidate')


@dataclass(frozen=True)
class HandoffExecutionConfig:
    bypass: BypassExecutionConfig = field(default_factory=BypassExecutionConfig)
    observation_window_s: float = .8
    maximum_patch_displacement_m: float = .06
    minimum_distinct_scans: int = 6
    preparation_cap_mps: float = .35
    maximum_initial_steering_rad: float = .08
    reacquire_distinct_scans: int = 2
    preparation_policy: str = 'front_patch_v0'
    persistence_window_policy: str = 'compare_then_trim_v0'
    follow_entry_space_enabled: bool = False
    reacquire_policy: str = 'positive_command_v0'
    front_patch_policy: str = 'contiguous_three_v0'

    def __post_init__(self):
        if not isinstance(self.bypass, BypassExecutionConfig):
            raise ValueError('typed bypass execution configuration required')
        if self.preparation_policy not in PREPARATION_POLICIES:
            raise ValueError('registered preparation policy required')
        if self.persistence_window_policy not in PERSISTENCE_WINDOWS:
            raise ValueError('registered persistence window policy required')
        if self.front_patch_policy not in FRONT_PATCH_POLICIES:
            raise ValueError('registered front patch policy required')
        if type(self.follow_entry_space_enabled) is not bool:
            raise ValueError('boolean follow entry space option required')
        if self.reacquire_policy not in ('positive_command_v0','known_hold_v1'):
            raise ValueError('registered reacquire policy required')
        if self.follow_entry_space_enabled and (self.preparation_policy != 'observed_entry_space_v1'
                or self.persistence_window_policy != 'trim_before_compare_v1'):
            raise ValueError('follow entry space requires explicit space and bounded window policies')
        for name, lo, hi in (('observation_window_s', .5, 1.5),
                             ('maximum_patch_displacement_m', .01, .06),
                             ('preparation_cap_mps', .05, .35),
                             ('maximum_initial_steering_rad', 0., .08)):
            value = getattr(self, name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lo <= value <= hi:
                raise ValueError('invalid handoff option: ' + name)
        for name, lo, hi in (('minimum_distinct_scans', 6, 20), ('reacquire_distinct_scans', 2, 10)):
            value = getattr(self, name)
            if type(value) is not int or not lo <= value <= hi:
                raise ValueError('invalid handoff count: ' + name)

    @classmethod
    def from_mapping(cls, values):
        values = dict(values)
        values['bypass'] = BypassExecutionConfig(**values['bypass'])
        # 선택지가 생기기 전의 schema5 기록은 당시 동작(즉시 준비 감속)으로 재생한다.
        values.setdefault('preparation_policy', 'front_patch_v0')
        return cls(**values)


def front_patch(scene, policy='contiguous_three_v0'):
    """전방의 실제 반환 조각. 물체 ID/전체 외곽/정지의 증명이 아니다."""
    if policy not in FRONT_PATCH_POLICIES:
        raise ValueError('registered front patch policy required')
    if policy == 'sparse_returns_v1':
        # 개방형 상대차의 작은 기둥에는 한두 광선만 걸릴 수 있다. 인접 광선의
        # 연결 대신 현재의 실제 반환3개를 좁은 관측 창에서 누적한다. 빈틈을
        # 채우거나 한 물체/정지차로 판정하지 않는다. 경로 점유 검사는 원래
        # 장면의 모든 반환을 그대로 사용하며 이 조각은 준비/진입 관측만 맡는다.
        points = np.asarray([ray.hit_xy_m for ray in scene.rays if ray.hit_xy_m is not None
            and .20 <= ray.hit_xy_m[0] <= 1.60 and abs(ray.hit_xy_m[1]) <= .24])
        if len(points) < 3:
            return None, 'sparse_front_too_few_returns'
        front = points[points[:,0] <= float(np.min(points[:,0])) + .20]
        if len(front) < 3 or not np.any(abs(front[:,1]) < .15):
            return None, 'sparse_front_too_few_near_returns'
        if float(np.ptp(front[:,1])) > .24:
            return None, 'sparse_front_lateral_spread'
        return front, 'sparse_front_returns'
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

    이동 증거(`motion_rate_mps`)는 준비 감속 판단용 부가 상태이며 `result`의
    기존 항목·준비 판정을 바꾸지 않는다. 직전 스캔과 연관되는 조각이 60 mm를
    넘게 움직이면 같은 조각의 이동으로 보고 그 관측 속도를 남긴다. 연관 범위
    (60 mm+G1 물체 이동 한도×간격)를 넘으면 다른 반환으로 바뀐 새 조각이다.
    """
    def __init__(self, config=None, object_motion_bound_mps=None):
        self.config = config or HandoffExecutionConfig()
        self.motion_bound = (FollowConfig().object_motion_bound_mps if object_motion_bound_mps is None
                             else object_motion_bound_mps)
        self.reset()

    def reset(self):
        self.rows = []; self.key = None; self.last_ref = None
        self.motion_rate_mps = None
        self.result = dict(ready=False, observed=False, fresh=False, reason='waiting')

    def motion_evidence(self):
        """마지막 이동 속도로 보아 아직 60 mm 안에 머물 수 있는 짧은 누적인가.

        이동 속도 u로 계속 움직였다면 누적 구간이 60 mm/u를 넘기 전에 다시
        초기화되어야 한다. 그보다 오래 머물렀으면 감속/정지 가능성이 있으므로
        이동 증거로 준비 감속을 막지 않는다. 정지/차량 ID 판정은 아니다.
        """
        if self.motion_rate_mps is None or not self.rows:
            return False
        span = self.rows[-1]['source']-self.rows[0]['source']
        return span < self.config.maximum_patch_displacement_m/self.motion_rate_mps

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
        patch, reason = front_patch(scene, self.config.front_patch_policy)
        if patch is None:
            self.rows = []; self.motion_rate_mps = None
            self.result = dict(ready=False, observed=False, fresh=True, reason=reason)
            return self.result
        deviations = []
        if self.config.persistence_window_policy == 'trim_before_compare_v1':
            # 다음 조각을 비교하기 전에 창 밖의 오래된 시작 표본을 제거한다.
            # 두 번째 표본부터도 지정 창 전체를 덮을 때만 제거하므로, 짧은 관측을
            # 준비 완료로 승격하지 않는다. 예전 기록의 비교→정리 순서는 기본 보존.
            while len(self.rows) > 1 and source-self.rows[1]['source'] >= self.config.observation_window_s-1e-9:
                self.rows.pop(0)
        for old in self.rows:
            try:
                projected = transport_points(old['patch'], history, old['time'], now)
            except ValueError:
                self.rows = []; deviations = []; self.motion_rate_mps = None; break
            deviations.append(float(np.linalg.norm(np.median(patch, axis=0)
                                                   - np.median(projected, axis=0))))
        if deviations and max(deviations) > self.config.maximum_patch_displacement_m:
            bound = self.config.maximum_patch_displacement_m
            interval = source-self.rows[-1]['source']
            worst = int(np.argmax(deviations))
            span = source-self.rows[worst]['source']
            associated = interval > 0 and deviations[-1] <= bound+self.motion_bound*interval
            self.motion_rate_mps = deviations[worst]/span if associated and span > 0 else None
            self.rows = []
        self.rows.append(dict(source=source, time=now, patch=patch.copy()))
        # 판정 창을 처음부터 다시 누적하지 않고 최소 0.8초의 연속 관측을 유지한다.
        while len(self.rows) > 2 and source-self.rows[1]['source'] >= self.config.observation_window_s:
            self.rows.pop(0)
        span = source-self.rows[0]['source']
        self.result = dict(ready=len(self.rows) >= self.config.minimum_distinct_scans and span >= self.config.observation_window_s-1e-9,
            observed=True, fresh=True, reason='low_motion_patch_candidate',
            sample_count=len(self.rows), source_span_s=span,
            max_compensated_displacement_m=max(deviations, default=0.),
            front_median_xy=np.median(patch, axis=0).tolist(),
            identity_or_stationarity_proven=False)
        return self.result


class BypassHandoffFollow:
    def __init__(self, config=None, execution=None, *, allow_simulation_trial=False):
        if allow_simulation_trial is not True:
            raise ValueError('handoff requires explicit simulation trial')
        self.config = config or handoff_baseline()
        self.execution = execution or HandoffExecutionConfig()
        if not isinstance(self.execution, HandoffExecutionConfig):
            raise ValueError('typed handoff execution configuration required')
        required = ('parallel_hold_trial', 'pass_release_trial', 'guard_curve_trial',
                    'traffic_recovery_trial', 'traffic_path_recovery_trial', 'corridor_fallback_enabled')
        if (self.config.partial_observation_policy != 'bounded_local_v4'
                or self.config.boundary_evidence_mode != 'corner_distance_candidate'
                or self.config.occupancy_timing_mode != 'per_ray_candidate'
                or not all(getattr(self.config, key) for key in required)
                or self.config.recent_observation_enabled
                or type(self.config.short_boundary_prefix_enabled) is not bool):
            raise ValueError('handoff requires the recorded integrated G1 baseline')
        # G1의 선택형 짧은 경계 관측은 실제 교통/5랩 구성에서 켜져 있다.
        # 명시된 값을 보존하며 G2 진입 관측 허가로 복사하지 않는다. 초기 제한
        # G2 비교의 OFF 기록도 같은 코드로 그대로 재생할 수 있어야 한다.
        self.normal = LocalFollow(self.config)
        self.bypass = BypassEntryFollow(self.config, self.execution.bypass, allow_simulation_trial=True)
        self.monitor = BlockingPersistence(self.execution, self.config.object_motion_bound_mps)
        self.mode = 'FOLLOW'; self.last_key = None; self.last_ref = None
        self.reacquire = 0

    def _execution_entries(self, scene, speed, previous, dt):
        """다음 새 스캔 직전 실행 검사와 같은 표적 여유로 본 진입 후보. 허가/자유 공간 아님.

        여유 증가는 낮은 이동 누적의 마지막 두 스캔 간격(측정값)×G1 물체 이동 한도다.
        고정 외벽 분류·기본/측정 여유·후보·조향 한계는 G2 비교 그대로 쓴다.
        """
        rows = self.monitor.rows
        growth = rows[-1]['source']-rows[-2]['source'] if len(rows) >= 2 else None
        if growth is None or not 0 < growth <= .25:
            return [], dict(reason='scan_interval_unavailable')
        result = _compare_entries(scene, scene.t_ref_s, speed, previous, dt, config=self.config,
            entry=self.bypass.guide.entry, stationary_boundaries=True, padding_growth_s=growth)
        usable = sorted((r['offset_m'], r['transition_m']) for r in result['candidates']
            if r.get('eligible') and r['offset_m'] and r['preview_point_gap_m'] >= 0
            and r['preview_return_progress_m'] >= r['path_points'][-1][0]-r['path_points'][0][0]-1e-9)
        return usable, dict(reason=result['reason'], padding_growth_s=growth, consistent=[list(u) for u in usable],
                            free_space_assumed=False, full_bypass_verified=False)

    def _preparation(self, scene, history, normal, persistence, previous, require_above_cap=True):
        """준비 감속 근거. 정지/ID·자유 공간 증명이 아니며 우회 허가와 분리된다.

        싼 조건부터 확인한다. G1이 이미 상한 이하이면 감속 차이가 없으므로 원호/
        두 벽 계산을 생략한다. 기록 항목은 마지막으로 확인한 조건을 남긴다.
        """
        cap = self.execution.preparation_cap_mps
        result = dict(policy=self.execution.preparation_policy, applied=False, cap_mps=cap,
                      identity_or_stationarity_proven=False, free_space_assumed=False)
        if not persistence.get('observed'):
            return dict(result, reason='no_current_front_patch')
        if require_above_cap and not normal[0] > cap:
            return dict(result, reason='g1_command_within_cap')
        if not normal[0] > 0:
            # 위치 유지 판단(진입 공간 정책)에서도 G1 정지 중에는 더 볼 것이 없다.
            return dict(result, reason='g1_command_hold')
        if self.monitor.motion_evidence():
            return dict(result, reason='patch_motion_observed', motion_rate_mps=self.monitor.motion_rate_mps)
        if len(self.monitor.rows) < 2:
            # 한 스캔은 이동/정지 어느 쪽의 비교 근거도 없다.
            return dict(result, reason='single_scan_patch')
        latest = self.monitor.rows[-1]
        try:
            patch = transport_points(latest['patch'], history, latest['time'], scene.t_ref_s)
        except (ValueError, MotionUnavailable):
            return dict(result, reason='patch_transport_unavailable')
        cfg = self.config
        # G1 점유 검사와 같은 원호·차체이며, 광선별 나이 대신 장면 전체 나이의
        # 더 큰 여유를 쓴다. 조각이 G1의 현재 진행을 막지 않으면 G1이 처리한다.
        margin = .025+cfg.measurement_padding_m+cfg.object_motion_bound_mps*max(0., scene.t_ref_s-scene.source_first_s)
        arc = min(swept_limit(patch, angle, cfg.wheelbase_m, horizon=2.5, margin=margin,
                              length=cfg.length_m, width=cfg.width_m) for angle in (normal[1], previous))
        result['patch_arc_clearance_m'] = float(arc)
        if arc >= 2.5:
            return dict(result, reason='patch_outside_g1_swept_arc')
        # G2 진입의 초기 조건과 같은 현재 두 벽 직선 기준. 곡선/한쪽 벽 결측에서는
        # 우회를 시작할 수 없으므로 준비 감속도 하지 않는다. 경로 허가로 쓰지 않는다.
        reference, evidence = straight_reference(scene, cfg)
        if reference is None:
            return dict(result, reason='two_wall_straight_reference_unavailable',
                        reference_reason=evidence.get('reason'))
        return dict(result, applied=True, reason='observed_entry_preparation')

    def _follow_entry_space(self, scene, normal, previous, target):
        """명시 시험용 전방 추종 여유. 물체 ID/정지 판정·우회 허가가 아니다.

        현재 진행 원호를 막는 조각과 양벽 직선이 관측되면 가장 긴 진입 전이의
        거리를 남긴다. 그 거리를 넘는 부분만 관측 창 동안 접근하도록 상한을 둔다.
        이동 앞차의 속도 정답을 쓰지 않고, 느려지거나 멈추는 앞차에도 같은 식이다.
        이는 추가 정체/랩 시간 손실을 수반할 수 있는 시험 전략이며 기본은 OFF다.
        """
        result = dict(applied=False, reason='unconfirmed_front_patch', free_space_assumed=False,
                      identity_or_stationarity_proven=False)
        if len(self.monitor.rows) < 2 and not self.monitor.motion_evidence():
            return target, result
        patch, reason = front_patch(scene, self.execution.front_patch_policy)
        if patch is None:
            return target, dict(result, reason=reason)
        cfg = self.config
        margin = .025+cfg.measurement_padding_m+cfg.object_motion_bound_mps*max(0.,scene.t_ref_s-scene.source_first_s)
        arc = min(swept_limit(patch, angle, cfg.wheelbase_m, horizon=2.5, margin=margin,
                              length=cfg.length_m, width=cfg.width_m) for angle in (normal[1],previous))
        if arc >= 2.5:
            return target, dict(result, reason='patch_outside_g1_swept_arc')
        reference, evidence = straight_reference(scene, cfg)
        if reference is None:
            return target, dict(result, reason='two_wall_straight_reference_unavailable')
        # 희소 조각은 앞/뒤 기둥을 함께 포함할 수 있다. 중앙값으로 앞쪽 실제
        # 반환보다 더 먼 진입 공간을 주장하지 않도록 가장 가까운 점을 쓴다.
        front_x = (np.min(patch[:,0]) if self.execution.front_patch_policy == 'sparse_returns_v1'
                   else np.median(patch[:,0]))
        distance = float(front_x)+cfg.wheelbase_m/2
        reserve = max(self.bypass.guide.entry.transitions_m)
        cap = max(0.,(distance-reserve)/self.execution.observation_window_s)
        return min(target,cap), dict(result, applied=cap<target, reason='observed_entry_distance_reserve',
            rear_axle_to_patch_m=distance, reserve_m=reserve, approach_window_s=self.execution.observation_window_s,
            cap_mps=cap, before_mps=target)

    def _output(self, target, steering, detail, reason):
        return target, steering, dict(detail, mode='g1_g2_handoff_trial', handoff_state=self.mode,
            reason=reason, behavior='HOLD' if target <= 0 else self.mode,
            simulation_trial_only=True, automatic_race_mode_selection=True,
            output_requires_independent_guard=True)

    def reset(self):
        self.normal.reset(); self.bypass.reset(); self.monitor.reset()
        self.mode = 'FOLLOW'; self.last_key = None; self.last_ref = None; self.reacquire = 0

    def command(self, scene, history, speed, side, previous, dt, *, raw_scan=None):
        if (raw_scan is None or not all(isinstance(v, (int,float)) and not isinstance(v,bool)
                and math.isfinite(v) for v in (speed,previous,dt))
                or abs(previous) > self.config.max_steering_rad or not 0 < dt <= .2
                or side not in ('left','right') or not source_valid(scene)):
            self.reset()
            return self._output(0., 0., {}, 'handoff_invalid_input')
        key = (scene.source_frame_id, scene.source_epoch, scene.source_first_s)
        if self.last_key is not None and (key[:2] != self.last_key[:2]
                or key[2] is None or self.last_key[2] is None or key[2] < self.last_key[2]
                or scene.t_ref_s < self.last_ref):
            self.reset()
            return self._output(0., previous, {}, 'handoff_source_reset')
        # G1의 옆차/소실 이력은 G2 중에도 관측한다. 모드 전환만으로 지우지 않는다.
        normal = self.normal.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        persistence = self.monitor.update(scene, history)
        fresh = key != self.last_key
        self.last_key = key; self.last_ref = scene.t_ref_s
        detail = dict(mode_before=self.mode, normal=normal[2], persistence=persistence,
                      scope='explicit_simulation_handoff_trial')
        fault = normal[2].get('selector_choice') == 'hold_fault'
        if fault or not source_valid(scene) or not math.isfinite(speed) or speed < 0:
            self.bypass.reset(); self.monitor.reset(); self.mode = 'FOLLOW'; self.reacquire = 0
            # 유한한 음의 엔코더 속도는 기존 G1이 정지하면서 관측/옆차 이력을
            # 보존하는 입력이다. 새 우회 경로만 버리고 G1 전체를 reset하지 않는다.
            return self._output(0., previous, detail,
                'handoff_reverse_motion_hold' if speed < 0 else 'handoff_input_fault')
        if self.mode == 'FOLLOW':
            # 확실한 현재 전방 반환을 보면서 관측 시간을 확보한다. 반환이 없어지면
            # G1을 따르며, 무반환 자체를 장애물/우회 신호로 사용하지 않는다.
            if self.execution.preparation_policy == 'front_patch_v0':
                prepare = persistence['observed']
            else:
                detail['preparation'] = self._preparation(scene, history, normal, persistence, previous,
                    require_above_cap=self.execution.preparation_policy != 'observed_entry_space_v1')
                prepare = detail['preparation']['applied']
            # 준비 감속 여부와 무관하게 낮은 이동 누적과 아래 진입 조건은 같다.
            target = min(normal[0], self.execution.preparation_cap_mps) if prepare else normal[0]
            if self.execution.follow_entry_space_enabled:
                target, detail['follow_entry_space'] = self._follow_entry_space(scene, normal, previous, target)
            space = self.execution.preparation_policy == 'observed_entry_space_v1'
            if space and prepare and not persistence['ready']:
                usable, detail['entry_space'] = self._execution_entries(scene, speed, previous, dt)
                longest = max(self.bypass.guide.entry.transitions_m)
                # 가장 완만한 진입이 아직 가능하면 더 다가가지 않는다. 이미 없으면 기다려도
                # 공간이 돌아오지 않으므로 G1을 따른다(아래 진입 검사는 그대로).
                if any(t == longest for _, t in usable):
                    target = 0.
                    detail['entry_space']['hold'] = True
                else:
                    detail['entry_space']['hold'] = False
            # .35는 출력 상한이다. 28카운트/15:1의 20 ms 속도 양자화로 측정값은
            # .340/.365를 오가므로 이를 입력 허용선으로 쓰면 관측을 매번 버린다.
            # 측정 속도는 그대로 G2에 전달하여 기존 반응/제동 영역에서 검사한다.
            if (persistence['ready'] and speed <= self.bypass.config.max_speed_mps
                    and abs(previous) <= self.execution.maximum_initial_steering_rad):
                candidate = self.bypass.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
                chosen = candidate[2].get('selected')
                detail['bypass'] = candidate[2]
                if chosen is not None and chosen['offset_m'] != 0. and candidate[0] > 0 and space:
                    # 진입 순간의 여유로만 고른 경로가 다음 새 스캔 전에 막히면 반쯤 들어가 멈춘다.
                    usable, detail['entry_admission'] = self._execution_entries(scene, speed, previous, dt)
                    if (chosen['offset_m'], chosen['transition_m']) not in usable:
                        self.bypass.reset()
                        detail['entry_admission']['admitted'] = False
                        return self._output(target, normal[1], detail, 'handoff_follow')
                    detail['entry_admission']['admitted'] = True
                if chosen is not None and chosen['offset_m'] != 0. and candidate[0] > 0:
                    selected_side = 'left' if chosen['offset_m'] > 0 else 'right'
                    threat = normal[2].get('selector_sides', {}).get(selected_side)
                    # 미관측 뒤차 부재 증명은 아니며 기존 소실/측방 위협을 보존한다.
                    if threat is not None and not threat.get('present') and not threat.get('seen'):
                        self.mode = 'BYPASS'
                        return self._output(candidate[0], candidate[1], detail, 'handoff_entry')
                    self.bypass.reset()
            else:
                self.bypass.reset()
            return self._output(target, normal[1], detail, 'handoff_follow')
        candidate = self.bypass.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        detail['bypass'] = candidate[2]
        centered = candidate[2].get('entry_phase') == 'centered'
        if centered:
            self.mode = 'REACQUIRE'
            # 경로/점유 검사를 완료한 G1의 간격 정지 역시 정상 출력이다. 이전의
            # 양수만 허용하는 연결은 이미 중앙에 복귀했어도 G2로 계속 접근했다.
            # 명시 후보에서만 알려진 정지 이유와 경로 근거를 인정하며 고장/경계
            # 미확정을 정상 준비로 바꾸지 않는다. 서로 다른 두 스캔은 그대로다.
            nd = normal[2]
            known_hold = (self.execution.reacquire_policy == 'known_hold_v1' and normal[0] == 0.
                and nd.get('reason') in ('follow_gap_hold','follow_release_wait')
                and isinstance(nd.get('path_points'),list) and len(nd['path_points']) >= 2
                and isinstance(nd.get('observed_path_ahead_m'),(int,float)) and nd['observed_path_ahead_m'] > 0
                and isinstance(nd.get('swept_clearance_m'),(int,float))
                and math.isfinite(nd['swept_clearance_m']) and nd['swept_clearance_m'] >= 0)
            ready = normal[0] > 0 or known_hold
            self.reacquire = self.reacquire + 1 if fresh and ready else self.reacquire
            if not ready:
                self.reacquire = 0
            if self.reacquire >= self.execution.reacquire_distinct_scans:
                self.mode = 'FOLLOW'; self.bypass.reset(); self.monitor.reset(); self.reacquire = 0
                return self._output(min(normal[0], self.execution.preparation_cap_mps), normal[1], detail, 'handoff_return')
            if known_hold:
                return self._output(0., normal[1], detail, 'handoff_reacquire_hold')
        else:
            self.reacquire = 0
            self.mode = 'BYPASS'
        return self._output(candidate[0], candidate[1], detail, 'handoff_bypass')
