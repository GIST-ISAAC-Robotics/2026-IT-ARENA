"""명시 시뮬레이션 G1/G2 전환. 낮은 이동 반환은 정지/물체 신원 증명이 아니다."""
from dataclasses import dataclass, field, replace
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
# 정지 중 보정 뒤 이동은 0–2 cm였고 재출발 첫 스캔은 2.8 cm였다. 이 스캔에서 우회를 시작하지 않는다.
ENTRY_FORWARD_DRIFT_M = .02


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
    # 이전 기록에는 없는 필드: 재생 시 당시의 고정 우회 여유를 유지한다.
    moving_follow_reserve: bool = False
    closing_follow_reserve: bool = False
    recent_forward_reserve: bool = False
    # 연속 세 스캔의 낮은 이동이 쌓이기 전이나 이동 근거가 남은 동안 고정 우회 여유를 걸지 않는다.
    low_motion_reserve: bool = False
    # 전진율을 못 얻은 조각은 G1 전체가 아니라 물체 이동 가정만큼만 풀어 우회 공간을 남긴다.
    moving_pace_floor: bool = False
    # 최근 전진 관측으로 가정 속도 이상의 추종을 허용한다. 정지차 우회 여유는 유지한다.
    observed_forward_pace: bool = False
    # 이전 기록에는 없는 선택 필드(기본 OFF). 낮은 이동 누적 초기화와 별개인 연속 전진율이
    # 이번 새 스캔에서 양수일 때만 고정 우회 여유 상한에 그 전진율을 더한다.
    continuous_forward_pace: bool = False
    moving_pass_trial: bool = False
    moving_pass_profile: str = 'slow_v1'
    reacquire_policy: str = 'positive_command_v0'
    front_patch_policy: str = 'contiguous_three_v0'
    entry_policy: str = 'registered_v0'
    preparation_hold_policy: str = 'off'
    initial_reference_policy: str = 'off'
    motion_pause_policy: str = 'off'
    relative_yield_policy: str = 'off'
    # 전방 조각이 전진하기 시작한 스캔의 우회 진입을 막는 선택형 근거. 정지 판정이 아니다.
    entry_motion_gate: bool = False
    stopped_entry_centering: bool = False
    adaptive_entry_preview: bool = False

    def __post_init__(self):
        if not isinstance(self.bypass, BypassExecutionConfig):
            raise ValueError('typed bypass execution configuration required')
        if type(self.adaptive_entry_preview) is not bool or (self.adaptive_entry_preview and self.entry_policy != 'corridor_local_v1'):
            raise ValueError('adaptive preview requires explicit corridor handoff')
        if type(self.stopped_entry_centering) is not bool or (self.stopped_entry_centering and not (
                self.entry_policy == 'corridor_local_v1' and self.low_motion_reserve and self.entry_motion_gate)):
            raise ValueError('stopped entry centering requires observed low-motion corridor handoff')
        if self.preparation_policy not in PREPARATION_POLICIES:
            raise ValueError('registered preparation policy required')
        if self.persistence_window_policy not in PERSISTENCE_WINDOWS:
            raise ValueError('registered persistence window policy required')
        if self.front_patch_policy not in FRONT_PATCH_POLICIES:
            raise ValueError('registered front patch policy required')
        if type(self.follow_entry_space_enabled) is not bool:
            raise ValueError('boolean follow entry space option required')
        if self.reacquire_policy not in ('positive_command_v0','known_hold_v1','observed_rejoin_v2'):
            raise ValueError('registered reacquire policy required')
        if type(self.moving_follow_reserve) is not bool or (self.moving_follow_reserve and not self.follow_entry_space_enabled):
            raise ValueError('moving follow reserve requires follow entry space')
        if type(self.closing_follow_reserve) is not bool or (self.closing_follow_reserve and not self.moving_follow_reserve):
            raise ValueError('closing follow reserve requires moving follow evidence')
        if type(self.recent_forward_reserve) is not bool or (self.recent_forward_reserve and not self.closing_follow_reserve):
            raise ValueError('recent forward reserve requires closing follow evidence')
        if type(self.low_motion_reserve) is not bool or (self.low_motion_reserve and (
                not self.closing_follow_reserve or self.recent_forward_reserve)):
            raise ValueError('low motion reserve requires closing follow evidence without recent forward')
        if type(self.moving_pace_floor) is not bool or (self.moving_pace_floor and not self.low_motion_reserve):
            raise ValueError('moving pace floor requires low motion reserve')
        if type(self.observed_forward_pace) is not bool or (self.observed_forward_pace and not self.moving_pace_floor):
            raise ValueError('observed forward pace requires moving pace floor')
        if type(self.continuous_forward_pace) is not bool or (self.continuous_forward_pace and not self.observed_forward_pace):
            raise ValueError('continuous forward pace requires observed forward pace')
        if type(self.moving_pass_trial) is not bool or (self.moving_pass_trial and (
                not self.continuous_forward_pace or self.entry_policy != 'corridor_local_v1')):
            raise ValueError('moving pass requires continuous forward pace and corridor trial')
        if self.moving_pass_profile not in ('slow_v1', 'medium_v1', 'faster_v1') or (
                self.moving_pass_profile != 'slow_v1' and not self.moving_pass_trial):
            raise ValueError('moving pass profile requires an explicit moving pass trial')
        if self.follow_entry_space_enabled and (self.preparation_policy != 'observed_entry_space_v1'
                or self.persistence_window_policy != 'trim_before_compare_v1'):
            raise ValueError('follow entry space requires explicit space and bounded window policies')
        if self.entry_policy not in ('registered_v0','corridor_local_v1'):
            raise ValueError('registered handoff entry policy required')
        if self.preparation_hold_policy not in ('off', 'bounded_hold_v1'):
            raise ValueError('registered preparation hold policy required')
        if self.preparation_hold_policy != 'off' and self.entry_policy != 'corridor_local_v1':
            raise ValueError('preparation hold requires explicit corridor handoff')
        if self.initial_reference_policy not in ('off','ready_current_pair_v1','ready_directional_pair_v2'):
            raise ValueError('registered initial reference policy required')
        if self.initial_reference_policy != 'off' and self.entry_policy != 'corridor_local_v1':
            raise ValueError('initial reference requires explicit corridor handoff')
        if self.motion_pause_policy not in ('off','bounded_motion_pause_v1'):
            raise ValueError('registered motion pause policy required')
        if self.motion_pause_policy != 'off' and self.entry_policy != 'corridor_local_v1':
            raise ValueError('motion pause requires explicit signed corridor handoff')
        if self.relative_yield_policy not in ('off', 'bounded_relative_yield_v1', 'bounded_relative_yield_v2', 'observed_progress_yield_v3'):
            raise ValueError('registered relative yield policy required')
        if self.relative_yield_policy != 'off' and (
                self.entry_policy != 'corridor_local_v1'
                or self.motion_pause_policy != 'bounded_motion_pause_v1'):
            raise ValueError('relative yield requires explicit corridor and bounded motion pause')
        if self.entry_policy == 'corridor_local_v1' and (
                self.bypass.corridor_policy != 'corridor_v2_signed_hold'
                or self.preparation_policy != 'observed_entry_space_v1'
                or self.persistence_window_policy != 'trim_before_compare_v1'
                or self.follow_entry_space_enabled is not True
                or self.reacquire_policy not in ('known_hold_v1','observed_rejoin_v2')):
            raise ValueError('corridor handoff requires explicit signed, space, bounded window and known hold policies')
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
    def __init__(self, config=None, object_motion_bound_mps=None, forward_pace_limit_mps=1.5):
        self.config = config or HandoffExecutionConfig()
        self.motion_bound = (FollowConfig().object_motion_bound_mps if object_motion_bound_mps is None
                             else object_motion_bound_mps)
        self.forward_pace = None
        if self.config.observed_forward_pace:
            from arena_autonomy.front_patch_motion import ObservedForwardPace
            self.forward_pace = ObservedForwardPace(self.config.maximum_patch_displacement_m, forward_pace_limit_mps)
        self.recent = None
        if self.config.recent_forward_reserve:
            from arena_autonomy.front_patch_motion import RecentForward
            self.recent = RecentForward(self.config.maximum_patch_displacement_m,self.motion_bound)
        self.reset()

    def reset(self):
        self.rows = []; self.key = None; self.last_ref = None
        if self.forward_pace is not None:
            self.forward_pace.reset()
        if self.recent is not None:
            self.recent.reset()
        self.motion_rate_mps = None
        self.forward_rate_mps = 0.
        self.forward_drift_m = 0.
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
        if self.forward_pace is not None:
            self.forward_pace.observe(source, now, np.median(patch,axis=0) if patch is not None else None, history)
        if self.recent is not None:
            self.recent.observe(source,now,np.median(patch,axis=0) if patch is not None else None,history)
        if patch is None:
            self.rows = []; self.motion_rate_mps = None
            self.result = dict(ready=False, observed=False, fresh=True, reason=reason)
            return self.result
        deviations = []
        deltas = []
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
            delta = np.median(patch, axis=0) - np.median(projected, axis=0)
            deltas.append(delta)
            deviations.append(float(np.linalg.norm(delta)))
        self.forward_drift_m = float(deltas[0][0]) if deltas else 0.
        if deviations and max(deviations) > self.config.maximum_patch_displacement_m:
            bound = self.config.maximum_patch_displacement_m
            interval = source-self.rows[-1]['source']
            worst = int(np.argmax(deviations))
            span = source-self.rows[worst]['source']
            associated = interval > 0 and deviations[-1] <= bound+self.motion_bound*interval
            self.motion_rate_mps = deviations[worst]/span if associated and span > 0 else None
            # 거리 변화의 크기만 쓰면 마주 오는 물체나 옆에서 들어오는 차도 전진으로
            # 오인한다. 서로 다른 3스캔 이상에서 전방으로 벗어난 성분만 인정한다.
            dx, dy = deltas[worst]
            self.forward_rate_mps = (min(self.motion_bound, dx/span)
                if self.motion_rate_mps is not None and len(self.rows)-worst >= 2
                and dx > max(bound, abs(dy)) else 0.)
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
        if self.execution.moving_pass_profile == 'faster_v1':
            from arena_autonomy.moving_pass import moving_pass_profile
            bound = moving_pass_profile(self.execution.moving_pass_profile).object_motion_bound_mps
            # 명시 빠른 상대 후보는 일반 추종/진입/복귀에서 같은 시차 운동 예산을 쓴다.
            self.config = replace(self.config, object_motion_bound_mps=max(bound, self.config.object_motion_bound_mps))
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
        if self.execution.entry_policy == 'corridor_local_v1':
            from arena_autonomy.bypass_handoff_corridor_trial import configure_guide
            guide_type = None
            if self.execution.motion_pause_policy == 'bounded_motion_pause_v1':
                from arena_autonomy.bypass_motion_pause_trial import OverspeedHoldGuide
                guide_type = OverspeedHoldGuide
            configure_guide(self,guide_type)
            self.bypass.guide.adaptive_preview=self.execution.adaptive_entry_preview
            if self.execution.adaptive_entry_preview:
                from arena_autonomy.adaptive_corridor_offset import AdaptiveCorridorEntryConfig
                expanded = AdaptiveCorridorEntryConfig(**vars(self.bypass.guide.registered_entry))
                self.bypass.guide.registered_entry = self.bypass.guide.entry = expanded
        self.monitor = BlockingPersistence(self.execution, self.config.object_motion_bound_mps, self.config.open_route_speed_mps)
        self.mode = 'FOLLOW'; self.last_key = None; self.last_ref = None
        self.reacquire = 0
        self.moving_pass = None
        if self.execution.moving_pass_trial:
            from arena_autonomy.moving_pass import MovingPassAdapter
            self.moving_pass = MovingPassAdapter(self)

        self._preparation_hold = None
        self._preparation_history = None
        if self.execution.preparation_hold_policy == 'bounded_hold_v1':
            from arena_autonomy.bypass_preparation_hold import BoundedPreparationHold
            self._preparation_hold = BoundedPreparationHold()

        self._initial_reference = None
        if self.execution.initial_reference_policy != 'off':
            from arena_autonomy.bypass_initial_reference import ReadyInitialReference
            self._initial_reference = ReadyInitialReference(
                directional_coverage=self.execution.initial_reference_policy=='ready_directional_pair_v2')
            self.bypass.guide.reference_provider = self._straight_reference

        self._relative_yield = None
        if self.execution.relative_yield_policy == 'bounded_relative_yield_v1':
            from arena_autonomy.bypass_relative_yield import RelativeYieldCommand
            self._relative_yield = RelativeYieldCommand()
        elif self.execution.relative_yield_policy == 'bounded_relative_yield_v2':
            from arena_autonomy.bypass_relative_yield_pause import MotionPauseRelativeYieldCommand
            self._relative_yield = MotionPauseRelativeYieldCommand()
        elif self.execution.relative_yield_policy == 'observed_progress_yield_v3':
            from arena_autonomy.bypass_yield_progress import ProgressRelativeYieldCommand
            self._relative_yield = ProgressRelativeYieldCommand()

    def _front_patch_advancing(self):
        return self.execution.entry_motion_gate and self.monitor.forward_drift_m > ENTRY_FORWARD_DRIFT_M

    def _observed_rejoin(self, normal, candidate):
        """우회 복귀 중 현재 G1 경로가 유효하면 중앙 정렬 완료를 별도로 기다리지 않는다.

        전방 조각 소실 자체는 근거가 아니다. G2가 이미 복귀를 선택하고, G1의 실제
        경계/점유 검사와 측방 이력이 양수 경로를 허용할 때만 기존 두 스캔 확인에 진입한다.
        """
        phase=candidate[2].get('entry_phase')
        if self.execution.bypass.rear_wall_witness and phase is None:
            # HOLD도 실제 복귀 단계를 지우지 않는다. G2 벽 길이 거절은 G1의
            # 현재 경로/점유/측방 검사를 통과한 출력으로 인계할 수 없다는 뜻이 아니다.
            # 입력·시각 고장은 guide.reset()이 entry로 돌리므로 과거 허가를 잇지 않는다.
            phase=self.bypass.guide.phase
        if (self.execution.reacquire_policy != 'observed_rejoin_v2'
                or phase != 'rejoin' or normal[0] <= 0):
            return False
        nd = normal[2]
        path, sides = nd.get('path_points'), nd.get('selector_sides')
        if (nd.get('selector_choice') != 'baseline' or not isinstance(path, list) or len(path) < 2
                or not isinstance(sides, dict) or set(sides) != {'left', 'right'}):
            return False
        if any(s.get('present') or (s.get('seen') and not s.get('passed')) for s in sides.values()):
            return False
        return all(isinstance(nd.get(k), (int, float)) and not isinstance(nd[k], bool)
                   and math.isfinite(nd[k]) and nd[k] > 0
                   for k in ('observed_path_ahead_m', 'swept_clearance_m'))

    def _straight_reference(self,scene,config=None):
        config=self.config if config is None else config
        if self._initial_reference is None:
            return straight_reference(scene,config)
        return self._initial_reference.reference(scene,config,self.monitor)

    def _execution_entries(self, scene, speed, previous, dt):
        """다음 새 스캔 직전 실행 검사와 같은 표적 여유로 본 진입 후보. 허가/자유 공간 아님.

        여유 증가는 낮은 이동 누적의 마지막 두 스캔 간격(측정값)×G1 물체 이동 한도다.
        고정 외벽 분류·기본/측정 여유·후보·조향 한계는 G2 비교 그대로 쓴다.
        """
        if self.execution.entry_policy == 'corridor_local_v1':
            from arena_autonomy.bypass_handoff_corridor_trial import execution_entries
            return execution_entries(self,scene,speed,previous,dt)
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
        if self.execution.low_motion_reserve and len(self.monitor.rows) < 3:
            # 두 스캔의 6 cm는 0.6 m/s 이하일 뿐이다. 세 스캔(0.3 m/s 이하)이 쌓인 뒤 준비한다.
            return dict(result, reason='short_low_motion_history')
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
        reference, evidence = self._straight_reference(scene, cfg)
        if reference is None:
            return dict(result, reason='two_wall_straight_reference_unavailable',
                        reference_reason=evidence.get('reason'))
        return dict(result, applied=True, reason='observed_entry_preparation')

    def _follow_entry_space(self, scene, normal, previous, target):
        target, detail = self._follow_entry_space_current(scene, normal, previous, target)
        if (self.execution.recent_forward_reserve and detail['reason']=='observed_entry_distance_reserve'
                and self.monitor.motion_evidence() and self.monitor.recent.rate>0
                and detail.get('before_mps',0)>0):
            pace=detail['cap_mps']+self.monitor.recent.rate
            target=min(detail['before_mps'],pace)
            detail=dict(detail,applied=pace<detail['before_mps'],reason='recent_advancing_patch',
                cap_mps=pace,recent_forward_floor_mps=self.monitor.recent.rate)
        if self._preparation_hold is not None:
            return self._preparation_hold.limit(self, scene, self._preparation_history, target, detail)
        return target, detail

    def _follow_entry_space_current(self, scene, normal, previous, target):
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
        reference, evidence = self._straight_reference(scene, cfg)
        if reference is None:
            return target, dict(result, reason='two_wall_straight_reference_unavailable')
        # 희소 조각은 앞/뒤 기둥을 함께 포함할 수 있다. 중앙값으로 앞쪽 실제
        # 반환보다 더 먼 진입 공간을 주장하지 않도록 가장 가까운 점을 쓴다.
        front_x = (np.min(patch[:,0]) if self.execution.front_patch_policy == 'sparse_returns_v1'
                   else np.median(patch[:,0]))
        distance = float(front_x)+cfg.wheelbase_m/2
        reserve = max(self.bypass.guide.entry.transitions_m)
        cap = max(0.,(distance-reserve)/self.execution.observation_window_s)
        pace = self.monitor.forward_rate_mps if self.execution.moving_follow_reserve else 0.
        closing = self.execution.closing_follow_reserve and pace > 0.
        if self.execution.moving_follow_reserve and self.monitor.motion_evidence() and (closing or pace > cap):
            # G1의 경로/점유/제동 판정은 상한이다. 미래 우회를 위한 추가 제한만
            # 실제 전방 조각의 이동률까지 풀고, 정지·측방·불연속 반환에는 적용하지 않는다.
            # 움직이는 물체와 같은 속도만 내면 벌어진 간격은 줄지 않는다.
            # 선택형 비교에서는 관측된 전진율과 기존 우회 여유보다 먼 거리의 접근분을 더한다.
            if closing:
                pace += cap
            return min(target, pace), dict(result, applied=pace<target,
                reason='advancing_patch_close' if closing else 'advancing_patch_follow',
                before_mps=target, cap_mps=pace, fixed_reserve_cap_mps=cap,
                reserve_m=reserve, rear_axle_to_patch_m=distance)
        if self.execution.low_motion_reserve and (self.monitor.motion_evidence() or len(self.monitor.rows) < 3):
            # 고정 여유는 멈췄을 수 있는 조각의 미래 우회용이다. 전진율을 못 얻은 빠른 조각은
            # 전진율 0으로 취급하지 않고, 간격·감속·직접 충돌 판단을 G1 추종에 그대로 맡긴다.
            waiting = dict(result, reason='reserve_waits_low_motion', fixed_reserve_cap_mps=cap,
                reserve_m=reserve, rear_axle_to_patch_m=distance)
            if not self.execution.moving_pace_floor:
                return target, waiting
            # 앞차 속도를 모를 때 0으로 두지 않고 G1의 물체 이동 가정을 쓴다. 정확한 속도 측정이 아니며
            # 앞차가 그보다 빠르면 간격이 벌어질 뿐이다. 이 거리는 앞차 급정지 뒤의 우회 공간이다.
            observed = self.monitor.forward_pace
            credit = (observed.rate if observed is not None and self.monitor.motion_evidence() else 0.)
            paced = cap+max(self.monitor.motion_bound, credit)
            if observed is not None:
                waiting.update(observed_forward_pace_mps=credit, assumed_pace_mps=self.monitor.motion_bound,
                               pace_samples=len(observed.rows))
            return min(target,paced), dict(waiting, applied=paced<target, before_mps=target, cap_mps=paced)
        observed = self.monitor.forward_pace
        if (self.execution.continuous_forward_pace and observed is not None and observed.rate > 0.
                and observed.rows and observed.rows[-1][0] == scene.source_first_s):
            # 고정 여유는 멈춘 조각의 미래 우회용이다. 이번 새 스캔까지 연속 전진한 조각에는
            # 보수적 전진율(끝점 6 cm 차감, 마지막 간격 전진율 이하)만큼만 더 푼다. 정지·감속·
            # 측방·튐·중복 시각에서는 그 값이 0이 되어 아래 고정 여유로 돌아간다. G1 출력이 상한이다.
            paced = cap+observed.rate
            return min(target,paced), dict(result, applied=paced<target, reason='continuous_forward_pace',
                rear_axle_to_patch_m=distance, reserve_m=reserve, fixed_reserve_cap_mps=cap,
                observed_forward_pace_mps=observed.rate, pace_samples=len(observed.rows),
                cap_mps=paced, before_mps=target)
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
        if self.moving_pass is not None:
            self.moving_pass.reset()
        self.mode = 'FOLLOW'; self.last_key = None; self.last_ref = None; self.reacquire = 0
        self._preparation_history = None
        if self._preparation_hold is not None:
            self._preparation_hold.clear()
        if self._initial_reference is not None:
            self._initial_reference.clear()
        if self._relative_yield is not None:
            self._relative_yield.reset()

    def _discard_path_only(self):
        if self.execution.entry_policy == 'corridor_local_v1':
            from arena_autonomy.bypass_handoff_corridor_trial import discard_path_only
            discard_path_only(self)
        else:
            self.bypass.reset()

    def command(self, scene, history, speed, side, previous, dt, *, raw_scan=None):
        from arena_autonomy.local_follow import forward_planning_speed, signed_feedback_detail
        planning = forward_planning_speed(self.config, speed)
        if self.moving_pass is not None:
            output = self.moving_pass.command(scene, history, planning, side, previous, dt, raw_scan=raw_scan)
        else:
            output = self._base_command(scene, history, planning, side, previous, dt, raw_scan=raw_scan)
        return signed_feedback_detail(output, speed, planning)

    def _base_command(self, scene, history, planning, side, previous, dt, *, raw_scan=None):
        if self._relative_yield is not None:
            output = self._relative_yield.command(self, scene, planning,
                lambda: self._prepared_command(scene, history, planning, side, previous, dt, raw_scan=raw_scan))
        else:
            output = self._prepared_command(scene, history, planning, side, previous, dt, raw_scan=raw_scan)
        return output

    def _prepared_command(self, scene, history, speed, side, previous, dt, *, raw_scan=None):
        if self._preparation_hold is None:
            return self._command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        self._preparation_history = history
        output = self._command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        self._preparation_hold.observe(self, scene, output)
        return output

    def _command(self, scene, history, speed, side, previous, dt, *, raw_scan=None):
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
        if self.execution.entry_policy == 'corridor_local_v1':
            self.bypass.guide.motion_history = history
        normal = self.normal.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        persistence = self.monitor.update(scene, history)
        fresh = key != self.last_key
        self.last_key = key; self.last_ref = scene.t_ref_s
        detail = dict(mode_before=self.mode, normal=normal[2], persistence=persistence,
                      scope='explicit_simulation_handoff_trial')
        fault = normal[2].get('selector_choice') == 'hold_fault'
        if fault and self.execution.motion_pause_policy == 'bounded_motion_pause_v1':
            from arena_autonomy.bypass_motion_pause_trial import signed_bypass_pause
            fault = not signed_bypass_pause(self,normal,speed)
        if fault or not source_valid(scene) or not math.isfinite(speed) or (speed < 0 and
                (self.execution.entry_policy != 'corridor_local_v1' or self.mode == 'FOLLOW')):
            self.bypass.reset(); self.monitor.reset(); self.mode = 'FOLLOW'; self.reacquire = 0
            if self._initial_reference is not None:
                self._initial_reference.clear()
            # 유한한 음의 엔코더 속도는 기존 G1이 정지하면서 관측/옆차 이력을
            # 보존하는 입력이다. 새 우회 경로만 버리고 G1 전체를 reset하지 않는다.
            return self._output(0., previous, detail,
                'handoff_reverse_motion_hold' if speed < 0 else 'handoff_input_fault')
        if self._initial_reference is not None:
            self._initial_reference.update(scene,self.config,history)
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
                # 낮은 이동 후보에서는 위 고정 여유가 1.20 m 앞에서 멈춘다. 측면 관측은 계속 쌓되
                # 별도 0 대기를 겹치지 않는다.
                if any(t == longest for _, t in usable) and not self.execution.low_motion_reserve:
                    target = 0.
                    detail['entry_space']['hold'] = True
                else:
                    detail['entry_space']['hold'] = False
            hold_steer = normal[1]
            if (self.execution.stopped_entry_centering and target == 0. and speed == 0.
                    and normal[0] > 0. and normal[2].get('selector_choice') == 'baseline'
                    and persistence.get('ready') and not self._front_patch_advancing()
                    and detail.get('follow_entry_space', {}).get('reason') == 'observed_entry_distance_reserve'):
                # 정지한 단일 모터축의 최근 표본을 확인한다. 차체 속도/물체 신원의 증명은 아니다.
                # 정상 G1은 전진을 허용했으나 우회 여유만 0인 동안, 기존 3 rad/s로 명령을 정렬한다.
                # 속도를 만들거나 우회 진입 조건을 바꾸지 않는다. 양수 우회 후보는 아래 원 검사로만 나온다.
                from arena_autonomy.stopped_entry_centering import stopped_shaft_window
                if stopped_shaft_window(history, scene.t_ref_s):
                    hold_steer = math.copysign(max(0., abs(previous) - 3. * dt), previous)
                    detail['stopped_entry_centering'] = dict(previous_rad=previous, requested_rad=hold_steer,
                        new_motion_authorized=False, shaft_quiet_window_s=.20)
            # .35는 출력 상한이다. 28카운트/15:1의 20 ms 속도 양자화로 측정값은
            # .340/.365를 오가므로 이를 입력 허용선으로 쓰면 관측을 매번 버린다.
            # 측정 속도는 그대로 G2에 전달하여 기존 반응/제동 영역에서 검사한다.
            if (persistence['ready'] and not self._front_patch_advancing() and speed <= self.bypass.config.max_speed_mps
                    and abs(previous) <= self.execution.maximum_initial_steering_rad):
                candidate = self.bypass.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
                chosen = candidate[2].get('selected')
                detail['bypass'] = candidate[2]
                if chosen is not None and chosen['offset_m'] != 0. and candidate[0] > 0 and space:
                    # 진입 순간의 여유로만 고른 경로가 다음 새 스캔 전에 막히면 반쯤 들어가 멈춘다.
                    usable, detail['entry_admission'] = self._execution_entries(scene, speed, previous, dt)
                    if (chosen['offset_m'], chosen['transition_m']) not in usable:
                        self._discard_path_only()
                        detail['entry_admission']['admitted'] = False
                        return self._output(target, hold_steer, detail, 'handoff_follow')
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
                self._discard_path_only()
            return self._output(target, hold_steer, detail, 'handoff_follow')
        candidate = self.bypass.command(scene, history, speed, side, previous, dt, raw_scan=raw_scan)
        detail['bypass'] = candidate[2]
        centered = candidate[2].get('entry_phase') == 'centered'
        observed_rejoin = self._observed_rejoin(normal, candidate)
        if self.execution.reacquire_policy == 'observed_rejoin_v2':
            detail['observed_rejoin_ready'] = observed_rejoin
        if centered or observed_rejoin:
            self.mode = 'REACQUIRE'
            # 경로/점유 검사를 완료한 G1의 간격 정지 역시 정상 출력이다. 이전의
            # 양수만 허용하는 연결은 이미 중앙에 복귀했어도 G2로 계속 접근했다.
            # 명시 후보에서만 알려진 정지 이유와 경로 근거를 인정하며 고장/경계
            # 미확정을 정상 준비로 바꾸지 않는다. 서로 다른 두 스캔은 그대로다.
            nd = normal[2]
            known_hold = (self.execution.reacquire_policy in ('known_hold_v1','observed_rejoin_v2') and normal[0] == 0.
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
