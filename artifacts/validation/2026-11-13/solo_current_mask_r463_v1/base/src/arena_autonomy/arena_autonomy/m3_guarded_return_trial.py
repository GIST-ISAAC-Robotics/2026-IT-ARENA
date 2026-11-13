"""복귀 뒤 실제 명령 관측을 유지하는 선택형 순수 시제품. 노드/launch 미연결.

기본 G1 복귀 명령이 지지되지 않으면 기록/활성 상태를 비우지 않고 기존 측방 후보
검사를 계속한다. 실제 복귀 뒤 새 명령이 거절되면 기존 HOLD처럼 0목표/이전각을 내며, 기본 제어기의 재개
카운터와 별도로 fresh 2회 재확인을 요구한다. 실제 STOP과 상위 입력 정지는 우선한다.
"""
from arena_autonomy.m3_execution_trial import M3ExecutionFollow, SceneHistory
from arena_autonomy.m3_return_guard import ReturnGuardAudit, assess_return_command


class GuardedReturnTrial:
    def __init__(self, follow_config=None, execution=None, *, enabled=False, allow_simulation_trial=False):
        if type(enabled) is not bool:
            raise ValueError('return guard option must be bool')
        self.inner = M3ExecutionFollow(follow_config, execution, allow_simulation_trial=allow_simulation_trial)
        self.enabled = enabled
        self.config = self.inner.config
        self.audit = ReturnGuardAudit()
        self.recent = SceneHistory()
        self.guard_held = False
        self.confirmations = 0
        self.last_epoch = self.last_source = self.last_ref = None

    def reset(self):
        """입력 오류 등의 일반 reset은 복귀 후 보호 필요성을 지우지 않는다."""
        self.inner.reset()
        self.recent.reset()
        self.guard_held = self.audit.ever_released
        self.confirmations = 0
        self.last_epoch = self.last_source = self.last_ref = None

    def reset_session(self):
        """새 경기/명시 초기화 전용. 운영 이관 전 호출 경계를 별도로 연결해야 한다."""
        self.audit.reset()
        self.reset()

    def command(self, scene, history, speed, side, previous_steering=0., control_dt=.02):
        if not self.enabled:
            return self.inner.command(scene, history, speed, side, previous_steering, control_dt)
        def review_release(baseline, frames):
            checked = assess_return_command(scene, frames, baseline[2].get('path_points'),
                target_speed=baseline[0], steering=baseline[1], measured_speed=speed,
                previous_steering=previous_steering, control_dt=control_dt,
                cfg=self.inner.lateral_config, execution=self.inner.execution)
            return {'accepted': checked['sampled_sweep_ok'], 'reason': checked['reason'], 'detail': checked}
        output = self.inner.command(scene, history, speed, side, previous_steering, control_dt,
                                    release_command_check=review_release)
        frames = self.recent.update(scene, history) if self.inner.execution.masked_history_support else []
        # 기본 객체의 내부 reset(새 epoch 등)이 외부 관문의 기억을 지우지 않게 분리했다.
        active = self.audit.observe_release(output[2])
        changed = (self.last_epoch is not None and self.last_epoch != scene.source_epoch) or (
            self.last_ref is not None and scene.t_ref_s < self.last_ref) or (
            self.last_source is not None and scene.source_first_s is not None and scene.source_first_s < self.last_source)
        fresh = scene.source_first_s is not None and (
            self.last_source is None or scene.source_epoch != self.last_epoch or scene.source_first_s > self.last_source)
        self.last_epoch, self.last_source, self.last_ref = scene.source_epoch, scene.source_first_s, scene.t_ref_s
        if changed and active:
            self.guard_held, self.confirmations = True, 0
        lateral = output[2].get('m3_lateral') or {}
        # 기존 측방 후보/상위 입력 STOP의 제어·명칭·조향은 그대로 둔다.
        if not active or lateral.get('mode') != 'BASELINE':
            if active:
                self.guard_held, self.confirmations = True, 0
            return output
        check = assess_return_command(scene, frames, output[2].get('path_points'),
            target_speed=output[0], steering=output[1], measured_speed=speed,
            previous_steering=previous_steering, control_dt=control_dt,
            cfg=self.inner.lateral_config, execution=self.inner.execution)
        detail = {**output[2], 'm3_return_guard': check, 'm3_guarded_return_trial': True,
                  'runtime_authorized': False, 'simulation_trial_only': True}
        if not check['sampled_sweep_ok']:
            self.guard_held, self.confirmations = True, 0
            # 이번 호출은 정지한다. 다음에는 기존 관측된 후보를 다시 검사하도록
            # 보호 상태만 활성화한다. 물체 부재나 새 자유 공간을 만들어 넣지 않는다.
            self.inner.lateral.active = True
            self.inner.held, self.inner.release_count = True, 0
            return 0., previous_steering, {**detail, 'reason': 'm3_return_guard_hold', 'behavior': 'HOLD'}
        # 0목표를 양수로 바꾸지 않는다. 지지되는 0목표 구간도 재개 확인을 대신하지 않는다.
        if output[0] <= 0:
            self.guard_held, self.confirmations = True, 0
            return output[0], output[1], detail
        if self.guard_held:
            if fresh:
                self.confirmations += 1
            if self.confirmations < self.config.confirmations:
                return 0., previous_steering, {**detail, 'reason': 'm3_return_guard_wait', 'behavior': 'HOLD'}
            self.guard_held = False
        return output[0], output[1], detail
