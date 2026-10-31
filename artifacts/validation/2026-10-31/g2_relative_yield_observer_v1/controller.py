"""오프라인 합성 시험 전용. 이전 호출의 감속 제안으로 이번 후보를 다시 계산한다.

생산 코드/CLI에는 연결하지 않는다. 최종 명령의 사후 clipping이 아니며,
실제 측정 속도와 현재 광선의 점유/관측/제동 검사를 그대로 사용한다.
"""
from dataclasses import replace
from arena_autonomy.bypass_handoff import BypassHandoffFollow
from observer import RelativeYieldObserver, observation, finite


class RelativeYieldFollow(BypassHandoffFollow):
    def __init__(self, *args, yield_enabled=False, **kwargs):
        if type(yield_enabled) is not bool:
            raise ValueError('explicit trial boolean required')
        self.yield_enabled = yield_enabled
        self.yield_observer = RelativeYieldObserver()
        self.last_yield_detail = {}
        super().__init__(*args, **kwargs)

    def reset(self):
        super().reset()
        self.yield_observer.reset()
        self.last_yield_detail = {}

    def command(self, scene, history, speed, direction, previous, dt, **kwargs):
        observer = self.yield_observer
        # 과거 제안은 감속 상한일 뿐 현재 입력의 통과 여부가 아니다. 다음 실제
        # 검사를 같은 history/speed/scene로 단 한 번 호출한다.
        valid_prior = bool(observer.rows and observer.detail.get('suggest_yield')
            and observer.key[0] == scene.source_epoch
            and finite(scene.t_ref_s) and finite(scene.source_first_s)
            and 0 <= scene.t_ref_s-observer.rows[-1]['source'] <= observer.scan_gap_s
            and scene.source_first_s >= observer.rows[-1]['source']
            and finite(speed) and 0 <= speed <= self.config.max_speed_mps
            and self.mode == 'BYPASS' and self.bypass.guide.phase == 'entry'
            and self.bypass.guide.path is not None)
        apply = self.yield_enabled and valid_prior
        guide = self.bypass.guide
        original_entry, original_registered = guide.entry, guide.registered_entry
        try:
            if apply:
                guide.entry = replace(original_entry,
                    speed_cap_mps=min(original_entry.speed_cap_mps, observer.trial_speed_cap_mps))
                guide.registered_entry = replace(original_registered,
                    speed_cap_mps=min(original_registered.speed_cap_mps, observer.trial_speed_cap_mps))
            output = super().command(scene, history, speed, direction, previous, dt, **kwargs)
        finally:
            guide.entry, guide.registered_entry = original_entry, original_registered
        current = observation(dict(input=dict(reference_s=scene.t_ref_s), output=output))
        advice = dict(observer.update(current))
        self.last_yield_detail = dict(enabled=self.yield_enabled, cap_requested=apply,
            applied_cap_mps=observer.trial_speed_cap_mps if apply else None,
            current_observer=advice, measurement_replaced=False,
            final_command_clipped=False, source='previous_call_advice_current_command_checks')
        return output
