"""짧은 signed 운동 대기에서 감속 의도만 보존하는 명시 v2 시험 후보."""
from dataclasses import replace
from arena_autonomy.bypass_relative_yield import RelativeYieldCommand,finite,observation


class MotionPauseRelativeYieldCommand(RelativeYieldCommand):
    maximum_motion_pause_s=1.

    def __init__(self):
        super().__init__()
        self.pause=None

    def reset(self):
        super().reset()
        self.pause=None

    def fresh(self,scene):
        observer=self.observer
        return bool(observer.rows and observer.key
            and getattr(scene,'source_epoch',None)==observer.key[0]
            and finite(getattr(scene,'t_ref_s',None))
            and finite(getattr(scene,'source_first_s',None))
            and 0<=scene.t_ref_s-scene.source_first_s<=observer.scan_gap_s
            and scene.source_first_s>=observer.key[1]
            and observer.yield_since is not None
            and 0<=scene.t_ref_s-observer.yield_since<=observer.maximum_continuation_s)

    def pause_valid(self,scene):
        p=self.pause
        return bool(p and self.fresh(scene)
            and 0<=scene.t_ref_s-p['start']<=self.maximum_motion_pause_s
            and 0<=scene.t_ref_s-p['last_ref']<=self.observer.scan_gap_s
            and 0<=scene.source_first_s-p['last_source']<=self.observer.scan_gap_s)

    def command(self,owner,scene,speed,evaluate):
        observer=self.observer;guide=owner.bypass.guide
        prior=bool(observer.detail.get('suggest_yield') and self.fresh(scene)
            and owner.mode=='BYPASS' and guide.phase=='entry' and guide.path is not None)
        paused_valid=self.pause_valid(scene) if self.pause is not None else False
        continuity=(paused_valid if self.pause is not None else bool(prior
            and 0<=scene.t_ref_s-observer.rows[-1]['source']<=observer.scan_gap_s))
        requested=bool(prior and continuity and finite(speed) and 0<=speed<=owner.config.max_speed_mps)
        original_entry,original_registered=guide.entry,guide.registered_entry
        try:
            if requested:
                guide.entry=replace(original_entry,speed_cap_mps=min(original_entry.speed_cap_mps,.20))
                guide.registered_entry=replace(original_registered,speed_cap_mps=min(original_registered.speed_cap_mps,.20))
            output=evaluate()
        finally:
            guide.entry,guide.registered_entry=original_entry,original_registered
        detail=output[2];bypass=detail.get('bypass') or {}
        reason=bypass.get('reason')
        known_pause=(reason=='entry_reverse_recovery_wait'
            or (reason=='entry_reverse_motion_hold' and bypass.get('geometry_retained') is True))
        retained=bool(prior and continuity and known_pause and output[0]==0.
            and detail.get('handoff_state')=='BYPASS' and owner.mode=='BYPASS'
            and guide.phase=='entry' and guide.path is not None
            and finite(speed) and abs(speed)<=owner.config.max_speed_mps)
        if retained:
            start=self.pause['start'] if self.pause else scene.t_ref_s
            self.pause=dict(start=start,last_ref=scene.t_ref_s,last_source=scene.source_first_s)
            advice=dict(observer.detail,motion_pause_intent_retained=True,
                pause_elapsed_s=scene.t_ref_s-start,current_observation_claimed=False,
                motion_permission_retained=False)
        else:
            current=observation(output,getattr(scene,'t_ref_s',None))
            # 정상 재개 첫 호출도 현재 면/벽/시각/5초예산을 다시 검사한다.
            # 대기 중 연속 유효 센서 호출을 확인한 경우에만 면 표본의 시간 공백을 허용한다.
            if self.pause is not None and paused_valid and prior and current is not None:
                advice=dict(observer.continue_observation(current,motion_pause_gap=True))
                advice['motion_pause_resumed']=advice.get('suggest_yield',False)
            else:
                if self.pause is not None:observer.reset()
                advice=dict(observer.update(current))
            self.pause=None
        return output[0],output[1],dict(detail,relative_yield=dict(cap_requested=requested,
            applied_cap_mps=.20 if requested else None,current_observer=advice,
            motion_pause_intent_retained=retained,measurement_replaced=False,
            final_command_clipped=False,source='bounded_motion_pause_intent_current_command_checks'))
