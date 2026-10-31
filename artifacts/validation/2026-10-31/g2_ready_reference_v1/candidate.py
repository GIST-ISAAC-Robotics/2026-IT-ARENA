"""오프라인 후보: 현재 반환 기준 보완을 기존 우회 준비 근거에 한정한다.

움직이는 앞차의 기존 거리 보존을 해제하지 않는다. 기존 양벽 기준이 실패한
호출에서만 낮은 이동의 누적 완료·현재 반환을 함께 요구해 보완을 제공한다.
"""
from contextlib import ExitStack
import importlib.util
from pathlib import Path
from unittest.mock import patch
import arena_autonomy.bypass_handoff as handoff
import arena_autonomy.bypass_entry_trial as entry
import arena_autonomy.bypass_corridor_trial as corridor

SOURCE=Path(__file__).resolve().parent.parent/'g2_initial_wall_candidate_v1/candidate.py'
spec=importlib.util.spec_from_file_location('initial_wall_saved_candidate',SOURCE)
prior=importlib.util.module_from_spec(spec);spec.loader.exec_module(prior)


def ready_for_reference(monitor):
    result=monitor.result
    return (result.get('observed') is True and result.get('ready') is True
            and not monitor.motion_evidence())


class ReadyPairFollow(handoff.BypassHandoffFollow):
    def __init__(self,*args,**kwargs):
        self.reference_memory=prior.CurrentPairReference();self.last_reference=None
        self.reference_use={}
        super().__init__(*args,**kwargs)

    def reset(self):
        super().reset();self.reference_memory.clear();self.last_reference=None;self.reference_use={}

    def command(self,scene,history,speed,*args,**kwargs):
        if speed<-.02:self.reference_memory.clear()
        proposed=self.reference_memory.resolve(scene,self.config,history)
        original=prior.ORIGINAL_REFERENCE(scene,self.config)
        self.last_reference=proposed
        self.reference_use=dict(proposed=proposed[1].get('reason'),fallback_calls=0,withheld_calls=0)
        def scoped(current,config):
            assert current is scene and config is self.config,'offline scope mismatch'
            if original[0] is not None or proposed[0] is None:return original
            if ready_for_reference(self.monitor):
                self.reference_use['fallback_calls']+=1
                return proposed
            self.reference_use['withheld_calls']+=1
            return original
        with ExitStack() as stack:
            for module in (handoff,entry,corridor):stack.enter_context(patch.object(module,'straight_reference',scoped))
            output=super().command(scene,history,speed,*args,**kwargs)
        if output[2].get('reason') in ('handoff_input_fault','handoff_invalid_input',
                'handoff_source_reset','handoff_reverse_motion_hold'):self.reference_memory.clear()
        return output
