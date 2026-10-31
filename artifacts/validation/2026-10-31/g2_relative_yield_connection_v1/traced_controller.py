"""합성 평가기 옵션을 실제 typed 실행 설정으로 변환하는 어댑터."""
from dataclasses import replace
from arena_autonomy.bypass_handoff import BypassHandoffFollow


class ProductionFollow(BypassHandoffFollow):
    def __init__(self,*,config,execution,allow_simulation_trial,yield_enabled=False):
        self.yield_enabled=yield_enabled
        self.last_yield_detail={}
        super().__init__(config=config,execution=replace(execution,
            relative_yield_policy='bounded_relative_yield_v1' if yield_enabled else 'off'),
            allow_simulation_trial=allow_simulation_trial)

    def reset(self):
        super().reset()
        self.last_yield_detail={}

    def command(self,*args,**kwargs):
        output=super().command(*args,**kwargs)
        self.last_yield_detail=dict(output[2].get('relative_yield') or {},enabled=self.yield_enabled)
        return output
