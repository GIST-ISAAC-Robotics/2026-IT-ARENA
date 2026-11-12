"""평가 전용 두 단계 결과 지연. 실제 완료/원천 시각·세대는 바꾸지 않는다."""
import math


class StagedDelay:
    def __init__(self,worker,clock,plan_s,recheck_s):
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<0 for v in (plan_s,recheck_s)):
            raise ValueError('finite nonnegative stage delays required')
        self.worker,self.clock=worker,clock
        self.delays={'plan':plan_s,'recheck':recheck_s}
        self.current=self.held=None
        self.released={'plan':0,'recheck':0};self.canceled=0

    def __getattr__(self,name):return getattr(self.worker,name)

    def _offer(self,data,stage,plan=None):
        if self.current is not None or self.held is not None:raise RuntimeError('delayed stage already active')
        if stage=='plan':self.worker.offer(data)
        else:self.worker.offer_recheck(data,plan)
        self.current=(data['reference_s'],stage,self.worker.generation)

    def offer(self,data):self._offer(data,'plan')
    def offer_recheck(self,data,plan):self._offer(data,'recheck',plan)

    def pump(self):
        if self.held is None:
            result=self.worker.pump()
            if result is not None:
                if self.current is None or result['stage']!=self.current[1]:
                    raise RuntimeError('delayed completion stage differs')
                self.held=result
        if self.held is None:return None
        reference,stage,generation=self.current
        if generation!=self.worker.generation:
            self.held=self.current=None;self.canceled+=1
            return None
        now=self.clock();delay=self.delays[stage]
        if now+1e-9<reference+delay:return None
        result=self.held;self.current=self.held=None;self.released[stage]+=1
        return {**result,'injected_delay_s':delay,'released_sim_s':now,
                'delay_scope':'fixed_stage_p95_minimum_age_not_CPU_prediction'}

    def invalidate(self):
        if self.current is not None or self.held is not None:self.canceled+=1
        self.held=self.current=None;self.worker.invalidate()

    def close(self):
        self.held=self.current=None
        return self.worker.close()
