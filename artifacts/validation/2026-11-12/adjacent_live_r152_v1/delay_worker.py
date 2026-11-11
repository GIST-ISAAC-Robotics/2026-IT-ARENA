"""시험 전용 배송 지연. 원 시각을 보존하고 실제 완료보다 앞서 내주지 않는다."""
import bisect
import math


class DelayedWorker:
    def __init__(self, worker, clock, timeline):
        if (not timeline or any(not math.isfinite(t) or not math.isfinite(delay) or t<0 or delay<0
                                for t,delay in timeline)
                or any(b[0]<=a[0] for a,b in zip(timeline,timeline[1:]))):
            raise ValueError('ordered nonnegative delay timeline required')
        self.worker,self.clock=worker,clock
        self.times=[t-timeline[0][0] for t,_ in timeline]
        self.delays=[delay for _,delay in timeline]
        self.origin=None;self.current=None;self.held=None
        self.delayed=0;self.canceled=0

    def __getattr__(self,name):
        return getattr(self.worker,name)

    def offer(self,data):
        if self.current is not None or self.held is not None:
            raise RuntimeError('delay worker already active')
        reference=data['reference_s']
        if self.origin is None:self.origin=reference
        index=max(0,bisect.bisect_right(self.times,reference-self.origin)-1)
        self.current=(reference,self.delays[index],index,self.worker.generation)
        self.worker.offer(data)

    def pump(self):
        if self.held is None:
            result=self.worker.pump()
            if result is not None:
                if self.current is None:
                    raise RuntimeError('delay result without request')
                self.held=result
        if self.held is None:return None
        reference,delay,index,generation=self.current
        if generation!=self.worker.generation:
            self.held=self.current=None;self.canceled+=1
            return None
        now=self.clock()
        if now+1e-9<reference+delay:return None
        result=self.held
        self.current=self.held=None;self.delayed+=1
        return {**result,'injected_delay_s':delay,'delay_timeline_index':index,
                'released_sim_s':now,'delay_scope':'test_delivery_only_not_CPU_prediction'}

    def invalidate(self):
        if self.current is not None or self.held is not None:self.canceled+=1
        self.current=self.held=None
        self.worker.invalidate()

    def close(self):
        self.current=self.held=None
        return self.worker.close()
