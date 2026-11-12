"""관측 전용 C1 시계 후보. 센서 취득 정확도/차체 장착/구동을 승인하지 않는다."""
from collections import Counter
import statistics
from arena_vehicle_interface.c1_native_preview import C1NativePreview


class C1SampleClock:
    time_basis='unverified_estimate'
    estimator='tracking_sample_clock_v1'

    def __init__(self,*,source_epoch,frame_id='c1_unmounted_preview'):
        self.raw=C1NativePreview(source_epoch=source_epoch,frame_id=frame_id)
        self.epoch=source_epoch;self.counts=Counter();self.last_raw_epoch=source_epoch
        self.last_host=None;self.last_scan=None;self.last_sdk=None
        self._clear()

    def _clear(self):
        self.prefix=[];self.model_rows=[];self.batch_count=0;self.samples=0;self.period=None;self.intercept=None;self.base=None
        self.last_published_end=None
        self.last_mapped_end_ns=None

    def _reset(self,reason):
        self.epoch+=1
        if self.epoch>=2**63:raise ValueError('source epoch exhausted')
        self.counts['reset_'+reason]+=1;self._clear()

    def convert(self,batch,*,receipt_monotonic_ns,receipt_ros_ns):
        nominal,diagnostic=self.raw.convert(batch,receipt_monotonic_ns=receipt_monotonic_ns,
                                           receipt_ros_ns=receipt_ros_ns)
        raw_epoch=nominal['source_epoch'];first=batch['sdk_estimated_first_us']*1000
        reason=None
        if raw_epoch!=self.last_raw_epoch:reason='source_epoch'
        elif self.last_scan is not None and batch['scan']!=self.last_scan+1:reason='scan_gap'
        elif self.last_sdk is not None and not 70_000_000<=first-self.last_sdk<=130_000_000:reason='revolution_gap'
        if reason:self._reset(reason)
        self.last_raw_epoch=raw_epoch;self.last_scan=batch['scan'];self.last_sdk=first
        self.last_host=receipt_monotonic_ns
        n=len(batch['nodes']);tick=self.samples;self.samples+=n;self.batch_count+=1
        self.model_rows.append(dict(tick=tick,n=n,first=first,end=batch['end_monotonic_ns']))
        self.model_rows=self.model_rows[-100:]
        diagnostic={**diagnostic,'time_estimator':self.estimator,'nominal_source_epoch':raw_epoch,
            'source_epoch':self.epoch,'absolute_accuracy_verified':False,'model_reset':reason,
            'nominal_diagnostics_preserved':True,'sample_clock_ready':self.period is not None,
            'timestamps_clipped':False}
        if self.base is None:self.base=first
        if self.period is None:
            self.prefix.append(dict(tick=tick,n=n,first=first,end=batch['end_monotonic_ns']))
            self.counts['warmup_batches']+=1
            if len(self.prefix)==20:
                slopes=[(b['first']-a['first'])/(b['tick']-a['tick'])
                        for i,a in enumerate(self.prefix) for j,b in enumerate(self.prefix) if j-i>=8]
                period=statistics.median(slopes)
                if not 180_000<=period<=220_000:
                    self._reset('rate_outside_candidate');return None,diagnostic,None
                self.period=period
                self.intercept=min(min(r['first']-self.base-r['tick']*period,
                    r['end']-self.base-(r['tick']+r['n']-1)*period) for r in self.prefix)
            return None,diagnostic,None
        prior_first=self.base+round(self.intercept+tick*self.period)
        if abs(first-prior_first)>10_000_000:
            self._reset('anchor_residual');diagnostic.update(rejected='anchor_residual',sample_clock_ready=False)
            return None,diagnostic,None
        refitted=False
        if self.batch_count%10==0:
            window=self.model_rows
            minimum_span=max(8,len(window)//2)
            slopes=[(b['first']-a['first'])/(b['tick']-a['tick'])
                    for i,a in enumerate(window) for j,b in enumerate(window) if j-i>=minimum_span]
            period=statistics.median(slopes)
            if not 180_000<=period<=220_000:
                self._reset('rate_outside_candidate');diagnostic.update(rejected='rate_outside_candidate',sample_clock_ready=False)
                return None,diagnostic,None
            self.period=period
            self.intercept=min(min(r['first']-self.base-r['tick']*period,
                r['end']-self.base-(r['tick']+r['n']-1)*period) for r in window)
            refitted=True
        desired_last=self.base+round(self.intercept+(tick+n-1)*self.period)
        effective=self.period
        if self.last_published_end is None:
            stamps=[self.base+round(self.intercept+(tick+i)*self.period) for i in range(n)]
        else:
            # 이미 발행한 시각은 바꾸지 않는다. 이번 회전의 표본 간격에 위상 수정을 분배한다.
            effective=(desired_last-self.last_published_end)/n
            if not 180_000<=effective<=220_000:
                self._reset('phase_step_outside_candidate');diagnostic.update(rejected='phase_step_outside_candidate',sample_clock_ready=False)
                return None,diagnostic,None
            stamps=[self.last_published_end+round((i+1)*effective) for i in range(n)]
        diagnostic.update(model_refitted=refitted,effective_sample_period_ns=effective,
                          previously_published_stamps_changed=False)
        residual=first-stamps[0];future=max(0,stamps[-1]-receipt_monotonic_ns)
        diagnostic.update(sample_clock_ready=True,sample_period_ns=self.period,
            sdk_anchor_residual_ns=residual,publication_wait_ns=future,
            sample_index_first=tick,estimated_end_monotonic_ns=stamps[-1])
        # 아래 한계는 관측 후보의 일관성/대기 예산이다. 실제 센서 오차의 상한이 아니다.
        failure=('anchor_residual' if abs(residual)>10_000_000 else
                 'publication_wait' if future>5_000_000 else
                 'nonmonotonic' if stamps[0]<0 or (self.last_published_end is not None and stamps[0]<=self.last_published_end)
                 else None)
        if failure:
            self._reset(failure);diagnostic.update(rejected=failure,sample_clock_ready=False)
            return None,diagnostic,None
        offset=receipt_ros_ns-receipt_monotonic_ns
        if self.last_mapped_end_ns is not None and stamps[0]+offset<=self.last_mapped_end_ns:
            self._reset('mapped_clock_overlap')
            diagnostic.update(rejected='mapped_clock_overlap',sample_clock_ready=False,
                              source_epoch=self.epoch)
            return None,diagnostic,None
        mapped=[(s+offset)/1e9 for s in stamps]
        if mapped[0]<0 or any(b<=a for a,b in zip(mapped,mapped[1:])):
            raise ValueError('mapped candidate clock precision/range invalid')
        payload={**nominal,'acquired_at_s':mapped,'source_epoch':self.epoch}
        self.last_published_end=stamps[-1];self.last_mapped_end_ns=stamps[-1]+offset
        self.counts['candidate_batches']+=1
        self.counts['deferred_batches']+=bool(future)
        return payload,diagnostic,max(receipt_monotonic_ns,stamps[-1])
