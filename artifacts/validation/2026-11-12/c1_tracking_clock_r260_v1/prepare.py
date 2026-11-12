"""r259 원본을 보존하고 관측 전용 추적 후보만 별도 사본으로 만든다."""
from pathlib import Path
A=Path(__file__).resolve().parent;old=A.parent/'c1_sample_clock_r259_v1'
if any((A/name).exists() for name in ('sample_clock.py','test_tracking.py','replay.py')):
    raise FileExistsError('초기 생성 전용: 검증한 후보와 후속 수정은 덮어쓰지 않습니다.')
source=(old/'sample_clock.py').read_text()
source=source.replace("estimator='prefix_sample_clock_v1'","estimator='tracking_sample_clock_v1'")
source=source.replace('self.prefix=[];self.samples=0;', 'self.prefix=[];self.model_rows=[];self.batch_count=0;self.samples=0;')
source=source.replace("n=len(batch['nodes']);tick=self.samples;self.samples+=n", "n=len(batch['nodes']);tick=self.samples;self.samples+=n;self.batch_count+=1\n        self.model_rows.append(dict(tick=tick,n=n,first=first,end=batch['end_monotonic_ns']))\n        self.model_rows=self.model_rows[-100:]")
start=source.index('        stamps=[self.base+round(')
end=source.index('        residual=first-stamps[0]',start)
replacement='''        prior_first=self.base+round(self.intercept+tick*self.period)
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
'''
source=source[:start]+replacement+source[end:]
(A/'sample_clock.py').write_text(source)
tests=(old/'test_clock.py').read_text()
(A/'test_tracking.py').write_text(tests)
replay=(old/'replay.py').read_text().replace("('r249','r251')","('r249','r251','r259')")
replay=replay.replace("folder=ROOT/'build'/f'{name}_recovered/c1'","folder=ROOT/'build'/f'{name}_recovered'/('live' if name=='r259' else 'c1')")
replay=replay.replace("                assert payload[key]==record['payload'][key],key", "                expected=record['payload']\n                assert expected is not None and payload[key]==expected[key],key")
(A/'replay.py').write_text(replay)
