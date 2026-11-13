"""C1 Standard 10 Hz의 선택형 호스트 추적 시계. 취득 정확도 교정은 아니다.

SDK 첫 점 추정과 실제 점 개수의 과거 기록으로 주기를 추정한다. 이미 보낸
시각은 바꾸지 않는다. 준비/재시작 때는 배열을 보내지 않고, 최대 5 ms 미래인
끝 점은 호출자가 ready_monotonic_ns까지 기다린 뒤 발행해야 한다.
"""
from collections import Counter
import statistics

from .c1_native_preview import C1NativePreview


class C1SampleClock:
    time_basis = 'unverified_estimate'
    estimator = 'tracking_sample_clock_v1'
    WARMUP_BATCHES = 20
    WINDOW_BATCHES = 100
    REFIT_BATCHES = 10
    PERIOD_MIN_NS = 180_000
    PERIOD_MAX_NS = 220_000
    ANCHOR_RESIDUAL_NS = 10_000_000
    MAX_PUBLICATION_WAIT_NS = 5_000_000

    def __init__(self, *, source_epoch, frame_id='c1_unmounted_preview'):
        self.raw = C1NativePreview(source_epoch=source_epoch, frame_id=frame_id)
        self.epoch = source_epoch
        self.counts = Counter()
        self.last_raw_epoch = source_epoch
        self.last_scan = self.last_sdk = None
        self._clear()

    def _clear(self):
        self.model_rows = []
        self.batch_count = self.samples = 0
        self.period = self.intercept = self.base = None
        self.last_published_end = self.last_mapped_end_ns = None

    def reset(self, reason, diagnostic=None):
        if self.epoch >= 2**63 - 1:
            raise ValueError('source epoch exhausted')
        self.epoch += 1
        self.counts['reset_' + reason] += 1
        self._clear()
        if diagnostic is not None:
            diagnostic.update(rejected=reason, sample_clock_ready=False,
                              source_epoch=self.epoch, model_reset=reason)

    def _fit(self):
        rows = self.model_rows
        # r260과 같이 초기20창은8차, 이후 추적은 창의 절반 이상을 비교한다.
        span = 8 if self.period is None else max(8, len(rows) // 2)
        slopes = [(b['first'] - a['first']) / (b['tick'] - a['tick'])
                  for i, a in enumerate(rows) for j, b in enumerate(rows)
                  if j - i >= span]
        period = statistics.median(slopes)
        if not self.PERIOD_MIN_NS <= period <= self.PERIOD_MAX_NS:
            return False
        self.period = period
        self.intercept = min(min(r['first'] - self.base - r['tick'] * period,
                                 r['end'] - self.base - (r['tick'] + r['n'] - 1) * period)
                             for r in rows)
        return True

    def convert(self, batch, *, receipt_monotonic_ns, receipt_ros_ns):
        nominal, diagnostic = self.raw.convert(
            batch, receipt_monotonic_ns=receipt_monotonic_ns,
            receipt_ros_ns=receipt_ros_ns)
        raw_epoch = nominal['source_epoch']
        first = batch['sdk_estimated_first_us'] * 1000
        reason = None
        if raw_epoch != self.last_raw_epoch:
            reason = 'source_epoch'
        elif self.last_scan is not None and batch['scan'] != self.last_scan + 1:
            reason = 'scan_gap'
        elif self.last_sdk is not None and not 70_000_000 <= first - self.last_sdk <= 130_000_000:
            reason = 'revolution_gap'
        if reason:
            self.reset(reason)
        self.last_raw_epoch, self.last_scan, self.last_sdk = raw_epoch, batch['scan'], first
        n, tick = len(batch['nodes']), self.samples
        self.samples += n
        self.batch_count += 1
        self.model_rows.append(dict(tick=tick, n=n, first=first, end=batch['end_monotonic_ns']))
        self.model_rows = self.model_rows[-self.WINDOW_BATCHES:]
        diagnostic = {**diagnostic, 'time_estimator': self.estimator,
                      'nominal_source_epoch': raw_epoch, 'source_epoch': self.epoch,
                      'absolute_accuracy_verified': False, 'model_reset': reason,
                      'nominal_diagnostics_preserved': True,
                      'sample_clock_ready': self.period is not None, 'timestamps_clipped': False}
        if self.base is None:
            self.base = first
        if self.period is None:
            self.counts['warmup_batches'] += 1
            if len(self.model_rows) == self.WARMUP_BATCHES and not self._fit():
                self.reset('rate_outside_candidate', diagnostic)
            return None, diagnostic, None
        prior_first = self.base + round(self.intercept + tick * self.period)
        if abs(first - prior_first) > self.ANCHOR_RESIDUAL_NS:
            self.reset('anchor_residual', diagnostic)
            return None, diagnostic, None
        refitted = self.batch_count % self.REFIT_BATCHES == 0
        if refitted and not self._fit():
            self.reset('rate_outside_candidate', diagnostic)
            return None, diagnostic, None
        desired_last = self.base + round(self.intercept + (tick + n - 1) * self.period)
        effective = self.period
        if self.last_published_end is None:
            stamps = [self.base + round(self.intercept + (tick + i) * self.period) for i in range(n)]
        else:
            # 과거 시각은 고정하고 이번 회전의 간격에만 위상 수정을 분배한다.
            effective = (desired_last - self.last_published_end) / n
            if not self.PERIOD_MIN_NS <= effective <= self.PERIOD_MAX_NS:
                self.reset('phase_step_outside_candidate', diagnostic)
                return None, diagnostic, None
            stamps = [self.last_published_end + round((i + 1) * effective) for i in range(n)]
        residual = first - stamps[0]
        future = max(0, stamps[-1] - receipt_monotonic_ns)
        diagnostic.update(model_refitted=refitted, effective_sample_period_ns=effective,
                          previously_published_stamps_changed=False, sample_clock_ready=True,
                          sample_period_ns=self.period, sdk_anchor_residual_ns=residual,
                          publication_wait_ns=future, sample_index_first=tick,
                          estimated_end_monotonic_ns=stamps[-1])
        # 일관성·대기 예산이며 실제 센서 시각 오차의 상한이 아니다.
        failure = ('anchor_residual' if abs(residual) > self.ANCHOR_RESIDUAL_NS else
                   'publication_wait' if future > self.MAX_PUBLICATION_WAIT_NS else
                   'nonmonotonic' if stamps[0] < 0 or (self.last_published_end is not None
                                                      and stamps[0] <= self.last_published_end) else None)
        if failure:
            self.reset(failure, diagnostic)
            return None, diagnostic, None
        offset = receipt_ros_ns - receipt_monotonic_ns
        if self.last_mapped_end_ns is not None and stamps[0] + offset <= self.last_mapped_end_ns:
            self.reset('mapped_clock_overlap', diagnostic)
            return None, diagnostic, None
        mapped = [(s + offset) / 1e9 for s in stamps]
        if mapped[0] < 0 or any(b <= a for a, b in zip(mapped, mapped[1:])):
            raise ValueError('mapped candidate clock precision/range invalid')
        payload = {**nominal, 'acquired_at_s': mapped, 'source_epoch': self.epoch}
        self.last_published_end = stamps[-1]
        self.last_mapped_end_ns = stamps[-1] + offset
        self.counts['candidate_batches'] += 1
        self.counts['deferred_batches'] += bool(future)
        return payload, diagnostic, max(receipt_monotonic_ns, stamps[-1])
