"""시험 강도 확인: 일부러 틀린 표본 구간 변형을 넣으면 r43 래스터 시험이 실패해야 한다. src는 바꾸지 않는다."""
import inspect, json, sys
import numpy as np
import arena_autonomy.lateral_intent as li
import test_grid_cost_r43 as t

ORIGINAL = li.ObservationGrid._free_sample_window


def variant(lo_pad, hi_pad, enlarge, drop_last=False):
    def window(self, o, vec, dist, free, counts, step):
        first, kept = ORIGINAL(self, o, vec, dist, free, counts, step)
        if (first == 0).all() and (kept == counts).all():
            return first, kept
        r = self.r*enlarge
        unit = vec/np.maximum(dist, 1e-9)[:, None]
        lo_t, hi_t = np.zeros(len(o)), free
        with np.errstate(all='ignore'):
            for axis, (lo, hi) in enumerate(((self.x0-r, self.x0+self.nx*self.r+r), (self.y0-r, self.y0+self.ny*self.r+r))):
                p, d = o[:, axis], unit[:, axis]
                a, b = (lo-p)/d, (hi-p)/d
                still = d == 0
                within = (p >= lo) & (p <= hi)
                lo_t = np.maximum(lo_t, np.where(still, np.where(within, -np.inf, np.inf), np.minimum(a, b)))
                hi_t = np.minimum(hi_t, np.where(still, np.where(within, np.inf, -np.inf), np.maximum(a, b)))
        hit = lo_t <= hi_t
        lo_t, hi_t = np.where(hit, lo_t, 0.), np.where(hit, hi_t, 0.)
        first = np.clip(np.floor(lo_t/step).astype(counts.dtype)+lo_pad, 0, counts)
        last = np.clip(np.floor(hi_t/step).astype(counts.dtype)+hi_pad, first, counts)
        kept = np.where(hit, last-first, 0)
        if drop_last:
            kept = np.maximum(kept-1, 0)
        return first, kept
    return window


def shifted_origin(self, o, vec, dist, free, counts, step):
    first, kept = ORIGINAL(self, o, vec, dist, free, counts, step)
    return np.zeros_like(first), kept     # 원래 번호 대신 0부터 다시 셈(잘못된 원점 재설정)


MUTANTS = {'exact_grid_no_slack': variant(0, 0, 0.), 'drop_last_sample': variant(-2, 3, 1., True),
           'shifted_origin': shifted_origin,
           # 대조군: 한 칸 넓힘·여유 없이 floor(hi)+1까지 담는 좁지만 수학적으로 맞는 구간(실패하지 않아야 정상)
           'control_tight_valid': variant(0, 1, 0.)}
CONTROLS = {'control_tight_valid'}
TESTS = [(n, f) for n, f in vars(t).items() if n.startswith('test_') and 'raster' in n or n.startswith('test_random')
         or n.startswith('test_sub_nano')]
report = {}
for name, mutant in MUTANTS.items():
    li.ObservationGrid._free_sample_window = mutant
    failed = []
    for test_name, fn in TESTS:
        params = inspect.signature(fn).parameters
        cases = ([{'name': k} for k in sorted(t.CASES)] if 'name' in params else
                 [{'seed': s} for s in range(24)] if 'seed' in params else [{}])
        for kwargs in cases:
            try:
                fn(**kwargs)
            except AssertionError:
                failed.append(f'{test_name}{kwargs}')
    report[name] = {'failed_cases': len(failed), 'examples': failed[:5]}
li.ObservationGrid._free_sample_window = ORIGINAL
print(json.dumps(report, indent=1))
ok = all(bool(v['failed_cases']) != (k in CONTROLS) for k, v in report.items())
print('MUTANTS_KILLED_AND_CONTROL_PASSED' if ok else 'PROBE_UNEXPECTED')
sys.exit(0 if ok else 1)
