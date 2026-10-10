"""보존 등속 실패 구간에서 LocalFollow.command 계산 시간을 이전/현재 소스로 비교(PC WSL, 분석 전용)."""
import json, sys, time
from pathlib import Path
import numpy as np
ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))
import replay_g1b_follow as rp  # noqa: E402

source = Path(sys.argv[1]).resolve()
modules = rp.load_modules(source)
original = modules.LocalFollow.command
samples = []


def timed(self, *args, **kwargs):
    began = time.perf_counter()
    result = original(self, *args, **kwargs)
    samples.append((time.perf_counter()-began)*1e3)
    return result


modules.LocalFollow.command = timed
run = ROOT/'artifacts/validation/2026-10-07/g1b_pilot_v3_slow_s0'
for _ in range(3):
    rp.replay(run, modules, 3.5, 16.0)
    rp.replay(run, modules, 8.5, 16.0, cold_start=[9.0])
values = np.asarray(samples)
print(json.dumps({'source': str(source), 'calls': len(values), 'mean_ms': values.mean(), 'p50_ms': np.percentile(values, 50),
                  'p99_ms': np.percentile(values, 99), 'max_ms': values.max(), 'over_20ms': int(np.sum(values > 20))}))
