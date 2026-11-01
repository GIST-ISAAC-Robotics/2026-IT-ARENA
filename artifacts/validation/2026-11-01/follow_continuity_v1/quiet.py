"""출발 후 0.05 m/s 미만 구간을 시간대로 나눈다(사후 평가)."""
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from slip_events import body_speed

for name in sys.argv[1:]:
    run = Path(name)
    r = json.loads((run / 'report.json').read_bytes())
    g = r['case_window']['green_sim_s']
    d = r['case_window']['duration_s']
    t, v, _, _ = body_speed(run, g)
    k = t <= d
    t, v = t[k], v[k]
    first = t[np.flatnonzero(v >= .05)[0]]
    print(name, 'first motion', round(float(first), 2))
    i = 0
    while i < len(t):
        if t[i] >= first and v[i] < .05:
            j = i
            while j + 1 < len(t) and v[j + 1] < .05:
                j += 1
            print(f'   {t[i]:6.2f}-{t[j]:6.2f} {t[j]-t[i]+.02:.2f}s')
            i = j + 1
        else:
            i += 1
