"""A·B 두 보충 판정을 나란히 기록한다. 판정을 새로 만들지 않으며 각 run 결과를 바꾸지 않는다. 사용: compare.py"""
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
rows = {}
for key, folder in (('A', 'A_no_injection'), ('B', 'B_c1_injection')):
    f = HERE/folder
    a = json.loads((f/'assessment_run1.json').read_bytes())
    rows[key] = dict(harness_exit_line=(f/'run1.exit').read_text().strip(), raw_pass=a['raw_pass'],
                     supplementary_passed=a['passed'], failed_gates=[k for k, v in a['gates'].items() if not v],
                     scan_age_s=a['scan_age_s'], injection=a['injection'], min_gap=a['min_gap'],
                     progress_m=a['course'].get('progress_m'), peak_mps=a['peak_mps'])
shift = {k: rows['B']['scan_age_s'][k]-rows['A']['scan_age_s'][k] for k in ('min', 'p50', 'p95', 'max')
         if rows['A']['scan_age_s'][k] is not None and rows['B']['scan_age_s'][k] is not None}
out = dict(runs=rows, scan_age_shift_B_minus_A_s=shift,
           expectation='about +0.03..0.07 s; recorded only, not a pre-registered gate')
(HERE/'pair_comparison.json').write_text(json.dumps(out, ensure_ascii=False, indent=2)+'\n', encoding='utf8')
print(json.dumps(out, ensure_ascii=False))
