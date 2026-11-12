"""Audit the Fusion motion sweep and export a simulator handoff table."""
import csv
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
data = json.loads((HERE / 'fusion_sweep.json').read_text(encoding='utf-8'))
assert data['error'] is None
neutral = data['neutral']
rows = [*reversed(data['branches']['negative']), neutral, *data['branches']['positive']]
assert len(rows) == 96
assert all(a['target_deg'] < b['target_deg'] for a, b in zip(rows, rows[1:]))
assert all(abs(r['target_deg'] - r['servo_deg']) < 0.02 for r in rows)
assert max(abs(r['left_heading_deg'] + r['left_joint_deg']) for r in rows) < 1e-6
assert max(abs(r['right_heading_deg'] + r['right_joint_deg']) for r in rows) < 1e-6
assert all(a['left_heading_deg'] > b['left_heading_deg'] and a['right_heading_deg'] > b['right_heading_deg'] for a, b in zip(rows, rows[1:]))
assert max(abs(((data['restoration'][k] + 180) % 360) - 180) for k in ('servo_deg', 'left_joint_deg', 'right_joint_deg')) < 0.05
with (HERE / 'steering_table.csv').open('w', encoding='utf-8', newline='') as f:
    w = csv.writer(f)
    w.writerow(['fusion_servo_deg', 'left_wheel_heading_deg_left_positive', 'right_wheel_heading_deg_left_positive', 'left_wheel_heading_rad_left_positive', 'right_wheel_heading_rad_left_positive'])
    for r in rows:
        l, rr = r['left_heading_deg'], r['right_heading_deg']
        w.writerow([f"{r['target_deg']:.1f}", f'{l:.8f}', f'{rr:.8f}', f'{math.radians(l):.10f}', f'{math.radians(rr):.10f}'])
print('PASS:', len(rows), 'rows, two continuous monotone branches, axle-vector agreement, and neutral restoration')
