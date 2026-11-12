"""Independent planar linkage solve and a non-destructive geometry sensitivity check."""
import json
import math
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
inventory = json.loads((HERE / 'fusion_inventory.json').read_text(encoding='utf-8'))
sweep = json.loads((HERE / 'fusion_sweep.json').read_text(encoding='utf-8'))
joints = inventory['occurrences'][0]['joints']

def point(index):
    matrix = joints[index]['transform']
    return np.array([matrix[3], matrix[11]]) * 10.0

kl, kr, pl, pr, servo, horn, drag = (point(i) for i in (0, 3, 2, 5, 6, 7, 8))

def rot(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s], [s, c]])

def solve(h, d, target, initial):
    length = np.linalg.norm(d - h)
    def residual(v):
        left, right, bar = v
        lp = kl + rot(left) @ (pl - kl)
        rp = kr + rot(right) @ (pr - kr)
        dp = lp + rot(bar) @ (d - pl)
        hp = servo + rot(math.radians(target)) @ (h - servo)
        return np.array([*(lp + rot(bar) @ (pr - pl) - rp), np.linalg.norm(dp - hp) - length])
    v = np.array(initial, dtype=float)
    for _ in range(50):
        f = residual(v)
        if np.linalg.norm(f) < 1e-9:
            return v
        jac = np.column_stack([(residual(v + np.eye(3)[i] * 1e-6) - f) / 1e-6 for i in range(3)])
        v -= np.linalg.solve(jac, f)
    raise RuntimeError('Planar solve did not converge at ' + str(target))

def run(h, d):
    result = {}
    errors = []
    for branch in sweep['branches'].values():
        state = np.zeros(3)
        for row in branch:
            target = row['target_deg']
            state = solve(h, d, target, state)
            result[target] = [float(x) for x in np.degrees(state[:2])]
            if np.array_equal(h, horn) and np.array_equal(d, drag):
                errors.extend([abs(result[target][0] - row['left_joint_deg']), abs(result[target][1] - row['right_joint_deg'])])
    return result, max(errors) if errors else None

actual, error = run(horn, drag)
aligned_horn = np.array([servo[0], horn[1]])
aligned, _ = run(aligned_horn, drag)
assert error < 1e-5, error
summary = {
    'actual_minus_plus_10_joint_deg': {'minus': actual[-10.0], 'plus': actual[10.0]},
    'horn_aligned_counterfactual_minus_plus_10_joint_deg': {'minus': aligned[-10.0], 'plus': aligned[10.0]},
    'max_fusion_planar_disagreement_deg': error,
    'horn_neutral_offset_mm': (horn - servo).tolist(),
    'horn_offset_from_longitudinal_deg': math.degrees(math.atan2(horn[0] - servo[0], horn[1] - servo[1])),
    'scope': 'planar ideal pin linkage; sensitivity is hypothetical and changes CAD geometry only in calculation',
}
(HERE / 'planar_check.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
print('PASS: planar linkage matches Fusion within', error, 'degrees across', len(actual), 'samples')
print(json.dumps(summary, indent=2))
