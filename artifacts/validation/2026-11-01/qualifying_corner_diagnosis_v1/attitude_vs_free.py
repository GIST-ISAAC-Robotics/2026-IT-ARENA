"""헛도는 쪽과 미세 자세의 관계를 본다. 접촉력 기록이 없어 간접 근거만 된다."""
import gzip
import json
import statistics
from pathlib import Path

HERE = Path(__file__).resolve().parent
R, TRACK = .0325, .135
out = {}
for name in ('rise_only', 'turn_only', 'combined_failed'):
    with gzip.open(HERE / f'extract_{name}.json.gz', 'rt', encoding='utf-8') as f:
        rows = [r for r in json.load(f)['trajectory'] if r['light'] == 'green' and r.get('left_speed_rad_s') is not None]
    groups = {'none': [], 'free_L': [], 'free_R': []}
    for r in rows:
        vx, w = r['truth_longitudinal_mps'], r['truth_yaw_rate_rad_s']
        if vx < .3:
            continue
        el = r['left_speed_rad_s'] * R - (vx - w * TRACK / 2)
        er = r['right_speed_rad_s'] * R - (vx + w * TRACK / 2)
        key = 'none' if max(abs(el), abs(er)) < .05 else ('free_L' if abs(el) > abs(er) else 'free_R') if max(abs(el), abs(er)) > .2 else None
        if key:
            groups[key].append(r)
    out[name] = {}
    for key, g in groups.items():
        if not g:
            continue
        roll = [r['truth_roll_rad'] * 1e6 for r in g]
        pitch = [r['truth_pitch_rad'] * 1e6 for r in g]
        z = [(r['truth_z_m'] - .0655) * 1e6 for r in g]
        tq = [abs(r['motor_torque_nm']) for r in g]
        out[name][key] = dict(n=len(g), roll_urad_median=round(statistics.median(roll), 1), roll_urad_p90_abs=round(sorted(abs(x) for x in roll)[int(.9 * len(roll))], 1),
                              pitch_urad_median=round(statistics.median(pitch), 1), z_um_median=round(statistics.median(z), 2),
                              z_um_max=round(max(z), 2), abs_torque_median=round(statistics.median(tq), 4))
    print(name, json.dumps(out[name]))
with (HERE / 'attitude_vs_free.json').open('w', encoding='utf-8', newline='\n') as f:
    json.dump(dict(scope='접촉력 직접 측정이 아닌 자세 간접 지표', runs=out), f, ensure_ascii=False, indent=1)
