"""기존 종료 STOP의 실제 변위와 감속식 가정을 분리한다. 주행 허가값을 바꾸지 않는다."""
import hashlib
import json
import math
from pathlib import Path

BASE = Path(__file__).resolve().parent
out = BASE / 'stop_comparison.json'
if out.exists():
    raise FileExistsError(out)
sources, rows = {}, []
for name in ('run1', 'run2', 'course1'):
    folder = BASE / name
    values = {}
    for filename in ('report.json', 'stop_trace.json', 'trace.json'):
        path = folder / filename
        sources[str(path.relative_to(BASE))] = hashlib.sha256(path.read_bytes()).hexdigest()
        values[filename] = json.loads(path.read_text(encoding='utf-8'))
    report, stop, trace = (values[n] for n in ('report.json', 'stop_trace.json', 'trace.json'))
    if not stop or any(not math.isfinite(r[k]) for r in stop for k in ('time', 'x', 'y', 'ground_speed_mps')):
        raise ValueError('invalid stop trace')
    times = [r['time'] for r in stop]
    if any(b <= a for a, b in zip(times, times[1:])):
        raise ValueError('stop trace is not ordered')
    speed = stop[0]['ground_speed_mps']
    motion = trace[-1]['autonomy'].get('motion', {})
    age = motion.get('oldest_observation_age_s')
    distance = math.hypot(stop[-1]['x'] - stop[0]['x'], stop[-1]['y'] - stop[0]['y'])
    path_length = sum(math.hypot(b['x'] - a['x'], b['y'] - a['y']) for a, b in zip(stop, stop[1:]))
    rows.append(dict(run=name, stop_initial_ground_speed_mps=speed, displacement_m=distance,
        report_displacement_m=report['stop_displacement_m'], path_length_m=path_length,
        stable_wait_s=report['stop_wait_sim_s'], location_xy_m=[stop[0]['x'], stop[0]['y']],
        max_sample_gap_s=max(b - a for a, b in zip(times, times[1:])),
        no_delay_equivalent_deceleration_mps2=speed**2 / (2 * path_length),
        ideal_braking_at_2mps2_m=speed**2 / 4,
        formula_clearance_at_age_0_m=.06 + .1 * speed + speed**2 / 4,
        formula_clearance_at_age_point1_m=.06 + .2 * speed + speed**2 / 4,
        last_pre_stop_controller_scan_age_s=age,
        last_pre_stop_controller_reference_s=motion.get('reference_stamp_s'),
        stop_first_pose_s=times[0],
        formula_clearance_at_last_controller_age_m=(.06 + (age + .1) * speed + speed**2 / 4) if age is not None else None))
result = dict(rows=rows, sources_sha256=sources,
    scope='three prior ending STOP samples only; no controlled same-position brake comparison or calibrated deceleration',
    formula='current braking_speed inverse: clearance = 0.06 + v*(scan_age+0.10) + v*v/(2*2.0)',
    limitations=['STOP start evidence is a pose sample, not hardware response timestamp',
        'last controller scan age is earlier than the STOP request and is not the exact guard age',
        'equivalent deceleration includes delay and variable braking; not a fitted actuator parameter',
        'SW STOP output and ordinary speed limiting have different histories; not interchangeable proof'])
result['inputs_unchanged'] = all(hashlib.sha256((BASE / p).read_bytes()).hexdigest() == h for p, h in sources.items())
out.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps(rows, ensure_ascii=False))
