import json, math, pathlib, hashlib
base = pathlib.Path('artifacts/validation/2026-11-01/race_continuity_v1')
run = base / 'solo_2p0_corrected'
trajectory = run / 'trajectory.jsonl'
out = base / 'solo_2p0_corrected_truth_speed_correction.json'
if out.exists():
    raise SystemExit(f'refusing to overwrite {out}')
samples = []
for line_no, line in enumerate(trajectory.open(encoding='utf-8'), 1):
    row = json.loads(line)
    dynamics = row.get('dynamics', {})
    longitudinal = dynamics.get('truth_longitudinal_mps')
    lateral = dynamics.get('truth_lateral_mps')
    if longitudinal is None or lateral is None:
        continue
    longitudinal = float(longitudinal)
    lateral = float(lateral)
    samples.append({
        'trajectory_line': line_no,
        'sim_time_s': dynamics.get('sim_time_s'),
        'truth_longitudinal_mps': longitudinal,
        'truth_lateral_mps': lateral,
        'truth_ground_speed_mps': math.hypot(longitudinal, lateral),
    })
if not samples:
    raise SystemExit('no truth velocity samples found')
peak = max(samples, key=lambda sample: sample['truth_ground_speed_mps'])
threshold_count = sum(sample['truth_ground_speed_mps'] >= 2.0 for sample in samples)
metrics = json.loads((base / 'solo_2p0_corrected_metrics.log').read_text(encoding='utf-8').split('ACTUAL_EXIT_CODE=')[0])
result = {
    'purpose': 'Correction to solo_2p0_corrected_inspection.json truth speed extraction; original inspection file is preserved.',
    'run': str(run),
    'source_file': str(trajectory),
    'source_file_sha256': hashlib.sha256(trajectory.read_bytes()).hexdigest(),
    'correct_fields': ['dynamics.truth_longitudinal_mps', 'dynamics.truth_lateral_mps'],
    'ground_speed_definition': 'hypot(truth_longitudinal_mps, truth_lateral_mps)',
    'truth_velocity_sample_count': len(samples),
    'peak_truth_ground_speed': peak,
    'samples_at_or_above_2_mps': threshold_count,
    'metrics_cross_check': {
        'metrics_peak_ground_mps': metrics.get('peak_ground_mps'),
        'matches_peak_within_1e-9_mps': abs(metrics.get('peak_ground_mps', float('nan')) - peak['truth_ground_speed_mps']) <= 1e-9,
        'run_progress_m': metrics.get('progress_m'),
        'completed_laps': metrics.get('completed_laps'),
        'collision_samples': metrics.get('collision_samples'),
    },
}
out.write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print(out)