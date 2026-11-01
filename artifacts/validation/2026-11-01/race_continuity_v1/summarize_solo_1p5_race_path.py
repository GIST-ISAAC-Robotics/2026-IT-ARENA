import json, math, pathlib, hashlib
base=pathlib.Path('artifacts/validation/2026-11-01/race_continuity_v1')
run=base/'solo_1p5_race_path'
manifest=json.loads((base/'solo_1p5_race_path_pre_manifest.json').read_text())
report=json.loads((run/'report.json').read_text())
metrics=json.loads((base/'solo_1p5_race_path_metrics.json').read_text())
rows=[json.loads(line) for line in (run/'trajectory.jsonl').open()]
speeds=[]
for i,row in enumerate(rows,1):
 d=row.get('dynamics',{})
 if d.get('truth_longitudinal_mps') is not None and d.get('truth_lateral_mps') is not None:
  lon=float(d['truth_longitudinal_mps']); lat=float(d['truth_lateral_mps'])
  speeds.append({'trajectory_line':i,'sim_time_s':d.get('sim_time_s'),'truth_longitudinal_mps':lon,'truth_lateral_mps':lat,'truth_ground_speed_mps':math.hypot(lon,lat)})
peak=max(speeds,key=lambda x:x['truth_ground_speed_mps'])
fd=json.loads((run/'follow_decisions.jsonl').open().readline())['data']['follow_config']
hash_diffs=[]
for rel,expected in manifest['source_sha256'].items():
 actual=hashlib.sha256(pathlib.Path(rel).read_bytes()).hexdigest()
 if actual!=expected: hash_diffs.append(rel)
result={'run':str(run),'profile':report.get('speed_profile'),'passed':report.get('passed'),'progress_m':report.get('progress_m'),'completed_laps':report.get('completed_laps'),'target_progress_m':report.get('target_progress_m'),'lap_length_m':report.get('lap_length_m'),'collision_samples':report.get('collision_samples'),'peak_truth_ground_speed':peak,'truth_velocity_samples':len(speeds),'truth_samples_at_or_above_1p5_mps':sum(x['truth_ground_speed_mps']>=1.5 for x in speeds),'metrics_peak_ground_mps':metrics.get('peak_ground_mps'),'best_one_second_mean_mps':metrics.get('best_one_second_mean_mps'),'stop_stable':metrics.get('stop_stable'),'stop_ack':report.get('normal_stop',report.get('failure_cleanup_stop')),'shutdown_clean':report.get('shutdown_clean'),'gazebo_stop_service':report.get('gazebo_stop_service'),'process_exit_codes':report.get('process_exit_codes'),'process_errors':report.get('process_errors'),'source_unchanged':report.get('source_unchanged'),'manifest_input_hash_count':len(manifest['source_sha256']),'source_hash_differences':hash_diffs,'ratio':manifest.get('drive_gear_ratio'),'follow_config':fd,'report_sha256':hashlib.sha256((run/'report.json').read_bytes()).hexdigest(),'trajectory_sha256':hashlib.sha256((run/'trajectory.jsonl').read_bytes()).hexdigest()}
out=base/'solo_1p5_race_path_truth_metrics.json'
if out.exists(): raise SystemExit(f'refusing to overwrite {out}')
out.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(result,ensure_ascii=False,indent=2))