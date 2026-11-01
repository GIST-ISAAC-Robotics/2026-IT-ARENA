import json, pathlib, math
root=pathlib.Path('artifacts/validation/2026-11-01/race_continuity_v1')
run=root/'solo_2p0_corrected'
report=json.loads((run/'report.json').read_text())
manifest=json.loads((root/'solo_2p0_corrected_pre_manifest.json').read_text())
rows=[]
for line in (run/'trajectory.jsonl').open():
 try: rows.append(json.loads(line))
 except Exception: pass
truth=[]
for r in rows:
 for k in ('truth_longitudinal_mps','truth_lateral_mps'):
  if k not in r: continue
 lon=float(r.get('truth_longitudinal_mps',0)); lat=float(r.get('truth_lateral_mps',0))
 truth.append((math.hypot(lon,lat),lon,lat,r.get('sim_time_s',r.get('stamp_s'))))
fd=run/'follow_decisions.jsonl'
first=json.loads(fd.open().readline())
print(json.dumps({'report':{k:report.get(k) for k in ('passed','progress_m','completed_laps','lap_length_m','collision_samples','target_progress_m','stop','shutdown_clean','error')},'trajectory_rows':len(rows),'truth_peak':max(truth) if truth else None,'truth_samples_over_2':sum(1 for x in truth if x[0]>=2.0),'follow_header':first,'source_hash_match_count':sum(1 for x in manifest.get('inputs',[]) if x.get('sha256')==x.get('current_sha256')),'manifest_input_count':len(manifest.get('inputs',[]))},ensure_ascii=False,indent=2))