from pathlib import Path
base=Path("artifacts/validation/2026-10-25/m3_execution_review_v1")
files=["scripts/g1_traffic_eval.py","artifacts/validation/2026-10-26/m3_return_command_guard_v1/encounter_window_audit.py","src/arena_gazebo/worlds/it_arena_official/centerline.csv","src/arena_gazebo/worlds/it_arena_official/scene.json"]
for run in ("run17_observed_release_closed_loop","run21_right_encounter"):
 for f in ("report.json","ego_poses.jsonl","additional_actors.json","follow_decisions.jsonl"):
  files.append(str(base/run/"run"/f))
for name in files:
 p=Path(name)
 print(repr(name),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)
