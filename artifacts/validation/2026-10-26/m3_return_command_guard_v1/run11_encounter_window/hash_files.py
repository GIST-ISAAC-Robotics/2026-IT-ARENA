import hashlib,json
from pathlib import Path
root=Path.cwd(); base=root/"artifacts/validation/2026-10-25/m3_execution_review_v1"
source=[root/"artifacts/validation/2026-10-26/m3_return_command_guard_v1/encounter_window_audit.py",root/"scripts/g1_traffic_eval.py"]
inputs=[root/"src/arena_gazebo/worlds/it_arena_official/centerline.csv",root/"src/arena_gazebo/worlds/it_arena_official/scene.json"]
for run in ("run17_observed_release_closed_loop","run21_right_encounter"):
 inputs += [base/run/"run"/f for f in ("report.json","ego_poses.jsonl","additional_actors.json","follow_decisions.jsonl")]
paths=source+inputs
print(json.dumps({str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths},sort_keys=True,indent=2))
