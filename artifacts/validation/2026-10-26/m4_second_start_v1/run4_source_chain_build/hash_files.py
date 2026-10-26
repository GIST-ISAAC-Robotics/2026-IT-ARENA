import hashlib,json
from pathlib import Path
root=Path.cwd(); files=["scripts/m4_second_start.py","scripts/m4_operation_trial.py","scripts/m4_operation_eval.py","scripts/validate_g1_traffic.py","src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py","tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py","config/tests/g1_traffic_v1.json","src/arena_bringup/launch/g1_follow.launch.py","tests/test_corner_follow.py","tests/test_source_provenance_r0_plus_b3.py"]
files += [str(p.relative_to(root)) for p in sorted((root/"src/arena_autonomy/arena_autonomy").glob("*.py"))]
print(json.dumps({p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in sorted(set(files))},sort_keys=True,indent=2))
