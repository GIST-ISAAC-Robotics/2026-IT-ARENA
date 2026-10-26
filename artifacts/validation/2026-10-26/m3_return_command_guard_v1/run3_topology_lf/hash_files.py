import hashlib,json
from pathlib import Path
root=Path.cwd()
files=['src/arena_autonomy/arena_autonomy/scan_topology.py', 'src/arena_autonomy/arena_autonomy/observation_support.py', 'src/arena_autonomy/arena_autonomy/m3_execution_trial.py', 'tests/test_scan_topology.py', 'tests/test_local_scene.py', 'tests/test_observation_support.py', 'tests/test_observation_depth_window.py', 'tests/test_observation_depth_equivalence.py', 'tests/test_observation_depth_cone.py', 'tests/test_observation_depth_chunk.py', 'tests/test_m3_baseline_release.py', 'tests/test_m3_release_gate_review.py', 'tests/test_m3_retained_threats.py', 'tests/test_m3_memory_geometry.py', 'tests/test_m3_return_guard.py']
result={p:hashlib.sha256((root/p).read_bytes()).hexdigest() for p in files}
print(json.dumps(result,sort_keys=True,indent=2))
