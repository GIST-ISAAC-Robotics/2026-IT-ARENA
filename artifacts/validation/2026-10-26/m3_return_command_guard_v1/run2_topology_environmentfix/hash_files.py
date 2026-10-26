import hashlib, json
from pathlib import Path
root=Path.cwd()
paths=sorted((root/"src/arena_autonomy/arena_autonomy").glob("*.py"))
tests=["tests/test_scan_topology.py","tests/test_local_scene.py","tests/test_observation_support.py","tests/test_observation_depth_window.py","tests/test_observation_depth_equivalence.py","tests/test_observation_depth_cone.py","tests/test_observation_depth_chunk.py","tests/test_m3_baseline_release.py","tests/test_m3_release_gate_review.py","tests/test_m3_retained_threats.py","tests/test_m3_memory_geometry.py"]
paths += [root/p for p in tests]
data={str(p.relative_to(root)):hashlib.sha256(p.read_bytes()).hexdigest() for p in paths}
print(json.dumps(data,sort_keys=True,indent=2))
