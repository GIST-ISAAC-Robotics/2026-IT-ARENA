from pathlib import Path
files=["tests/test_scan_topology.py","tests/test_local_scene.py","tests/test_observation_support.py","tests/test_observation_depth_window.py","tests/test_observation_depth_equivalence.py","tests/test_observation_depth_cone.py","tests/test_observation_depth_chunk.py","tests/test_m3_baseline_release.py","tests/test_m3_release_gate_review.py","tests/test_m3_retained_threats.py","tests/test_m3_memory_geometry.py","tests/test_m3_return_guard.py"]
for name in files:
 p=Path(name)
 print(repr(name), "exists=",p.is_file(), "resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)
