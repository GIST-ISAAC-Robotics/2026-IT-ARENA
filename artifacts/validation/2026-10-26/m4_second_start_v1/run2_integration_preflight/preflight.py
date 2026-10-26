from pathlib import Path
files=["tests/test_m4_second_start.py","tests/test_m4_operation_trial.py","tests/test_m4_operation_cli.py","tests/test_source_provenance_r0_plus_b3.py","tests/test_corner_follow.py","tests/test_g1_dense_stop.py","tests/test_physics_step_option.py","tests/test_g1_traffic_eval.py","tests/test_g1_traffic_extensions.py","src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py","tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py","scripts/m4_second_start.py","scripts/m4_operation_trial.py","scripts/m4_operation_eval.py","scripts/validate_g1_traffic.py","config/tests/g1_traffic_v1.json","src/arena_bringup/launch/g1_follow.launch.py"]
for name in files:
 p=Path(name); print(repr(name),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)
for name in ("/opt/ros/jazzy/setup.bash","install/setup.bash"):
 p=Path(name); print(repr(name),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)
