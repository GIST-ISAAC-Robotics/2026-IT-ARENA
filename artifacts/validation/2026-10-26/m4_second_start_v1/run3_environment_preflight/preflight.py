import hashlib,json,sys,zipfile
from pathlib import Path
repo=Path.cwd(); out=Path(sys.argv[1]); out.mkdir(parents=True,exist_ok=True)
required=["tests/test_m4_second_start.py","tests/test_m4_operation_trial.py","tests/test_m4_operation_cli.py","tests/test_source_provenance_r0_plus_b3.py","tests/test_corner_follow.py","tests/test_g1_dense_stop.py","tests/test_physics_step_option.py","tests/test_g1_traffic_eval.py","tests/test_g1_traffic_extensions.py","src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py","tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py","scripts/m4_second_start.py","scripts/m4_operation_trial.py","scripts/m4_operation_eval.py","scripts/validate_g1_traffic.py","config/tests/g1_traffic_v1.json","src/arena_bringup/launch/g1_follow.launch.py"]
for name in required:
 p=repo/name; print(repr(name),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): return_code=92; raise SystemExit(return_code)
import xacro,rclpy,launch,arena_autonomy
mods={"xacro":getattr(xacro,"__file__",None),"rclpy":getattr(rclpy,"__file__",None),"launch":getattr(launch,"__file__",None),"arena_autonomy.__path__":list(arena_autonomy.__path__)}
(out/"imports.json").write_text(json.dumps(mods,indent=2)+"\n",encoding="utf-8")
print("IMPORTS",json.dumps(mods,sort_keys=True))
import validate_g1_traffic as v
runtime=v.source_paths("config/tests/g1_traffic_v1.json",None,None)
paths=sorted({Path(x).resolve() for x in runtime})
manifest=[str(p.relative_to(repo)) for p in paths]
(out/"runtime_source_paths.json").write_text(json.dumps(manifest,indent=2)+"\n",encoding="utf-8")
extra=[repo/p for p in ("src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py","tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py",*required[:9])]
all_paths=sorted(set(paths+extra))
with zipfile.ZipFile(out/"source_test_snapshot.zip","w",compression=zipfile.ZIP_DEFLATED) as z:
 for p in all_paths:
  if not p.is_file(): raise FileNotFoundError(p)
  z.write(p,p.relative_to(repo).as_posix())
(out/"frozen_files.json").write_text(json.dumps([str(p.relative_to(repo)) for p in all_paths],indent=2)+"\n",encoding="utf-8")
