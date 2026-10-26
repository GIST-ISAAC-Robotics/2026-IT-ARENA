from pathlib import Path
import json,zipfile,hashlib,sys
repo=Path.cwd(); out=Path(sys.argv[1]); required=["tests/test_corner_follow.py","tests/test_source_provenance_r0_plus_b3.py","scripts/validate_g1_traffic.py","config/tests/g1_traffic_v1.json","src/arena_bringup/launch/g1_follow.launch.py","install/arena_bringup/share/arena_bringup/launch/g1_follow.launch.py","src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py","tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py"]
for n in required:
 p=repo/n; print(repr(n),"exists=",p.is_file(),"resolved=",repr(str(p.resolve())))
 if not p.is_file(): raise SystemExit(92)
import numpy,pytest,xacro,rclpy,launch,arena_autonomy,validate_g1_traffic as v
mods={"numpy":numpy.__version__,"pytest":pytest.__version__,"xacro":xacro.__file__,"rclpy":rclpy.__file__,"launch":launch.__file__,"arena_autonomy.__path__":list(arena_autonomy.__path__)}
(out/"imports.json").write_text(json.dumps(mods,indent=2)+"\n",encoding="utf-8"); print("IMPORTS",json.dumps(mods,sort_keys=True))
runtime=[(repo/p).resolve() for p in v.source_paths("config/tests/g1_traffic_v1.json",None,None)]
extras=[(repo/p).resolve() for p in ("src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py","tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py","tests/test_corner_follow.py","tests/test_source_provenance_r0_plus_b3.py")]
paths=sorted(set(runtime+extras)); names=[p.relative_to(repo).as_posix() for p in paths]
(out/"runtime_source_paths.json").write_text(json.dumps([p.relative_to(repo).as_posix() for p in runtime],indent=2)+"\n",encoding="utf-8")
(out/"frozen_files.json").write_text(json.dumps(names,indent=2)+"\n",encoding="utf-8")
with zipfile.ZipFile(out/"source_freeze.zip","w",zipfile.ZIP_DEFLATED) as z:
 for p in paths: z.write(p,p.relative_to(repo).as_posix())
