from pathlib import Path
import hashlib,json,sys,zipfile,os
root=Path.cwd(); out=Path(sys.argv[1]); h2=root/"artifacts/validation/2026-10-26/m4_second_start_v1"
run4=h2/"run4_source_chain_build"
# Prior gates must all be successful.
checks={"tests":(run4/"tests_exit_code.txt").read_text().strip(),"build":(run4/"build_exit_code.txt").read_text().strip(),"install":(run4/"install_exit_code.txt").read_text().strip()}
print("run4_exit_codes",checks)
if any(v!="0" for v in checks.values()): raise SystemExit(93)
# Current source/test/config state must match the post-install source manifest.
expected=json.loads((run4/"install_post_hashes.json").read_text())
current={}
for rel in expected:
 p=root/rel
 if not p.is_file(): raise FileNotFoundError(p)
 current[rel]=hashlib.sha256(p.read_bytes()).hexdigest()
diff={k:{"expected":expected[k],"current":current[k]} for k in expected if expected[k]!=current[k]}
print("run4_hash_mismatches",json.dumps(diff,sort_keys=True))
if diff: raise SystemExit(94)
# Runtime source_paths and frozen test files must match run4 freeze, and zip bytes must still match.
import sys as _sys
_sys.path.insert(0,str(root/"scripts"))
import validate_g1_traffic as evaluator
runtime=list(evaluator.source_paths("config/tests/g1_traffic_v1.json",None,None))
old_runtime=json.loads((run4/"runtime_source_paths.json").read_text())
if runtime!=old_runtime: raise SystemExit("runtime source_paths changed")
frozen=json.loads((run4/"frozen_files.json").read_text())
zip_path=run4/"source_freeze.zip"
with zipfile.ZipFile(zip_path) as z:
 names=sorted(z.namelist())
 expected_names=sorted(frozen)
 if names!=expected_names: raise SystemExit("freeze zip membership mismatch")
 bad=[]
 for n in names:
  blob=z.read(n); actual=hashlib.sha256((root/n).read_bytes()).hexdigest(); zipped=hashlib.sha256(blob).hexdigest()
  if actual!=zipped: bad.append(n)
 if bad: raise SystemExit("freeze zip hash mismatch: "+repr(bad))
# Existing ROS/Gazebo trial process check; never terminate matches.
tokens=("gz sim","gzserver","gzclient","ros2 launch","ros_gz_bridge","validate_g1_traffic.py","arena_gazebo")
found=[]
for p in Path("/proc").iterdir():
 if not p.name.isdigit() or int(p.name)==os.getpid(): continue
 try:
  cmd=(p/"cmdline").read_bytes().replace(b"\0",b" ").decode(errors="replace")
  comm=(p/"comm").read_text().strip()
 except (OSError,PermissionError): continue
 if any(t in cmd for t in tokens) or comm in ("gzserver","gzclient"):
  found.append({"pid":int(p.name),"comm":comm,"cmdline":cmd})
print("preexisting_sim_ros_processes",json.dumps(found,sort_keys=True))
(out/"preexisting_processes.json").write_text(json.dumps(found,indent=2)+"\n",encoding="utf-8")
if found: raise SystemExit(95)
# Snapshot runtime source set plus run-specific test/source dependencies.
extra=["tests/test_m4_second_start.py","tests/test_m4_operation_trial.py","tests/test_m4_operation_cli.py","tests/test_source_provenance_r0_plus_b3.py","tests/test_corner_follow.py","tests/test_g1_dense_stop.py","tests/test_physics_step_option.py","tests/test_g1_traffic_eval.py","tests/test_g1_traffic_extensions.py","src/arena_autonomy/arena_autonomy/m3_guarded_return_trial.py","tests/test_m3_guarded_return_trial.py","tests/test_m3_return_guard.py","src/arena_bringup/launch/g1_follow.launch.py","config/tests/g1_traffic_v1.json"]
paths=sorted(set(runtime+extra)); (out/"runtime_source_paths.json").write_text(json.dumps(runtime,indent=2)+"\n",encoding="utf-8")
(out/"frozen_files.json").write_text(json.dumps(paths,indent=2)+"\n",encoding="utf-8")
with zipfile.ZipFile(out/"execution_sources.zip","w",zipfile.ZIP_DEFLATED) as z:
 for n in paths: z.write(root/n,n)
(out/"pre_run_hashes.json").write_text(json.dumps({n:hashlib.sha256((root/n).read_bytes()).hexdigest() for n in paths},indent=2,sort_keys=True)+"\n",encoding="utf-8")
print("freeze_count",len(paths),"source_paths_count",len(runtime),"zip",str(out/"execution_sources.zip"))
