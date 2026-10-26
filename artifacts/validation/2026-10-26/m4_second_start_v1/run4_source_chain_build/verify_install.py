import hashlib,json,importlib
from pathlib import Path
import arena_autonomy
root=Path.cwd(); modules=("observation_support","scan_topology","m3_execution_trial","local_pursuit","follow_trace")
report={"arena_autonomy_path":list(arena_autonomy.__path__),"modules":{},"launch":{}}
failed=[]
for name in modules:
 mod=importlib.import_module("arena_autonomy."+name)
 actual=Path(mod.__file__).resolve(); source=root/"src/arena_autonomy/arena_autonomy"/(name+".py")
 ah=hashlib.sha256(actual.read_bytes()).hexdigest(); sh=hashlib.sha256(source.read_bytes()).hexdigest()
 report["modules"][name]={"installed_file":str(actual),"installed_sha256":ah,"source_file":str(source),"source_sha256":sh,"match":ah==sh}
 if ah!=sh: failed.append(name)
source=root/"src/arena_bringup/launch/g1_follow.launch.py"
installed=root/"install/arena_bringup/share/arena_bringup/launch/g1_follow.launch.py"
sh=hashlib.sha256(source.read_bytes()).hexdigest(); ih=hashlib.sha256(installed.read_bytes()).hexdigest()
report["launch"]={"source_file":str(source),"installed_file":str(installed),"source_sha256":sh,"installed_sha256":ih,"match":sh==ih}
if sh!=ih: failed.append("g1_follow.launch.py")
report["helper_modules_discovered"]={name:importlib.util.find_spec("arena_autonomy."+name) is not None for name in ("m3_guarded_return_trial","scan_topology")}
report["success"]=not failed; report["mismatches"]=failed
out=Path(__import__("sys").argv[1]); (out/"install_verify.json").write_text(json.dumps(report,indent=2)+"\n",encoding="utf-8")
print(json.dumps(report,sort_keys=True))
raise SystemExit(0 if not failed else 1)
