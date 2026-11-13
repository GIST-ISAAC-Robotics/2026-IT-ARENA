from pathlib import Path
import hashlib,json,zipfile
bundle=Path(__file__).resolve().parent; repo=bundle.parents[3]
files={}
for path in (bundle/'test_overlay'/'arena_autonomy').rglob('*.py'):
    files['arena_autonomy/'+path.relative_to(bundle/'test_overlay'/'arena_autonomy').as_posix()]=path
for path in (repo/'src/arena_vehicle_interface/arena_vehicle_interface').rglob('*.py'):
    files['arena_vehicle_interface/'+path.relative_to(repo/'src/arena_vehicle_interface/arena_vehicle_interface').as_posix()]=path
files['ros_panel_probe.py']=bundle/'ros_panel_probe.py'
manifest={name:hashlib.sha256(path.read_bytes()).hexdigest() for name,path in files.items()}
with zipfile.ZipFile(bundle/'ros_sources_v2.zip','x',zipfile.ZIP_DEFLATED) as z:
    for name,path in files.items():z.write(path,name)
    z.writestr('manifest.json',json.dumps(manifest,indent=2))
print(len(files))

