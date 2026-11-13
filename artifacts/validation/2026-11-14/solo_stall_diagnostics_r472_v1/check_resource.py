import datetime,hashlib,json,os
from pathlib import Path
A=Path(__file__).resolve().parent
owned=[]
for p in Path('/proc').iterdir():
 if not p.name.isdigit() or int(p.name)==os.getpid():continue
 try:
  raw=(p/'cmdline').read_bytes().split(b'\0')
  if any(x.endswith((b'/diagnose.py',b'/inspect_components.py')) or x in (b'diagnose.py',b'inspect_components.py') for x in raw):owned.append({'pid':int(p.name),'argv':[x.decode(errors='replace') for x in raw]})
 except (OSError,PermissionError):pass
m=json.loads((A/'source_manifest.json').read_text())['files']
unchanged=all(hashlib.sha256((A/'stage'/p).read_bytes()).hexdigest()==h for p,h in m.items())
r={'at_utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'remaining_owned_work':owned,'source_files':len(m),'source_unchanged':unchanged,'no_ros_gazebo_or_devices_started':True,'diagnostic_exit':int((A/'diagnostic_v1.exit').read_text().strip()),'components_wrapper_exit':1,'components_wrapper_issue':'Windows bash -c quoting lost code variable after completed Python output; original log/blank exit preserved; no calculation rerun','components_output_records':len(json.loads((A/'component_evidence.json').read_text()))}
(A/'resource_return.json').write_text(json.dumps(r,indent=2)+'\n')
print(json.dumps(r));assert not owned and unchanged
