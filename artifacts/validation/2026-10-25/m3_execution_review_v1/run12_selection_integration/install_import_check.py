import hashlib, importlib, json, pathlib, sys
root=pathlib.Path.cwd()
checks={
 'arena_autonomy.m3_execution_trial': root/'src/arena_autonomy/arena_autonomy/m3_execution_trial.py',
 'arena_autonomy.follow_trace': root/'src/arena_autonomy/arena_autonomy/follow_trace.py',
 'arena_autonomy.local_pursuit': root/'src/arena_autonomy/arena_autonomy/local_pursuit.py',
}
results={}
for name,source in checks.items():
 mod=importlib.import_module(name); installed=pathlib.Path(mod.__file__).resolve()
 a=hashlib.sha256(source.read_bytes()).hexdigest(); b=hashlib.sha256(installed.read_bytes()).hexdigest()
 results[name]={'installed_path':str(installed),'source_path':str(source.resolve()),'source_sha256':a,'installed_sha256':b,'match':a==b}
s=root/'src/arena_bringup/launch/g1_follow.launch.py'; i=root/'install/arena_bringup/share/arena_bringup/launch/g1_follow.launch.py'
a=hashlib.sha256(s.read_bytes()).hexdigest(); b=hashlib.sha256(i.read_bytes()).hexdigest()
results['g1_follow.launch.py']={'installed_path':str(i),'source_path':str(s.resolve()),'source_sha256':a,'installed_sha256':b,'match':a==b}
pathlib.Path(sys.argv[1]).write_text(json.dumps(results,indent=2)+'\n'); print(json.dumps(results,indent=2))
