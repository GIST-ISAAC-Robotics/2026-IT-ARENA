import hashlib, importlib, pathlib, sys
root=pathlib.Path.cwd()
checks={
 'arena_autonomy.m3_execution_trial': root/'src/arena_autonomy/arena_autonomy/m3_execution_trial.py',
 'arena_autonomy.follow_trace': root/'src/arena_autonomy/arena_autonomy/follow_trace.py',
 'arena_autonomy.local_pursuit': root/'src/arena_autonomy/arena_autonomy/local_pursuit.py',
}
results={}
for name,source in checks.items():
 mod=importlib.import_module(name)
 installed=pathlib.Path(mod.__file__).resolve()
 src_hash=hashlib.sha256(source.read_bytes()).hexdigest()
 installed_hash=hashlib.sha256(installed.read_bytes()).hexdigest()
 results[name]={'installed_path':str(installed),'source_path':str(source.resolve()),'source_sha256':src_hash,'installed_sha256':installed_hash,'match':src_hash==installed_hash}
launch_source=root/'src/arena_bringup/launch/g1_follow.launch.py'
launch_installed=root/'install/arena_bringup/share/arena_bringup/launch/g1_follow.launch.py'
results['g1_follow.launch.py']={'installed_path':str(launch_installed),'source_path':str(launch_source.resolve()),'source_sha256':hashlib.sha256(launch_source.read_bytes()).hexdigest(),'installed_sha256':hashlib.sha256(launch_installed.read_bytes()).hexdigest(),'match':hashlib.sha256(launch_source.read_bytes()).digest()==hashlib.sha256(launch_installed.read_bytes()).digest()}
pathlib.Path(sys.argv[1]).write_text(__import__('json').dumps(results,indent=2)+'\n')
print(__import__('json').dumps(results,indent=2))
