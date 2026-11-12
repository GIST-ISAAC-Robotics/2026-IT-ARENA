from pathlib import Path
import hashlib,json,os,signal,subprocess,sys,time,zipfile

source=Path('/home/q/it-arena-r358-20261113.zip')
assert hashlib.sha256(source.read_bytes()).hexdigest()==sys.argv[1]
root=Path('/home/q/it-arena-r358-20261113');root.mkdir(exist_ok=False)
with zipfile.ZipFile(source) as z:
    for name in z.namelist():
        assert not Path(name).is_absolute() and '..' not in Path(name).parts
    z.extractall(root)
a=root/'artifacts/validation/2026-11-13/shared_ray_timed_r358_v1'
manifest=json.loads((a/'source_manifest.json').read_bytes())
def changed():
    return [n for n,h in manifest.items() if not (root/n).is_file() or hashlib.sha256((root/n).read_bytes()).hexdigest()!=h]
assert not changed()
started=time.time();runs=[];error=None
env=dict(os.environ,PYTHONPATH=os.pathsep.join(str(root/p) for p in ('src/arena_autonomy','src/arena_vehicle_interface','scripts')))
try:
    with (a/'native_build.log').open('x') as log:
        subprocess.run(['/usr/bin/python3','setup.py','build_ext','--inplace'],cwd=root/'src/arena_autonomy',stdout=log,stderr=subprocess.STDOUT,check=True,timeout=120)
    for i,mode in enumerate(('split','shared','shared','split'),1):
        out=a/f'jetson_{i}_{mode}'
        cmd=['/usr/bin/python3','-B',str(a/'timed_replay.py'),'--classification',mode,
            '--policy','deadline','--mode','delay20','--submission','latest','--duration','31',
            '--plan-ms','0','--current-ms','0','--trace',
            'artifacts/validation/2026-11-13/final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl','--out',str(out)]
        with (a/f'jetson_{i}_{mode}.log').open('x') as log:
            child=subprocess.Popen(cmd,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:code=child.wait(timeout=100)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid,signal.SIGTERM)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=5)
                raise
        runs.append(dict(index=i,classification=mode,returncode=code))
        print(json.dumps(runs[-1]),flush=True)
        if code:raise RuntimeError(f'bounded timing run {i} failed: {code}')
except Exception as exc:
    error=repr(exc)
finally:
    record=dict(runs=runs,error=error,changed_sources=changed(),wall_s=time.time()-started,
        devices_opened=False,firmware_changed=False,power_settings_changed=False,source_bundle_sha256=sys.argv[1])
    (a/'execution.json').write_text(json.dumps(record,indent=2)+'\n')
    with zipfile.ZipFile('/home/q/r358-results.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in [a/'execution.json',a/'native_build.log',*a.glob('jetson_*.log'),*a.glob('jetson_*/*.json'),*a.glob('jetson_*/*.jsonl')]:
            if p.is_file():z.write(p,p.relative_to(a).as_posix())
    print(json.dumps(record),flush=True)
if error:raise SystemExit(1)
assert not record['changed_sources']
