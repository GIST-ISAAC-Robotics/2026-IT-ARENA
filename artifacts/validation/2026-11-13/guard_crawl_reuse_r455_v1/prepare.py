from pathlib import Path
import hashlib,json,shutil
B=Path(__file__).resolve().parent;R=B.parents[3]
def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    stage=B/'stage';stage.mkdir(exist_ok=False);sources={}
    for pkg in ('arena_autonomy','arena_vehicle_interface'):
        for p in (R/'src'/pkg/pkg).rglob('*.py'):
            relative=p.relative_to(R);dest=stage/relative;dest.parent.mkdir(parents=True,exist_ok=True)
            shutil.copyfile(p,dest);sources[relative.as_posix()]=digest(p)
    tests=('test_runtime_guard_crawl.py','test_guard_curve_trial.py','test_guard_masked_speed.py',
           'test_pass_release_trial.py','test_guard_observation_policy.py')
    for name in tests:
        p=R/'tests'/name;dest=stage/'tests'/name;dest.parent.mkdir(exist_ok=True)
        shutil.copyfile(p,dest);sources['tests/'+name]=digest(p)
    for name in ('__init__.py','guard_crawl.py','released_plan.py'):
        src=R/'src/arena_autonomy/arena_autonomy/race_runtime_policy'/name
        dest=B/'original/race_runtime_policy'/name;dest.parent.mkdir(parents=True,exist_ok=True)
        shutil.copyfile(src,dest)
    shutil.copyfile(R/'tests/test_runtime_guard_crawl.py',B/'original/test_runtime_guard_crawl.py')
    (B/'original_sha256.json').write_text(json.dumps(sources,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(snapshot_files=len(sources),common_modified=False)))
if __name__=='__main__':main()
