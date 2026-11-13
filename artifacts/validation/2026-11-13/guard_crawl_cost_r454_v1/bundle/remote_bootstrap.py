"""r454의 단일 격리 batch. r447 native 사본 재사용, ROS/장치/설정 변경 없음."""
from pathlib import Path
import argparse,hashlib,json,os,runpy,shutil,signal,subprocess,sys,time,traceback,zipfile
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
def main():
    p=argparse.ArgumentParser()
    for k in ('archive','root','sha256'):p.add_argument('--'+k,required=True)
    a=p.parse_args(); root=Path(a.root).resolve(); archive=Path(a.archive).resolve()
    assert os.uname().machine=='aarch64' and root.parent==Path.home() and root.name=='it-arena-r454-a'
    assert not root.exists() and digest(archive)==a.sha256
    root.mkdir()
    with zipfile.ZipFile(archive) as source:
        for n in source.namelist():assert not Path(n).is_absolute() and '..' not in Path(n).parts and '\\' not in n
        source.extractall(root)
    manifest_path=root/'source_manifest.json'; manifest=json.loads(manifest_path.read_bytes()); manifest_hash=digest(manifest_path)
    assert digest(__file__)==digest(root/'remote_bootstrap.py')
    helpers=runpy.run_path(str(root/'r447_bootstrap_reference.py'))
    def changed():
        return [n for n,h in manifest['files'].items() if digest(root/n)!=h]+([] if digest(manifest_path)==manifest_hash else ['source_manifest.json'])
    assert not changed()
    # 사전 접속 확인 뒤 경합이 새로 생겼으면 계산을 시작하지 않는다.
    interference=helpers['other_work'](root)
    before=helpers['safe_environment_snapshot'](); step=None; error=None; native=None
    started=time.monotonic()
    try:
        assert not any(interference.values()),interference
        prior=Path('/home/q/it-arena-r447-a')
        for name,h in manifest['native_sources_match_r447'].items():assert digest(prior/name)==h,name
        candidates=list((prior/'src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'))
        assert len(candidates)==1
        binary=candidates[0]; target=root/'src/arena_autonomy/arena_autonomy'/binary.name
        shutil.copyfile(binary,target)
        native=dict(source=str(binary),destination=str(target),sha256=digest(target),source_sha256=digest(binary),rebuild=False)
        assert native['sha256']==native['source_sha256']
        env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=os.pathsep.join([
            str(root/'src/arena_autonomy'),str(root/'src/arena_vehicle_interface'),str(root)]))
        command=[sys.executable,'-B',str(root/'run_cost.py')]
        with (root/'cost.log').open('x') as log:
            child=subprocess.Popen(command,cwd=root,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            step=dict(command=command,pid=child.pid,timeout_s=120,forced=False)
            try:step['exit_code']=child.wait(timeout=120)
            except subprocess.TimeoutExpired:
                step['forced']=True;os.killpg(child.pid,signal.SIGINT)
                try:step['exit_code']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);step['exit_code']=child.wait(timeout=5)
            try:os.killpg(child.pid,0);step['residual_group']=True
            except ProcessLookupError:step['residual_group']=False
            if step['residual_group']:os.killpg(child.pid,signal.SIGKILL)
            if step['exit_code'] or step['forced'] or step['residual_group']:raise RuntimeError(step)
    except BaseException as exc:
        error=repr(exc);(root/'traceback.txt').write_text(traceback.format_exc())
    finally:
        remaining=[]
        for proc in Path('/proc').iterdir():
            if not proc.name.isdigit() or int(proc.name)==os.getpid():continue
            try:
                if (proc/'cwd').resolve(strict=True).is_relative_to(root):remaining.append(int(proc.name))
            except (FileNotFoundError,PermissionError,ProcessLookupError):pass
        receipt=dict(completed=error is None,error=error,step=step,native=native,source_changes=changed(),
            owned_remaining=remaining,wall_s=time.monotonic()-started,environment_before=before,
            environment_after=helpers['safe_environment_snapshot'](),interference=interference,
            archive_sha256=a.sha256,source_manifest_sha256=manifest_hash,devices_opened=False,settings_changed=False)
        (root/'execution.json').write_text(json.dumps(receipt,indent=2)+'\n')
        result=root.with_name(root.name+'-results.zip')
        with zipfile.ZipFile(result,'x',zipfile.ZIP_DEFLATED) as z:
            for item in [root/'cost.log',root/'traceback.txt',root/'execution.json',*(root/'cost').glob('*')]:
                if item.is_file():z.write(item,item.relative_to(root).as_posix())
        print(json.dumps({k:v for k,v in receipt.items() if not k.startswith('environment_')}),flush=True)
        print(json.dumps(dict(result_zip=str(result),sha256=digest(result))),flush=True)
    return 0 if receipt['completed'] and not receipt['source_changes'] and not remaining else 1
if __name__=='__main__':raise SystemExit(main())
