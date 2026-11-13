"""r476 단일 계산 자식 정상 회수. 기존 r470 ARM binary만 사본으로 재사용."""
from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,sys,traceback,zipfile
from environment_reference import other_work,safe_environment_snapshot
A=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    assert A==Path('/home/q/it-arena-r476-a') and os.uname().machine=='aarch64'
    manifest=A/'source_manifest.json';m=json.loads(manifest.read_bytes());mh=sha(manifest)
    assert m['root_authorized'] and m['jetson_exclusive'] and not (A/'execution_receipt.json').exists()
    before_work=other_work(A);assert not any(before_work.values()),before_work
    def changes():return [p for p,h in m['files'].items() if not (A/p).is_file() or sha(A/p)!=h]
    assert not changes()
    reuse=m['native_reuse'];native=Path(reuse['source']);assert sha(native)==reuse['sha256']
    source_root=Path('/home/q/it-arena-r470-a/stage')
    for p,h in reuse['source_sha256'].items():assert sha(source_root/p)==h and sha(A/p)==h,p
    destination=A/'src/arena_autonomy/arena_autonomy'/native.name
    assert not destination.exists();shutil.copyfile(native,destination)
    (A/'results').mkdir();before=safe_environment_snapshot();step={};error=None
    try:
        with (A/'run_cost.log').open('x') as log:
            child=subprocess.Popen([sys.executable,'-B',str(A/'run_cost.py')],cwd=A,
                env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            step=dict(pid=child.pid,command=[sys.executable,'-B',str(A/'run_cost.py')],forced=False)
            (A/'run.pid').write_text(str(child.pid))
            try:step['exit_code']=child.wait(timeout=240)
            except subprocess.TimeoutExpired:
                step['forced']=True;os.killpg(child.pid,signal.SIGINT)
                try:step['exit_code']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);step['exit_code']=child.wait(timeout=5)
            try:os.killpg(child.pid,0)
            except ProcessLookupError:step['residual_group']=False
            else:step['residual_group']=True;os.killpg(child.pid,signal.SIGKILL)
            assert step['exit_code']==0 and not step['forced'] and not step['residual_group'],step
    except BaseException as exc:error=repr(exc);(A/'traceback.txt').write_text(traceback.format_exc())
    finally:
        receipt=dict(completed=error is None,error=error,step=step,source_changes=changes(),
            manifest_unchanged=mh==sha(manifest),native_original_unchanged=sha(native)==reuse['sha256'],
            native_copied_sha256=sha(destination),native_source_hash_verified=True,native_build=False,
            environment_before=before,environment_after=safe_environment_snapshot(),preflight_work=before_work,
            remaining=other_work(A),ports_opened=False,settings_changed=False,packages_installed=False,
            ros_started=False,driving=False)
        (A/'execution_receipt.json').write_text(json.dumps(receipt,indent=2)+'\n')
        archive=A.with_name(A.name+'-results.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in [A/'execution_receipt.json',A/'traceback.txt',*A.glob('*.log'),*(A/'results').glob('*')]:
                if p.is_file():z.write(p,p.relative_to(A).as_posix())
        print(json.dumps(dict(completed=receipt['completed'],error=error,step=step,source_changes=receipt['source_changes'],
            remaining=receipt['remaining'],archive=str(archive),archive_sha256=sha(archive))),flush=True)
    return 0 if receipt['completed'] and not receipt['source_changes'] and not any(receipt['remaining'].values()) else 1
if __name__=='__main__':raise SystemExit(main())
