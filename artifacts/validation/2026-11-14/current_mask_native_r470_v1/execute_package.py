"""할당된 r470 ARM 사본만 빌드·검사·측정하고 소유 PID를 정상 회수한다."""
import hashlib,json,os
from pathlib import Path
import shutil,signal,subprocess,sys,traceback,zipfile
from environment_reference import other_work,safe_environment_snapshot

ROOT=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()

def main():
    assert ROOT==Path('/home/q/it-arena-r470-a') and os.uname().machine=='aarch64'
    manifest=ROOT/'source_manifest.json';m=json.loads(manifest.read_bytes());manifest_sha=sha(manifest)
    assert m['root_reviewed'] and m['jetson_exclusive_resource_assigned']
    assert m['computations']==24 and m['warmups']==2
    assert not (ROOT/'execution_receipt.json').exists()
    interference=other_work(ROOT);assert not any(interference.values()),interference
    def changed():
        found=[p for p,h in m['files'].items() if not (ROOT/p).is_file() or sha(ROOT/p)!=h]
        if sha(manifest)!=manifest_sha:found.append('source_manifest.json')
        return found
    assert not changed()
    native=m['native_reuse'];source=Path(native['source']);assert sha(source)==native['sha256']
    for p,h in native['original_source_sha256'].items():assert sha(ROOT/'base'/p)==h
    destination=ROOT/'base/src/arena_autonomy/arena_autonomy'/source.name
    assert not destination.exists();shutil.copyfile(source,destination)
    (ROOT/'results').mkdir(exist_ok=False)
    environment=dict(os.environ,PYTHONDONTWRITEBYTECODE='1',PYTHONPATH=str(ROOT))
    before=safe_environment_snapshot();steps=[];error=None;stage_binary=None
    def execute(label,command,cwd,timeout):
        with (ROOT/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=environment,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            row=dict(label=label,command=command,pid=child.pid,timeout_s=timeout,forced=False);steps.append(row)
            try:row['returncode']=child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                row['forced']=True;os.killpg(child.pid,signal.SIGINT)
                try:row['returncode']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);row['returncode']=child.wait(timeout=5)
            try:os.killpg(child.pid,0)
            except ProcessLookupError:row['residual_group']=False
            else:
                row['residual_group']=True;os.killpg(child.pid,signal.SIGKILL)
            assert row['returncode']==0 and not row['forced'] and not row['residual_group'],row
    try:
        execute('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],ROOT/'stage/src/arena_autonomy',180)
        candidates=list((ROOT/'stage/src/arena_autonomy/arena_autonomy').glob('_numeric_kernels*.so'))
        assert len(candidates)==1
        stage_binary=dict(path=str(candidates[0]),sha256=sha(candidates[0]))
        execute('native_verify',[sys.executable,'-B',str(ROOT/'verify_native.py')],ROOT,60)
        for version in ('base','stage'):
            execute(version,[sys.executable,'-B',str(ROOT/'run_cost.py'),version],ROOT,60)
    except BaseException as exc:
        error=repr(exc);(ROOT/'traceback.txt').write_text(traceback.format_exc())
    finally:
        receipt=dict(completed=error is None,error=error,steps=steps,source_changes=changed(),
            source_manifest_sha256=manifest_sha,base_native_sha256=sha(destination),stage_binary=stage_binary,
            native_build_in_isolated_stage=True,packages_installed=False,environment_before=before,
            environment_after=safe_environment_snapshot(),preflight_interference=interference,
            remaining=other_work(ROOT),hardware_activation=False,physical_measurement=False,ports_opened=False)
        (ROOT/'execution_receipt.json').write_text(json.dumps(receipt,indent=2))
        archive=ROOT.with_name(ROOT.name+'-results.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in [ROOT/'execution_receipt.json',ROOT/'traceback.txt',*ROOT.glob('*.log'),*(ROOT/'results').glob('*.json')]:
                if p.is_file():z.write(p,p.relative_to(ROOT).as_posix())
        print(json.dumps(dict(completed=receipt['completed'],error=error,steps=steps,source_changes=receipt['source_changes'],
            remaining=receipt['remaining'],stage_binary=stage_binary,archive=str(archive),archive_sha256=sha(archive))))
    return 0 if error is None and not receipt['source_changes'] and not any(receipt['remaining'].values()) else 1

if __name__=='__main__':raise SystemExit(main())
