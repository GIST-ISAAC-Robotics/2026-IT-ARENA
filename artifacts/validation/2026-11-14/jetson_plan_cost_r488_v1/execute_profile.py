"""r482 소유 계산 자식·원본 무결성·PID 회수. 센서/포트 접근 없음."""
from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,sys,traceback,zipfile
from environment_reference import safe_environment_snapshot
A=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def other_work():
    result=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) in (os.getpid(),os.getppid()):continue
        try:
            if p.stat().st_uid!=os.getuid():continue
            argv=(p/'cmdline').read_bytes().split(b'\0');exe=(p/'exe').resolve().name;cwd=(p/'cwd').resolve()
            if exe not in ('bash','sh','dash','sshd','sshd-session') and ('it-arena' in str(cwd).lower()
                    or any('it-arena' in x.decode(errors='replace').lower() for x in argv[:2])
                    or exe in ('gz','gzserver','gzclient','cc1plus','g++','colcon')):
                result.append(dict(pid=int(p.name),exe=exe,cwd=str(cwd)))
        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    return result
def main():
    assert A==Path('/home/q/it-arena-r488-a') and os.uname().machine=='aarch64'
    m=json.loads((A/'manifest.json').read_bytes());mh=sha(A/'manifest.json')
    def changes():return [p for p,h in m['files'].items() if not (A/p).is_file() or sha(A/p)!=h]
    assert not changes();work=other_work();assert not work,work
    reuse=m['native_reuse'];native=Path(reuse['source']);assert sha(native)==reuse['sha256']
    source=Path('/home/q/it-arena-r470-a/stage')
    for p,h in reuse['source_sha256'].items():assert sha(source/p)==h and sha(A/'base'/p)==h,p
    dest=A/'base/src/arena_autonomy/arena_autonomy'/native.name
    assert not dest.exists();shutil.copy2(native,dest);assert sha(dest)==reuse['sha256']
    before=safe_environment_snapshot();step={};error=None
    try:
        with (A/'profile.log').open('x') as log:
            child=subprocess.Popen([sys.executable,'-B',str(A/'run_profile.py')],cwd=A,
                env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1'),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            step=dict(pid=child.pid,forced=False);(A/'profile.pid').write_text(str(child.pid))
            try:step['exit_code']=child.wait(timeout=120)
            except subprocess.TimeoutExpired:
                step['forced']=True;os.killpg(child.pid,signal.SIGINT)
                try:step['exit_code']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);step['exit_code']=child.wait(timeout=5)
            try:os.killpg(child.pid,0)
            except ProcessLookupError:step['residual_group']=False
            else:step['residual_group']=True;os.killpg(child.pid,signal.SIGKILL)
            assert step['exit_code']==0 and not step['forced'] and not step['residual_group'],step
    except BaseException as exc:error=repr(exc);(A/'profile_traceback.txt').write_text(traceback.format_exc())
    finally:
        receipt=dict(completed=error is None,error=error,step=step,source_changes=changes(),
            manifest_unchanged=sha(A/'manifest.json')==mh,native_original_unchanged=sha(native)==reuse['sha256'],
            native_copied_sha256=sha(dest),environment_before=before,environment_after=safe_environment_snapshot(),
            preflight_work=work,remaining=other_work(),devices_or_ports_opened=False,settings_changed=False,
            packages_installed=False,ros_started=False,driving=False)
        (A/'profile_execution.json').write_text(json.dumps(receipt,indent=2)+'\n')
        archive=A.with_name(A.name+'-profile-results.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in [A/'profile_execution.json',A/'profile_traceback.txt',A/'profile.log',
                      *(A/'profile_results').glob('*'),A/'manifest.json']:
                if p.is_file():z.write(p,p.relative_to(A).as_posix())
        print(json.dumps(dict(completed=receipt['completed'],error=error,step=step,source_changes=receipt['source_changes'],
            remaining=receipt['remaining'],archive=str(archive),archive_sha256=sha(archive))),flush=True)
    return 0 if receipt['completed'] and not receipt['source_changes'] and not receipt['remaining'] else 1
if __name__=='__main__':raise SystemExit(main())
