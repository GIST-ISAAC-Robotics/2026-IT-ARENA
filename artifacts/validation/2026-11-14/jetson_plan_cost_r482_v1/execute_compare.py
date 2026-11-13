"""r482 비교 및 관련 회귀의 독점 실행/회수. 기존 소스와 binary 불변."""
from pathlib import Path
import hashlib,json,os,shutil,signal,subprocess,sys,traceback,zipfile
from environment_reference import safe_environment_snapshot
from execute_profile_v2 import other_work
A=Path(__file__).resolve().parent
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
def main():
    assert A==Path('/home/q/it-arena-r482-a') and os.uname().machine=='aarch64'
    m=json.loads((A/'compare_manifest.json').read_bytes());mh=sha(A/'compare_manifest.json')
    def changes():return [p for p,h in m['files'].items() if not (A/p).is_file() or sha(A/p)!=h]
    assert not changes();work=other_work();assert not work,work
    reuse=m['native_reuse'];native=Path(reuse['source']);assert sha(native)==reuse['sha256']
    dest=A/'stage/src/arena_autonomy/arena_autonomy'/native.name
    assert not dest.exists();shutil.copy2(native,dest)
    assert sha(dest)==reuse['sha256']
    before=safe_environment_snapshot();steps=[];error=None
    env=dict(os.environ,PYTHONDONTWRITEBYTECODE='1')
    def run(label,command,run_env=env):
        with (A/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=A,env=run_env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            step=dict(label=label,pid=child.pid,forced=False);steps.append(step)
            (A/'compare.pid').write_text(str(child.pid))
            try:step['exit_code']=child.wait(timeout=120)
            except subprocess.TimeoutExpired:
                step['forced']=True;os.killpg(child.pid,signal.SIGINT)
                try:step['exit_code']=child.wait(timeout=5)
                except subprocess.TimeoutExpired:os.killpg(child.pid,signal.SIGKILL);step['exit_code']=child.wait(timeout=5)
            try:os.killpg(child.pid,0)
            except ProcessLookupError:step['residual_group']=False
            else:step['residual_group']=True;os.killpg(child.pid,signal.SIGKILL)
            assert step['exit_code']==0 and not step['forced'] and not step['residual_group'],step
    try:
        test_env=dict(env,PYTHONPATH=':'.join(str(A/'stage/src'/p) for p in ('arena_autonomy','arena_vehicle_interface')))
        run('compare_pytest',[sys.executable,'-B','-m','pytest','-q','-p','no:cacheprovider',
            'stage/tests/test_current_lane_preference.py','stage/tests/test_current_lane_region_reuse.py',
            '-k','not explicit_launch_option_and_recorded_header'],test_env)
        for round_id in range(3):
            for version in (('base','stage') if round_id!=1 else ('stage','base')):
                run(f'compare_{version}_{round_id}',[sys.executable,'-B',str(A/'run_compare.py'),version,str(round_id)])
    except BaseException as exc:error=repr(exc);(A/'compare_traceback.txt').write_text(traceback.format_exc())
    finally:
        receipt=dict(completed=error is None,error=error,steps=steps,source_changes=changes(),
            manifest_unchanged=sha(A/'compare_manifest.json')==mh,native_original_unchanged=sha(native)==reuse['sha256'],
            native_copied_sha256=sha(dest),environment_before=before,environment_after=safe_environment_snapshot(),
            preflight_work=work,remaining=other_work(),devices_or_ports_opened=False,settings_changed=False,
            packages_installed=False,ros_started=False,driving=False)
        (A/'compare_execution.json').write_text(json.dumps(receipt,indent=2)+'\n')
        archive=A.with_name(A.name+'-compare-results.zip')
        with zipfile.ZipFile(archive,'x',zipfile.ZIP_DEFLATED) as z:
            for p in [A/'compare_execution.json',A/'compare_traceback.txt',*A.glob('compare_*.log'),
                      *(A/'compare_results').glob('*')]:
                if p.is_file():z.write(p,p.relative_to(A).as_posix())
        print(json.dumps(dict(completed=receipt['completed'],error=error,steps=steps,source_changes=receipt['source_changes'],
            remaining=receipt['remaining'],archive=str(archive),archive_sha256=sha(archive))),flush=True)
    return 0 if receipt['completed'] and not receipt['source_changes'] and not receipt['remaining'] else 1
if __name__=='__main__':raise SystemExit(main())
