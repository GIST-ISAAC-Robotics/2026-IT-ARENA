"""r382/r407의 새 루트·단계 실행·해시/소유 process group 회수. smoke/full 같은 도구."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import zipfile


def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def environment_snapshot():
    values={}
    for path in [Path('/proc/loadavg'),Path('/proc/meminfo'),
                 *Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq'),
                 *Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_governor'),
                 *Path('/sys/class/thermal').glob('thermal_zone*/temp')]:
        try:values[str(path)]=path.read_text()[:4096]
        except OSError as error:values[str(path)]=repr(error)
    return dict(monotonic_s=time.monotonic(),cpu_count=os.cpu_count(),readings=values,
        thread_environment={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
        settings_changed=False)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ('archive','root'):parser.add_argument('--'+key,type=Path,required=True)
    parser.add_argument('--sha256',required=True);parser.add_argument('--subset',choices=('smoke','full'),required=True)
    parser.add_argument('--smoke-report',type=Path);parser.add_argument('--smoke-sha256')
    args=parser.parse_args()
    assert sys.platform=='linux' and os.uname().machine==('x86_64' if args.subset=='smoke' else 'aarch64')
    archive=args.archive.resolve();root=args.root.resolve();result_zip=root.with_name(root.name+'-results.zip')
    assert digest(archive)==args.sha256
    assert root.parent==Path.home() and root.name.startswith('it-arena-r411-') and not root.exists()
    assert not result_zip.exists(),'preserve prior result archive'
    smoke=None
    if args.subset=='full':
        assert args.smoke_report and args.smoke_sha256 and digest(args.smoke_report)==args.smoke_sha256
        smoke=json.loads(args.smoke_report.read_bytes())
        assert smoke['subset']=='smoke' and smoke['machine']=='x86_64' and smoke['passed'] is True
        assert smoke['completed_calls']==80 and smoke['historical_exact_reproduction_passed'] is True
        assert smoke['same_machine_invariants_passed'] is True
    else:assert args.smoke_report is args.smoke_sha256 is None
    root.mkdir()
    with zipfile.ZipFile(archive) as source:
        for n in source.namelist():assert not Path(n).is_absolute() and '..' not in Path(n).parts and '\\' not in n
        source.extractall(root)
    work=root/'r411';manifest_path=root/'source_manifest.json';manifest=json.loads(manifest_path.read_bytes())
    manifest_sha=digest(manifest_path)
    assert digest(Path(__file__))==digest(work/'remote_bootstrap.py'),'bootstrap differs from archive'
    if smoke is not None:assert smoke['manifest_sha256']==manifest_sha,'smoke used another source/input manifest'
    def changed():
        result=[n for n,h in manifest['files'].items() if not (root/n).is_file() or digest(root/n)!=h]
        if digest(manifest_path)!=manifest_sha:result.append('source_manifest.json')
        return result
    assert not changed()
    env=dict(os.environ,PYTHONPATH=os.pathsep.join(str(root/p) for p in
        ('src/arena_autonomy','src/arena_vehicle_interface','r411')),PYTHONDONTWRITEBYTECODE='1')
    steps=[];failure=None;began=time.monotonic();before=environment_snapshot()
    def execute(label,command,cwd,timeout):
        with (work/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            record=dict(label=label,command=command,pid=child.pid,timeout_s=timeout,forced=False);steps.append(record)
            timed_out=False
            try:record['returncode']=child.wait(timeout=timeout)
            except subprocess.TimeoutExpired:
                timed_out=True;record['forced']=True;os.killpg(child.pid,signal.SIGINT)
                try:record['returncode']=child.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    os.killpg(child.pid,signal.SIGKILL);record['returncode']=child.wait(timeout=3)
            end=time.monotonic()+.5
            while True:
                try:os.killpg(child.pid,0)
                except ProcessLookupError:record['residual_group']=False;break
                if time.monotonic()>=end:record['residual_group']=True;break
                time.sleep(.01)
            if record['residual_group']:
                os.killpg(child.pid,signal.SIGKILL);raise RuntimeError((label,'residual process group force-reaped'))
            if timed_out:raise TimeoutError((label,timeout))
            if record['returncode']:raise RuntimeError((label,record['returncode']))
    try:
        execute('pure_helpers',[sys.executable,'-B',str(work/'test_helpers.py')],root,10)
        execute('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],root/'src/arena_autonomy',180)
        execute('import_preflight',[sys.executable,'-B',str(work/'preflight.py')],root,30)
        execute(args.subset+'_run',[sys.executable,'-B',str(work/'run.py'),'--subset',args.subset],root,180)
    except Exception as error:failure=repr(error)
    finally:
        path=work/args.subset/'report.json';report=json.loads(path.read_bytes()) if path.is_file() else {}
        source_changes=changed()
        result=dict(execution_completed=failure is None and not source_changes,error=failure,steps=steps,
            subset=args.subset,machine=os.uname().machine,root=str(root),source_archive_sha256=args.sha256,
            source_manifest_sha256=manifest_sha,source_count=len(manifest['files']),changed_sources=source_changes,
            cost_collection_completed=report.get('cost_collection_completed'),
            historical_exact_reproduction_passed=report.get('historical_exact_reproduction_passed'),
            same_machine_invariants_passed=report.get('same_machine_invariants_passed'),
            smoke_report_sha256=args.smoke_sha256,wall_s=time.monotonic()-began,
            environment_before=before,environment_after=environment_snapshot(),devices_opened=False,
            automatic_adoption_allowed=False,settings_changed=False,original_r406_passed=False)
        (work/'execution.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
        with zipfile.ZipFile(result_zip,'x',zipfile.ZIP_DEFLATED) as output:
            for p in [*work.glob('*.log'),work/'execution.json',work/'import_preflight.json',
                      * (work/args.subset).glob('*')]:
                if p.is_file():output.write(p,p.relative_to(work).as_posix())
        print(json.dumps({k:v for k,v in result.items() if not k.startswith('environment_')}),flush=True)
    if failure or source_changes:raise SystemExit(1)


if __name__=='__main__':main()
