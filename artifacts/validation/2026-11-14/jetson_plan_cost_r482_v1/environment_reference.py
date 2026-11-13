"""r447 독립 ARM 루트: native 빌드, 저장 입력 각 1회, 뒤이어 비구동 생성자 확인."""
from pathlib import Path
import argparse,hashlib,json,os,signal,subprocess,sys,time,traceback,zipfile

def digest(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def environment_snapshot():
    values={}
    for path in [Path('/proc/loadavg'),Path('/proc/meminfo'),
                 *Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq'),
                 *Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_governor'),
                 *Path('/sys/class/thermal').glob('thermal_zone*/temp')]:
        try:
            with path.open('rb') as stream:data=stream.read(4096)
            if not isinstance(data,bytes):raise TypeError('metadata read returned non-bytes')
            values[str(path)]=data.decode('utf8',errors='replace')
        except (OSError,TypeError) as error:values[str(path)]=dict(available=False,error=repr(error))
    return dict(monotonic_s=time.monotonic(),cpu_count=os.cpu_count(),readings=values,
        thread_environment={k:os.environ.get(k) for k in ('OPENBLAS_NUM_THREADS','OMP_NUM_THREADS','MKL_NUM_THREADS')},
        settings_changed=False)
def safe_environment_snapshot():
    try:return environment_snapshot()
    except Exception as error:return dict(available=False,error=repr(error),settings_changed=False)

def other_work(root):
    processes=[];ports=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit() or int(p.name) in (os.getpid(),os.getppid()):continue
        try:
            if p.stat().st_uid!=os.getuid():continue
            argv=(p/'cmdline').read_bytes().split(b'\0');command=' '.join(x.decode(errors='replace') for x in argv)
            exe=(p/'exe').resolve().name;cwd=(p/'cwd').resolve()
            # SSH shell command text contains this script path; inspect executable/cwd, not shell payload.
            if exe not in ('bash','sh','dash','sshd','sshd-session') and (
                'it-arena' in str(cwd).lower() or any('it-arena' in x.decode(errors='replace').lower() for x in argv[:2])
                or exe in ('gzserver','gzclient','gz','cc1plus','g++','colcon')):
                processes.append(dict(pid=int(p.name),exe=exe,cwd=str(cwd),command=command))
            for fd in (p/'fd').iterdir():
                try:target=str(fd.resolve())
                except OSError:continue
                if target.startswith(('/dev/ttyUSB','/dev/ttyACM','/dev/video','/dev/serial')):
                    ports.append(dict(pid=int(p.name),target=target))
        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    return dict(relevant_processes=processes,open_sensor_ports=ports)

def main():
    parser=argparse.ArgumentParser()
    for k in ('archive','root'):parser.add_argument('--'+k,type=Path,required=True)
    parser.add_argument('--sha256',required=True);a=parser.parse_args()
    assert sys.platform=='linux' and os.uname().machine=='aarch64'
    root=a.root.resolve();archive=a.archive.resolve();result_zip=root.with_name(root.name+'-results.zip')
    assert root.parent==Path.home() and root.name.startswith('it-arena-r447-') and not root.exists()
    assert not result_zip.exists() and digest(archive)==a.sha256
    interference=other_work(root);assert not any(interference.values()),interference
    root.mkdir()
    with zipfile.ZipFile(archive) as source:
        for n in source.namelist():assert not Path(n).is_absolute() and '..' not in Path(n).parts and '\\' not in n
        source.extractall(root)
    manifest_path=root/'source_manifest.json';manifest=json.loads(manifest_path.read_bytes());manifest_sha=digest(manifest_path)
    assert digest(Path(__file__))==digest(root/'remote_bootstrap.py')
    def changed():
        result=[n for n,h in manifest['files'].items() if not (root/n).is_file() or digest(root/n)!=h]
        if digest(manifest_path)!=manifest_sha:result.append('source_manifest.json')
        return result
    assert not changed()
    env=dict(os.environ,PYTHONPATH=os.pathsep.join([str(root/'src/arena_autonomy'),str(root/'src/arena_vehicle_interface'),
        str(root),os.environ.get('PYTHONPATH','')]),PYTHONDONTWRITEBYTECODE='1')
    steps=[];failure=None;began=time.monotonic();before=safe_environment_snapshot()
    def execute(label,command,cwd,timeout,environment=None):
        with (root/(label+'.log')).open('x') as log:
            child=subprocess.Popen(command,cwd=cwd,env=environment or env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
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
        execute('native_build',[sys.executable,'-B','setup.py','build_ext','--inplace'],root/'src/arena_autonomy',180)
        execute('import_preflight',[sys.executable,'-B',str(root/'import_preflight.py')],root,30)
        execute('simulation',[sys.executable,'-B',str(root/'run_cost.py'),'simulation'],root,180)
        execute('estimated',[sys.executable,'-B',str(root/'run_cost.py'),'estimated'],root,180)
        execute('summarize',[sys.executable,'-B',str(root/'run_cost.py'),'summarize'],root,30)
        execute('node_check',[sys.executable,'-B',str(root/'node_check/verify_nodes.py')],root,90,
            dict(env,ROS_DOMAIN_ID='147',ROS_LOCALHOST_ONLY='1'))
    except Exception as error:
        failure=repr(error);(root/'traceback.txt').write_text(traceback.format_exc())
    finally:
        path=root/'cost/report.json';report=json.loads(path.read_bytes()) if path.is_file() else {}
        source_changes=changed()
        remaining=[]
        for p in Path('/proc').iterdir():
            if not p.name.isdigit() or int(p.name)==os.getpid():continue
            try:
                if (p/'cwd').resolve(strict=True).is_relative_to(root):remaining.append(int(p.name))
            except (FileNotFoundError,PermissionError,ProcessLookupError):pass
        result=dict(execution_completed=failure is None and not source_changes and not remaining,error=failure,steps=steps,
            machine=os.uname().machine,root=str(root),archive_sha256=a.sha256,manifest_sha256=manifest_sha,
            source_count=len(manifest['files']),changed_sources=source_changes,owned_remaining=remaining,
            cost_collection_completed=report.get('cost_collection_completed'),same_arm_invariants_passed=report.get('same_arm_invariants_passed'),
            original_x86_comparator_passed=report.get('x86_existing_comparator_passed'),wall_s=time.monotonic()-began,
            environment_before=before,environment_after=safe_environment_snapshot(),interference=interference,
            devices_opened=False,settings_changed=False,full_ros_realtime_verified=False,physical_driving_verified=False)
        (root/'execution.json').write_text(json.dumps(result,indent=2)+'\n')
        with zipfile.ZipFile(result_zip,'x',zipfile.ZIP_DEFLATED) as output:
            for p in [*root.glob('*.log'),root/'execution.json',root/'import_preflight.json',root/'traceback.txt',
                      *(root/'cost').rglob('*'),*(root/'node_check').glob('*.json'),*(root/'node_check').glob('*.jsonl'),
                      root/'node_check/traceback.txt']:
                if p.is_file():output.write(p,p.relative_to(root).as_posix())
        print(json.dumps({k:v for k,v in result.items() if not k.startswith('environment_')}),flush=True)
        print(json.dumps(dict(result_zip=str(result_zip),result_sha256=digest(result_zip))),flush=True)
    return 0 if result['execution_completed'] else 1

if __name__=='__main__':raise SystemExit(main())
