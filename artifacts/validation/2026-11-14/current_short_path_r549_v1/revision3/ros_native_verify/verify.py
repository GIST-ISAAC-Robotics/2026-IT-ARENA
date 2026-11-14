"""ROS 환경의 단일 회귀와 지정 저장 입력 진단만 실행·회수한다."""
import datetime,hashlib,importlib,json,os,signal,subprocess,sys,time,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[5];REV=A.parent;R549=REV.parent
STORED=ROOT/'artifacts/validation/2026-11-14/measured_steering_reacquire_lap_r550_v1/stall_extract'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
read=lambda p:json.loads(Path(p).read_bytes())
def write(name,value):
    with (A/name).open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')
def owned():
    rows=[]
    for p in Path('/proc').glob('[0-9]*'):
        if int(p.name) in (os.getpid(),os.getppid()):continue
        try:
            exe=(p/'exe').resolve().name;env=(p/'environ').read_bytes().split(b'\0')
            if exe in ('bash','sh','dash'):continue
            if b'ROS_DOMAIN_ID=223' in env or any(v.startswith(b'GZ_PARTITION=arena_autonomy_test_') for v in env) or exe in ('gzserver','gzclient','gz','ruby'):rows.append(dict(pid=int(p.name),exe=exe))
        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    return rows
def execute(label,command):
    started=time.monotonic();timed=False;forced=False
    with (A/(label+'.log')).open('x',encoding='utf-8') as stream:
        process=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        write(label+'_started.json',dict(pid=process.pid,supervisor_pid=os.getpid(),command=command,at_utc=datetime.datetime.now(datetime.timezone.utc).isoformat()))
        print(json.dumps(dict(stage=label,pid=process.pid,started=True)),flush=True)
        try:code=process.wait(timeout=120)
        except subprocess.TimeoutExpired:
            timed=True;os.killpg(process.pid,signal.SIGINT)
            try:code=process.wait(timeout=10)
            except subprocess.TimeoutExpired:forced=True;os.killpg(process.pid,signal.SIGKILL);code=process.wait(timeout=5)
    result=dict(pid=process.pid,exit_code=code,elapsed_wall_s=time.monotonic()-started,timed_out=timed,forced_kill=forced,
        remaining_owned_processes=owned(),log_sha256=sha(A/(label+'.log')))
    write(label+'_result.json',result);print(json.dumps(dict(stage=label,**result)),flush=True)
    return result
def main():
    assert Path.cwd()==ROOT
    manifest=read(REV/'manifest.json');sources={p:v['after'] for p,v in manifest.items()}
    assert all(sha(ROOT/p)==h for p,h in sources.items())
    preserved=[REV/n for n in ('manifest.json','regression.xml','regression_fixed.xml','ros_trace.xml')]
    preserved += [R549/'diagnose.py',ROOT/'tests/test_corner_follow.py',ROOT/'tests/test_follow_trace.py',ROOT/'scripts/probe_corner_follow.py']
    preserved += list(STORED.glob('*.json'))
    inputs={p.relative_to(ROOT).as_posix():sha(p) for p in preserved};imports={}
    for name in ('rclpy','ackermann_msgs.msg','sensor_msgs.msg','std_msgs.msg','pytest','arena_autonomy.local_path','arena_autonomy.local_follow','arena_autonomy.current_wall_reacquire','arena_autonomy._numeric_kernels'):
        module=importlib.import_module(name);imports[name]=str(Path(module.__file__).resolve())
    for name in ('local_path','local_follow','current_wall_reacquire'):
        assert Path(imports['arena_autonomy.'+name])==(ROOT/'src/arena_autonomy/arena_autonomy'/(name+'.py')).resolve()
    assert not owned()
    diagnostic=REV/'r550_diagnostic.json';assert not diagnostic.exists()
    write('preflight.json',dict(imports=imports,source_sha256=sources,preserved_inputs_sha256=inputs,
        environment={k:os.environ.get(k) for k in ('ROS_DISTRO','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','ROS_STATIC_PEERS','PYTHONPATH')},
        reused_environment='guard_status_payload_r543_v1/wsl_ros_verification_v1/run.sh',source_or_config_edited=False,driving_started=False,physical_device_access=False))
    test=execute('corner_counterfactual',[sys.executable,'-m','pytest','tests/test_corner_follow.py::test_counterfactual_is_labeled_and_does_not_mutate_or_promote_trace','-q','--tb=short','-p','no:cacheprovider','--junitxml='+str(A/'corner_counterfactual.xml')])
    assert not test['remaining_owned_processes']
    diag=execute('r550_diagnostic',[sys.executable,str(R549/'diagnose.py'),'revision3/r550_diagnostic.json',STORED.relative_to(ROOT).as_posix()])
    cases=ET.parse(A/'corner_counterfactual.xml').getroot().findall('.//testcase') if (A/'corner_counterfactual.xml').exists() else []
    counts=dict(tests=len(cases),failures=sum(c.find('failure') is not None for c in cases),errors=sum(c.find('error') is not None for c in cases),skips=sum(c.find('skipped') is not None for c in cases))
    changed=[p for p,h in {**inputs,**sources}.items() if sha(ROOT/p)!=h]
    rows=read(diagnostic) if diagnostic.exists() else []
    result=dict(passed=test['exit_code']==diag['exit_code']==0 and counts==dict(tests=1,failures=0,errors=0,skips=0) and not changed and not owned(),
        test=test,counts=counts,diagnostic=diag,diagnostic_path=diagnostic.relative_to(ROOT).as_posix(),diagnostic_sha256=sha(diagnostic) if diagnostic.exists() else None,
        diagnostic_rows=len(rows),outputs=[dict(case=r['case'],phase=r['phase'],reference_s=r['reference_s'],output=None if r['output'] is None else r['output'][:2],path_error=r.get('path_error'),pursue_error=r.get('pursue_error')) for r in rows],
        source_or_input_changes=changed,remaining_owned_processes=owned(),old_regression_failures_preserved=True,
        old_ros_failure='regression_fixed.xml: ModuleNotFoundError rclpy',old_hash_contract_failure_retested=False,driving_started=False,physical_device_access=False)
    write('result.json',result);print(json.dumps(result),flush=True)
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
