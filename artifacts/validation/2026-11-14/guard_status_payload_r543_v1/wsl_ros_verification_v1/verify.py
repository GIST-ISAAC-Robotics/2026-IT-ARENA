"""실제 ROS 상태 계약 2파일만 실행하고 PID/입력/종료를 회수한다."""
import datetime,hashlib,importlib,json,os,signal,subprocess,sys,time,xml.etree.ElementTree as ET
from pathlib import Path
A=Path(__file__).resolve().parent;ROOT=A.parents[4];TRANSFER=A.parent/'common_transfer'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()
now=lambda:datetime.datetime.now(datetime.timezone.utc).isoformat()
def write(name,value):
    with (A/name).open('x',encoding='utf-8') as f:json.dump(value,f,ensure_ascii=False,indent=2);f.write('\n')
def owned():
    result=[]
    for p in Path('/proc').glob('[0-9]*'):
        if int(p.name) in (os.getpid(),os.getppid()):continue
        try:
            exe=(p/'exe').resolve().name;env=(p/'environ').read_bytes().split(b'\0')
            if exe in ('bash','sh','dash'):continue
            if b'ROS_DOMAIN_ID=223' in env:result.append(dict(pid=int(p.name),exe=exe))
        except (FileNotFoundError,PermissionError,ProcessLookupError):pass
    return result
def main():
    assert Path.cwd()==ROOT
    manifest=json.loads((TRANSFER/'manifest.json').read_bytes());expected={p:v['after_sha256'] for p,v in manifest['files'].items()}
    assert all(sha(ROOT/p)==h for p,h in expected.items())
    imports={}
    for name in ['rclpy','ackermann_msgs.msg','sensor_msgs.msg','std_msgs.msg','pytest','arena_autonomy.guard_status_payload','arena_autonomy.lidar_safety']:
        module=importlib.import_module(name);imports[name]=str(Path(module.__file__).resolve())
    for name in ['guard_status_payload','lidar_safety']:
        assert Path(imports['arena_autonomy.'+name])==(ROOT/'src/arena_autonomy/arena_autonomy'/(name+'.py')).resolve()
    before=owned();assert not before,before
    log=TRANSFER/'ros_pytest.log';junit=TRANSFER/'ros_pytest.xml';assert not log.exists() and not junit.exists()
    command=[sys.executable,'-m','pytest','tests/test_guard_status_payload.py','tests/test_guard_status_payload_ros.py','-q','--tb=short','-p','no:cacheprovider','--junitxml='+junit.relative_to(ROOT).as_posix()]
    write('preflight.json',dict(at_utc=now(),imports=imports,source_sha256=expected,command=command,
        environment={k:os.environ.get(k) for k in ('ROS_DISTRO','ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','ROS_AUTOMATIC_DISCOVERY_RANGE','ROS_STATIC_PEERS','PYTHONPATH')},competing_domain_processes=before,
        gazebo_started=False,driving_started=False,physical_device_access=False,source_or_config_edited=False,timeout_s=120))
    started=time.monotonic();timed_out=False;forced=False
    with log.open('x',encoding='utf-8') as stream:
        proc=subprocess.Popen(command,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        write('started.json',dict(at_utc=now(),pytest_pid=proc.pid,supervisor_pid=os.getpid(),log=str(log),junit=str(junit)))
        print(json.dumps(dict(started=True,pytest_pid=proc.pid)),flush=True)
        try:code=proc.wait(timeout=120)
        except subprocess.TimeoutExpired:
            timed_out=True;os.killpg(proc.pid,signal.SIGINT)
            try:code=proc.wait(timeout=10)
            except subprocess.TimeoutExpired:forced=True;os.killpg(proc.pid,signal.SIGKILL);code=proc.wait(timeout=5)
    remaining=owned();changed=[p for p,h in expected.items() if sha(ROOT/p)!=h]
    counts={}
    if junit.exists():
        cases=ET.parse(junit).getroot().findall('.//testcase');counts=dict(tests=len(cases),failures=sum(x.find('failure') is not None for x in cases),errors=sum(x.find('error') is not None for x in cases),skipped=sum(x.find('skipped') is not None for x in cases))
    value=dict(at_utc=now(),pytest_pid=proc.pid,exit_code=code,elapsed_wall_s=time.monotonic()-started,timed_out=timed_out,forced_kill=forced,
        counts=counts,changed_inputs=changed,remaining_owned_processes=remaining,normal_process_exit=code==0 and not timed_out and not forced,
        passed=code==0 and counts==dict(tests=21,failures=0,errors=0,skipped=0) and not changed and not remaining,
        log_sha256=sha(log),junit_sha256=sha(junit) if junit.exists() else None,gazebo_started=False,driving_started=False,physical_device_access=False)
    write('result.json',value);print(json.dumps(value),flush=True)
    return 0 if value['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
