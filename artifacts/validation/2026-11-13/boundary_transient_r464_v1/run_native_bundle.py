"""한 배정 세션에서 국소 회귀/원본/후보를 순차 실행하고 소유 PID를 회수한다."""
from pathlib import Path
import json,os,signal,subprocess,sys,time

HERE=Path(__file__).resolve().parent
ROOT=HERE/'execution_v1'
receipt=ROOT/'bundle_receipt_v3.json'
assert not receipt.exists()
os.environ['PYTHONDONTWRITEBYTECODE']='1'
os.environ['R464_PACKAGE_ROOT']=str(ROOT/'candidate')
data={'pid':os.getpid(),'started_unix_s':time.time(),'steps':[],'allowed':'pytest9 + native baseline405/candidate405',
      'ROS_started':False,'new_drive':False,'Jetson_used':False}
def save():receipt.write_text(json.dumps(data,indent=2),encoding='utf-8')
def ancestors():
    ids={os.getpid()};pid=os.getpid()
    while pid>1:
        fields=(Path('/proc')/str(pid)/'stat').read_text().split(') ',1)[1].split()
        pid=int(fields[1]);ids.add(pid)
    return ids
def process_audit():
    own=ancestors();matching=[];competition=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():continue
        try:cmd=(entry/'cmdline').read_bytes().replace(b'\0',b' ').decode(errors='replace')
        except OSError:continue
        pid=int(entry.name)
        if pid in own:continue
        if 'boundary_transient_r464_v1' in cmd:matching.append({'pid':pid,'cmd':cmd})
        if any(s in cmd for s in ('gz sim','ros2 launch','solo_current_mask_r463_v1/','run_native_bundle.py')):
            competition.append({'pid':pid,'cmd':cmd})
    return {'excluded_own_carrier_ancestors':sorted(own),'matching_r464':matching,'competition':competition}
data['before']=process_audit();save()
assert not data['before']['matching_r464'] and not data['before']['competition']
try:
    commands=[('pytest_remaining1',[sys.executable,'-B',str(HERE/'run_focused_tests.py'),'-k','test_refresh_keeps_real_occupied_points_in_hold_check']),
              ('baseline405',[sys.executable,'-B',str(HERE/'native_compare.py'),'baseline']),
              ('candidate405',[sys.executable,'-B',str(HERE/'native_compare.py'),'candidate'])]
    for name,cmd in commands:
        logfile=ROOT/(name+'_v3.log')
        began=time.monotonic()
        with logfile.open('w',encoding='utf-8') as output:
            process=subprocess.Popen(cmd,cwd=HERE,stdout=output,stderr=subprocess.STDOUT,start_new_session=True)
            item={'name':name,'pid':process.pid,'command':cmd,'log':str(logfile),'exit_code':None}
            data['steps'].append(item);save()
            try:item['exit_code']=process.wait(timeout=300)
            except subprocess.TimeoutExpired:
                item['timeout']=True
                os.killpg(process.pid,signal.SIGTERM)
                try:item['exit_code']=process.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid,signal.SIGKILL);item['exit_code']=process.wait(timeout=5)
        item['wall_s']=time.monotonic()-began
        item['pid_exists_after_wait']=(Path('/proc')/str(process.pid)).exists();save()
        print(json.dumps(item),flush=True)
        if item['exit_code']!=0:raise RuntimeError(name+' failed; preserve original log and stop bundle')
    data['success']=True
except BaseException as exc:
    data['success']=False;data['error']={'type':type(exc).__name__,'message':str(exc)}
    raise
finally:
    data['after']=process_audit();data['finished_unix_s']=time.time();save()
    print(json.dumps({'success':data.get('success'),'after':data['after']}),flush=True)
