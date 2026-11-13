"""일시중지 회수: 프로세스 조회만 수행. ROS/센서/시험을 시작하지 않는다."""
import json,os
from pathlib import Path

root=Path('/mnt/c/Users/Jinhyeong/Documents/ChatGPT/IT ARENA local/artifacts/validation/2026-11-13/vehicle_marker_r466_v1')
old=json.loads((root/'execution_v1/run1/result.json').read_bytes())
known=[json.loads((root/'execution_v1/run1/pid.json').read_bytes())['pid'],*old['child_processes']]
def inspect(pid):
    proc=Path('/proc',str(pid))
    try:
        command=(proc/'cmdline').read_bytes().decode(errors='replace').replace('\0',' ')
        status=(proc/'status').read_text()
        return dict(pid=pid,command=command,state=next(x for x in status.splitlines() if x.startswith('State:')),
            start_ticks=(proc/'stat').read_text().split()[21])
    except (OSError,ProcessLookupError):return dict(pid=pid,absent=True)
matching=[]
for path in Path('/proc').iterdir():
    if not path.name.isdigit() or int(path.name) in (os.getpid(),os.getppid()):continue
    record=inspect(int(path.name));cmd=record.get('command','')
    if '/home/jinhyeong/it-arena-r466-ros-a' in cmd or 'run_ros_validation.py' in cmd:
        matching.append(record)
audit=dict(current_audit_pid=os.getpid(),current_audit_parent=os.getppid(),
    known_pid_current_identity=[inspect(pid) for pid in known],owned_matching_processes=matching,
    owned_matching_count=len(matching),original_vision_cleanup=old['vision_cleanup'],
    original_child_alive_after_close=old['vision_child_alive_after_close'],
    meaning='current PID identity is checked; PID existence alone is not treated as the original process',
    additional_tests_started=False)
(root/'execution_v1/pause_pid_audit.json').write_text(json.dumps(audit,indent=2)+'\n')
print(json.dumps(audit,indent=2))
