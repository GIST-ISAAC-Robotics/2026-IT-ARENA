"""I/O 작업을 멈추지 않고 ROS 소유 작업에 350 ms 대기를 1회 삽입한다."""
from pathlib import Path
import json,runpy,time
import rclpy
A=Path(__file__).resolve().parent;root=A.parents[3]
original=rclpy.spin_once;began=None;done=False
def spin_once(node,*args,**kwargs):
    global began,done
    now=time.monotonic()
    if began is None:began=now
    if not done and now-began>=3.:
        done=True;start=time.monotonic_ns();time.sleep(.350);end=time.monotonic_ns()
        (A/'run/injected_stall.json').write_text(json.dumps(dict(start_monotonic_ns=start,
            end_monotonic_ns=end,duration_ms=(end-start)/1e6,scope='ROS owner only; sleep releases GIL; not a system or GIL-wide stall'))+'\n')
    return original(node,*args,**kwargs)
rclpy.spin_once=spin_once
runpy.run_path(str(root/'scripts/bridge_sensor_link_preview.py'),run_name='__main__')
