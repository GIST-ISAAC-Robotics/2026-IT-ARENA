"""출발 전 지속 거절을 확인한 소유 시험만 STOP 후 조기 회수한다."""
import json
import os
from pathlib import Path
import signal
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String

A = Path(__file__).resolve().parent
pid = 820
argv = Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')
assert argv[:2] == [b'python3', b'artifacts/validation/2026-11-13/published_route_drive_r338_v1/run.py']
os.environ.update(ROS_DOMAIN_ID=str(150+pid % 30), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
rclpy.init()
n = Node('r338_owned_stop')
statuses, outputs = [], []
n.create_subscription(String, '/actuation/status', lambda m: statuses.append(json.loads(m.data)), 20)
n.create_subscription(String, '/actuation/output_status', lambda m: outputs.append(json.loads(m.data)), 20)
calls = []
for srv, kind, request in [('/autonomy/enable', SetBool, SetBool.Request(data=False)),
                           ('/actuation/stop', Trigger, Trigger.Request())]:
    client = n.create_client(kind, srv)
    assert client.wait_for_service(timeout_sec=5.)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(n, future, timeout_sec=5.)
    assert future.done() and future.result().success
    calls.append(dict(service=srv, success=True, message=future.result().message))
until = time.monotonic()+3.
while time.monotonic() < until:
    rclpy.spin_once(n, timeout_sec=.02)
last = statuses[-1] if statuses else {}
out = outputs[-1] if outputs else {}
confirmed = last.get('state') == 'ESTOP_LATCHED' and out.get('speed_mps') == 0.
(A/'early_stop.json').write_text(json.dumps(dict(reason='persistent current-region rejection before progress; do not repeat the same failure for 80 s',
    owner_pid=pid, calls=calls, final_mcu=last, final_output=out, stop_confirmed=confirmed,
    original_acceptance_changed=False, early_abort_not_completed_trial=True), indent=2)+'\n')
n.destroy_node()
rclpy.shutdown()
assert confirmed, 'inspect simulation before shutdown'
assert Path(f'/proc/{pid}/cmdline').read_bytes().split(b'\0')[:2] == argv[:2]
os.kill(pid, signal.SIGINT)
print('owned simulation STOP acknowledged; validator interrupt sent')
