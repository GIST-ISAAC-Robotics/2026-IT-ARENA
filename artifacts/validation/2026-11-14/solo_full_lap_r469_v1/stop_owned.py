"""지속 코너 정체의 소유 시험에 STOP을 요청하고 평가기의 정상 회수를 기다린다."""
import hashlib
import json
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger
from std_msgs.msg import String

A = Path(__file__).resolve().parent
root = A.parents[3]
expected = [b'python3', b'artifacts/validation/2026-11-14/solo_full_lap_r469_v1/run.py']
matches = []
for proc in Path('/proc').iterdir():
    if not proc.name.isdigit():
        continue
    try:
        if (proc/'cmdline').read_bytes().split(b'\0')[:2] == expected:
            assert (proc/'cwd').resolve() == root
            matches.append(int(proc.name))
    except (FileNotFoundError, PermissionError):
        pass
assert len(matches) == 1, matches
pid = matches[0]
os.environ.update(ROS_DOMAIN_ID=str(150+pid % 30), ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
rclpy.init()
n = Node('r469_owned_stop')
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
(A/'early_stop.json').write_text(json.dumps(dict(
    reason='persistent no-progress or unsafe observed state; retain original full-run failure and collect evidence',
    helper_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
    owner_pid=pid, calls=calls, final_mcu=last, final_output=out, stop_confirmed=confirmed,
    original_acceptance_changed=False, early_abort_not_completed_trial=True,
    validator_signal_sent=False), indent=2)+'\n')
n.destroy_node()
rclpy.shutdown()
assert confirmed, 'inspect simulation before shutdown'
print('owned simulation STOP acknowledged; original validator performs STOP verification and cleanup')
