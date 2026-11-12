"""주행 없는 빈 Gazebo world에서 GZ→ROS 시계·Python timer 연결 진단."""
from pathlib import Path
from collections import Counter
import json,os,signal,subprocess,time
A=Path(__file__).resolve().parent
os.environ.update(ROS_DOMAIN_ID='212',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',GZ_PARTITION='arena_clock_probe_r211')
os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS','LARGE_DATA')
import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
world=A/'empty.sdf'
world.write_text('''<?xml version="1.0"?><sdf version="1.9"><world name="clock_probe">
<physics name="default" type="ignored"><max_step_size>0.001</max_step_size><real_time_factor>1</real_time_factor></physics>
<plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
</world></sdf>''')
processes=[];streams=[];counts=Counter();first=None;last=None;error=None
rclpy.init();node=Node('r211_clock_reader',parameter_overrides=[Parameter('use_sim_time',value=True)])
def receive(msg):
    global first,last
    value=msg.clock.sec+msg.clock.nanosec*1e-9
    counts['clock']+=1
    if first is None:first=value
    last=value
node.create_subscription(Clock,'/clock',receive,QoSProfile(depth=10,reliability=ReliabilityPolicy.BEST_EFFORT))
node.create_timer(.02,lambda:counts.update(timer=1))
try:
    for name,cmd in [('gazebo',['gz','sim','-s','-r',str(world)]),('bridge',['ros2','run','ros_gz_bridge','parameter_bridge','/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'])]:
        f=(A/(name+'.log')).open('x');streams.append(f)
        processes.append(subprocess.Popen(cmd,stdout=f,stderr=subprocess.STDOUT,start_new_session=True))
    deadline=time.monotonic()+20
    while time.monotonic()<deadline:
        rclpy.spin_once(node,timeout_sec=.02)
        if first is not None and last-first>=2 and counts['timer']>=50:break
    passed=bool(first is not None and last-first>=2 and counts['timer']>=50)
except BaseException as e:
    error=f'{type(e).__name__}: {e}';passed=False
finally:
    cleanup=[]
    for p in reversed(processes):
        forced=False
        if p.poll() is None:
            os.killpg(p.pid,signal.SIGINT)
            try:p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                forced=True;os.killpg(p.pid,signal.SIGKILL);p.wait(timeout=5)
        cleanup.append(dict(pid=p.pid,returncode=p.returncode,forced=forced))
    for f in streams:f.close()
    node.destroy_node();rclpy.shutdown()
result=dict(passed=passed,counts=dict(counts),first_s=first,last_s=last,error=error,cleanup=cleanup,scope='empty world clock transport only, no controller or sensor test')
with (A/'result.json').open('x') as f:json.dump(result,f,indent=2);f.write('\n')
print(json.dumps(result));raise SystemExit(0 if passed else 1)
