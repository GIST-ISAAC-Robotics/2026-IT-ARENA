import json,time,statistics
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
from arena_autonomy.lidar_motion_ros import MotionInput
A=Path(__file__).resolve().parent;out=A/'run';out.mkdir(exist_ok=True)
rclpy.init(args=[])
node=Node('imu_preview_consumer',parameter_overrides=[
    Parameter('lidar_compensation',value='both'),Parameter('motion_imu_topic',value='/hardware_preview/imu'),
    Parameter('motion_imu_frame_id',value='lsm6dsox_sensor'),
    Parameter('motion_feedback_mode',value='drive_motor_shaft'),Parameter('motion_gear_ratio',value=2.5*81/17)])
rows=[];statuses=[];motion=MotionInput(node,record=rows.append)
node.create_subscription(String,'/hardware_preview/imu_status',lambda msg:statuses.append(json.loads(msg.data)),10)
(out/'receiver.ready').write_text('ready')
started=time.monotonic()
try:
    while rclpy.ok() and time.monotonic()-started<65:rclpy.spin_once(node,timeout_sec=.1)
except (KeyboardInterrupt,ExternalShutdownException):pass
finally:
    times=[r['stamp_s'] for r in rows if r['kind']=='gyro'];steps=[b-a for a,b in zip(times,times[1:])]
    ages=[r['received_s']-r['stamp_s'] for r in rows if r['kind']=='gyro']
    def summary(x):
        s=sorted(x);return {'count':len(x),'min':min(x),'median':statistics.median(x),'p95':s[int((len(s)-1)*.95)],'max':max(x)} if x else {'count':0}
    result={'counts':motion.counts,'rejections':motion.gyro_rejections,'gyro_step_s':summary(steps),'callback_age_s':summary(ages),'status_count':len(statuses),'statuses_keep_unverified':bool(statuses) and all(s['time_contract_verified'] is False and s['sensor_acquisition_timestamp'] is False for s in statuses),'motor_samples':len(motion.history.wheels.times),'drive_output':False,'mounting_calibrated':False,'scope':'common MotionInput gyro subscription only; FIFO/register causal estimate; no absolute host time/encoder/deskew/drive validation'}
    (out/'receiver_summary.json').write_text(json.dumps(result,indent=2))
    with (out/'receiver_rows.jsonl').open('x') as f:
        for r in rows:f.write(json.dumps(r)+'\n')
    print(json.dumps(result));node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
