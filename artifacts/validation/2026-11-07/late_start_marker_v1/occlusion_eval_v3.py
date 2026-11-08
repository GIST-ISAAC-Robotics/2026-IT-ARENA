"""새 코스 가림판·B0390 실제 RGB·B3 늦은 출발 제한 평가. 판은 상대 차량 모델이 아니다."""
import inspect
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import threading
import time
from types import SimpleNamespace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
import race_ops_eval as base
import validate_basic_autonomy as solo

# 옛 운영 좌표는 실행하지 않는다. 이 래퍼가 아래 run 전체를 새로 등록한다.
base.REGISTERED_SCENARIOS['official_v2026_11_06'] = ('representative',)
scene=json.loads((ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06/scene.json').read_text())
slot=next(s for s in scene['starting_grid']['slots'] if s['index']==3)
screen_x=slot['x']+.34*math.cos(slot['yaw_rad'])
screen_y=slot['y']+.34*math.sin(slot['yaw_rad'])
SCREEN = HERE/'screen_v3.sdf'
SCREEN.write_text(f'''<sdf version="1.9"><model name="eval_late_start_screen"><static>true</static>
<pose>{screen_x} {screen_y} .205 0 0 {slot["yaw_rad"]}</pose><link name="screen"><visual name="opaque">
<geometry><box><size>.006 .35 .22</size></box></geometry><material>
<ambient>.15 .15 .15 1</ambient><diffuse>.15 .15 .15 1</diffuse></material>
</visual></link></model></sdf>''', encoding='utf-8')
original_launch = solo.start_demo_process

def launch(*args, **kwargs):
    bound = inspect.signature(original_launch).bind(*args, **kwargs)
    extra = list(bound.arguments.get('extra_arguments', ()))
    extra += ['g1_piecewise_wall_trial:=true', 'g1_marker_late_start_trial:=true',
              f'evaluation_actor_sdf:={SCREEN}']
    bound.arguments['extra_arguments'] = extra
    return original_launch(*bound.args, **bound.kwargs)
solo.start_demo_process = launch


def run(self):
    import cv2
    import rclpy
    from rclpy.parameter import Parameter
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Image
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from arena_autonomy.core import image_rgb, marker_ids
    rclpy.init()
    node = rclpy.create_node('late_start_evidence', parameter_overrides=[Parameter('use_sim_time', value=True)])
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    latest, rows = {}, []
    def rgb(message):
        latest['rgb'] = (message.header.stamp.sec+message.header.stamp.nanosec*1e-9, image_rgb(message).copy())
    def status(message):
        data = json.loads(message.data)
        latest['status'] = data
        rows.append(data)
    node.create_subscription(Image, '/camera/color/image_raw', rgb,
                             QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    node.create_subscription(String, '/autonomy/status', status, 20)
    clients = {name: node.create_client(Trigger, '/sim/traffic_light/set_'+name) for name in ('off','red','green')}
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    def now(): return node.get_clock().now().nanoseconds*1e-9
    def wait(predicate, timeout=120.):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if predicate():return
            time.sleep(.02)
        raise base.Stop('late start observation timeout')
    def light(name):
        client=clients[name]
        if not client.wait_for_service(timeout_sec=15.): raise base.Stop('signal service missing')
        future=client.call_async(Trigger.Request())
        wait(future.done, 15.)
        if not future.result().success: raise base.Stop('signal request rejected')
        self.note('signal_requested_'+name)
    def hold(seconds):
        end=now()+seconds
        wait(lambda: now()>=end, seconds*self.scale+10.)
    def picture(name):
        stamp, image=latest['rgb']
        cv2.imwrite(str(self.output/(name+'.png')), cv2.cvtColor(image,cv2.COLOR_RGB2BGR))
        self.note(name, capture_s=stamp, marker_ids=marker_ids(image,.02),
                  signal=latest.get('status',{}).get('signal'))
    try:
        self.ops.wait(lambda s: s['binding'].get('session') and s['race'].get('latch')=='process_start',120.,'process start')
        wait(lambda: 'rgb' in latest)
        initial=self.truth.latest()
        light('off')
        hold(.4)
        entry=self.press('prepare_start')
        self.stage(entry, ('policy_accepted',))
        self.wait(lambda s: s['race']['policy_reason']=='wait_visual_start' and s['mcu']['state'] in ('ARMED','ACTIVE'),20.,'armed waiting')
        light('red')
        hold(2.)
        picture('blocked_red')
        assert not self.ops.state['race']['started'], 'started through opaque red scene'
        assert latest['status']['signal'] != 'red4_confirmed', 'red lamps are visible despite screen'
        assert not marker_ids(latest['rgb'][1],.02), 'signal markers are visible despite screen'
        assert max(math.dist(p[1:3],initial[1:3]) for p in self.truth.rows) < .03, 'motion before start'
        light('green')
        green_begin=now()
        hold(.8)
        picture('blocked_green')
        assert not self.ops.state['race']['started'], 'started while screen still blocks lamps'
        req=f'name:"eval_late_start_screen" position {{x:{screen_x} y:{screen_y} z:-10}} orientation {{w:1}}'
        moved=subprocess.run(['gz','service','-s','/world/it_arena_track/set_pose', '--reqtype','gz.msgs.Pose',
            '--reptype','gz.msgs.Boolean','--timeout','3000','--req',req],capture_output=True,text=True,timeout=5.)
        if 'data: true' not in moved.stdout: raise base.Stop('screen move failed: '+moved.stdout)
        revealed=now()
        self.note('screen_removed', removed_sim_s=revealed)
        wait(lambda: self.ops.state['race']['started'] or now()-green_begin>=5.)
        picture('revealed_green')
        assert self.ops.state['race']['started'], 'no start within 5-second green window'
        started=now()
        seen=[r for r in rows if r.get('started')]
        assert seen and seen[0].get('signal')=='late_green_confirmed', 'did not use marker late start'
        start_pose=self.truth.latest()
        wait(lambda: math.dist(self.truth.latest()[1:3],start_pose[1:3])>=1. or now()-green_begin>=5.)
        light('off')
        wait(lambda: math.dist(self.truth.latest()[1:3],start_pose[1:3])>=1., 60.)
        self.note('late_start_progress', reveal_to_observed_start_s=started-revealed,
                  green_request_to_start_s=started-green_begin,
                  progress_m=math.dist(self.truth.latest()[1:3],start_pose[1:3]),
                  first_started_sim_s=seen[0]['sim_time_s'])
        self.stop_car()
        hold(.5)
        self.note('late_start_complete', final_truth_speed=self.truth.speed(),
                  selected_mode=latest['status'].get('vision',{}).get('signal_mode'))
    except AssertionError as error:
        raise base.Stop(str(error)) from error
    finally:
        executor.shutdown(timeout_sec=5.)
        thread.join(timeout=5.)
        node.destroy_node()
        rclpy.shutdown()
        (self.output/'autonomy_trace.json').write_text(json.dumps(rows,ensure_ascii=False),encoding='utf-8')
base.Scenario.run = run

if __name__=='__main__':
    raise SystemExit(base.main())
