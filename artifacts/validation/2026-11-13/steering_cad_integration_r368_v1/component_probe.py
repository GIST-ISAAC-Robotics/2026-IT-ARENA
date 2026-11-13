"""Gazebo의 고정 차체/무부하 조향 부품 검사. 자율주행/실물 노드와 연결하지 않는다."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import xml.etree.ElementTree as ET

import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64, String
from arena_vehicle_interface.steering_calibration import CadMapping

A=Path(__file__).resolve().parent
ROOT=A.parents[3]

def main():
    out=A/'component'
    out.mkdir(exist_ok=False)
    model=ET.parse(A.parent/'released_crawl_drive_r364_v1/run1/runtime_vehicle.sdf').getroot().find('model')
    model.set('name','servo_bench')
    for parent in model.iter():
        for child in list(parent):
            if child.tag=='sensor': parent.remove(child)
    for plugin in list(model.findall('plugin')):
        if plugin.get('name')!='arena::SingleMotorDrive': model.remove(plugin)
    drive=model.find("plugin[@name='arena::SingleMotorDrive']")
    ET.SubElement(drive,'steering_model').text='fusion_cad_2026_11_13'
    drive.find('steering_limit').text=str(math.pi/6)
    for name in ('front_left_steering_joint','front_right_steering_joint'):
        limits=model.find("joint[@name='%s']/axis/limit"%name)
        limits.find('lower').text=str(-math.pi/6)
        limits.find('upper').text=str(math.pi/6)
    fixed=ET.SubElement(model,'joint',name='test_fixed_chassis',type='fixed')
    ET.SubElement(fixed,'parent').text='world'
    ET.SubElement(fixed,'child').text='chassis'
    sdf=ET.Element('sdf',version='1.10');world=ET.SubElement(sdf,'world',name='servo_component')
    ET.SubElement(world,'gravity').text='0 0 0'
    physics=ET.SubElement(world,'physics',name='default',type='ignored')
    ET.SubElement(physics,'max_step_size').text='.001'
    ET.SubElement(physics,'real_time_factor').text='.5'
    ET.SubElement(world,'plugin',filename='gz-sim-physics-system',name='gz::sim::systems::Physics')
    world.append(model)
    path=out/'world.sdf'; ET.ElementTree(sdf).write(path,encoding='unicode')
    env=dict(os.environ,GZ_PARTITION='r368_'+str(os.getpid()),ROS_DOMAIN_ID='183',ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST')
    env['GZ_SIM_SYSTEM_PLUGIN_PATH']=str(ROOT/'install/arena_gazebo/lib')+':'+env.get('GZ_SIM_SYSTEM_PLUGIN_PATH','')
    os.environ.update({k:env[k] for k in ('ROS_DOMAIN_ID','ROS_AUTOMATIC_DISCOVERY_RANGE')})
    processes=[]; logs=[]; samples=[]; phases=[]; errors=[]
    node=None
    try:
        for name, command in (
            ('gazebo',['gz','sim','-s','-r',str(path)]),
            ('bridge',['ros2','run','ros_gz_bridge','parameter_bridge',
                '/model/servo_bench/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                '/model/servo_bench/steer_angle@std_msgs/msg/Float64]gz.msgs.Double',
                '/model/servo_bench/drivetrain@std_msgs/msg/String[gz.msgs.StringMsg'])):
            log=(out/(name+'.log')).open('w'); logs.append(log)
            processes.append(subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=True))
        rclpy.init();node=rclpy.create_node('r368_component_probe')
        sub=node.create_subscription(String,'/model/servo_bench/drivetrain',lambda m:samples.append(json.loads(m.data)),100)
        speed=node.create_publisher(Twist,'/model/servo_bench/cmd_vel',10)
        steer=node.create_publisher(Float64,'/model/servo_bench/steer_angle',10)
        until=time.monotonic()+30
        while not samples and time.monotonic()<until: rclpy.spin_once(node,timeout_sec=.05)
        if not samples: raise RuntimeError('no Gazebo diagnostics')
        mapping=CadMapping.default();geometry=dict(wheelbase=.128,track=.1239,wheel_limit=math.pi/6)
        limit=mapping.command_limit(**geometry)
        for target in (0.,.2,limit,0.,-.2,-limit,0.):
            begin=samples[-1]['sim_time_s']; deadline=time.monotonic()+15
            while samples[-1]['sim_time_s']<begin+1.5 and time.monotonic()<deadline:
                speed.publish(Twist());steer.publish(Float64(data=target));rclpy.spin_once(node,timeout_sec=.02)
            if samples[-1]['sim_time_s']<begin+1.5: raise RuntimeError('phase timeout')
            actual=samples[-1];goal=mapping.central(target,**geometry)
            phase=dict(command_rad=target,sim_time_s=actual['sim_time_s'],
                expected_left_rad=goal.left_rad,expected_right_rad=goal.right_rad,
                actual_left_rad=actual['left_steering_rad'],actual_right_rad=actual['right_steering_rad'],
                model=actual.get('steering_model'),cad_relative_target_rad=actual.get('cad_relative_target_rad'),
                target_error_rad=abs(actual.get('cad_relative_target_rad',99)-goal.servo_rad),
                maximum_settled_error_rad=max(abs(actual['left_steering_rad']-goal.left_rad),abs(actual['right_steering_rad']-goal.right_rad)))
            phases.append(phase)
        speed.publish(Twist());steer.publish(Float64(data=0.))
    except Exception as error:
        errors.append(str(error))
    finally:
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        cleanup=[]
        for proc in reversed(processes):
            forced=False
            if proc.poll() is None: os.killpg(proc.pid,signal.SIGINT)
            try: proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                forced=True;os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
            cleanup.append(dict(pid=proc.pid,exitcode=proc.returncode,forced=forced))
        for log in logs: log.close()
    passed=(not errors and len(phases)==7 and all(p['model']=='fusion_cad_2026_11_13' and
        p['target_error_rad']<1e-10 and p['maximum_settled_error_rad']<.01 for p in phases) and
        all(not p['forced'] for p in cleanup))
    report=dict(passed=passed,phases=phases,errors=errors,cleanup=cleanup,samples=len(samples),
                scope='fixed chassis, no gravity/contact, zero motor command, simulation component only; not autonomous driving or loaded hardware steering')
    (out/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    (out/'samples.jsonl').write_text(''.join(json.dumps(row)+'\n' for row in samples))
    print(json.dumps(report))
    return 0 if passed else 1

if __name__=='__main__': raise SystemExit(main())
