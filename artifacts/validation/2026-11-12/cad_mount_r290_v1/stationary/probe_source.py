"""선택형 CAD 장착의 실제 Gazebo 정지 수신. 이동 명령/실물 장치 없음."""
import json
import math
import os
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from arena_autonomy.local_path import mask_scan
from arena_autonomy.local_scene import blind_parameter
import capture_official_track_views as capture
from capture_rear_marker_views import camera_sdf
from lidar_shutdown import audit_shutdown, group_processes, stop_gazebo_service, stop_launch

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parent/'stationary'


def main():
    OUT.mkdir(exist_ok=False)
    os.environ.update(ROS_DOMAIN_ID=str(170+os.getpid()%50),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
                      ROS_STATIC_PEERS='',GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}')
    os.environ.setdefault('FASTDDS_BUILTIN_TRANSPORTS','LARGE_DATA')
    profile=ROOT/'src/arena_description/config/b0390_c1_cad_20261112_frontwire_4mm.yaml'
    world=ROOT/'src/arena_gazebo/worlds/it_arena_official_v2026_11_06'
    capture.WORLD_NAME=ET.parse(world/'world.sdf').getroot().find('world').get('name')
    slot=next(s for s in json.loads((world/'scene.json').read_text())['starting_grid']['slots'] if s['index']==3)
    command=['ros2','launch','--noninteractive','arena_bringup','simulation.launch.py',
        'headless:=true','track:=official_v2026_11_06','grid_slot:=3','autonomy:=false',
        'autonomy_mode:=local_pursuit','traffic_light:=false','render_sensors:=true',
        'tof_safety:=false','depth_camera:=false','lidar_acquisition:=snapshot',
        'render_backend:=wsl_nvidia','batch_static_visuals:=true','speed_bump:=false',
        'pilot_marker_z:=0.14',f'local_sensor_config:={profile}',f'runtime_artifacts:={OUT}']
    report=dict(scope='stationary Gazebo render/snapshot rays only; no drive or physical proof',command=command,passed=False)
    log=(OUT/'launch.log').open('w');simulation=None;bridge=None
    rclpy.init();node=rclpy.create_node('cad_mount_stationary_probe');state={'scans':[],'images':0,'odoms':[]}
    def scan(msg):
        if len(state['scans'])<20:state['scans'].append(msg)
    def odom(msg):
        state['odom']=msg
        state['odoms'].append([msg.pose.pose.position.x,msg.pose.pose.position.y,msg.pose.pose.position.z])
    def picture(msg):state.update(image=msg,images=state['images']+1)
    node.create_subscription(LaserScan,'/scan',scan,qos_profile_sensor_data)
    node.create_subscription(Odometry,'/odom',odom,10)
    node.create_subscription(Image,'/cad_mount/photo',picture,qos_profile_sensor_data)
    def wait(predicate):
        deadline=time.monotonic()+90
        while not predicate():
            if simulation.poll() is not None:raise RuntimeError('simulation exited before capture')
            if time.monotonic()>deadline:raise TimeoutError('stationary sensor timeout')
            rclpy.spin_once(node,timeout_sec=.05)
    try:
        simulation=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=log,stdin=subprocess.DEVNULL,start_new_session=True)
        wait(lambda:len(state['scans'])>=20 and 'odom' in state)
        view=dict(name='cad_mount',position=(-.36,.26,.42),target=(-.015,0.,.115),hfov=1.05)
        # 평가 자세는 외부 촬영 위치 확인에만 사용한다. 제어 노드는 실행하지 않는다.
        sdf=camera_sdf(view,slot,'/cad_mount/photo');(OUT/'photo.sdf').write_text(sdf)
        capture.gazebo_service('create','gz.msgs.EntityFactory','sdf: '+json.dumps(sdf))
        bridge=subprocess.Popen(['ros2','run','ros_gz_bridge','parameter_bridge',
            '/cad_mount/photo@sensor_msgs/msg/Image[gz.msgs.Image'],stdout=log,stderr=log,stdin=subprocess.DEVNULL,start_new_session=True)
        wait(lambda:state['images']>=3)
        assert cv2.imwrite(str(OUT/'cad_mount.png'),capture.image_array(state['image']))
        text=(OUT/'lidar_blind_sectors.json').read_text();spec=blind_parameter(0.,text)
        scans=state['scans'][5:]; rows=[]
        for message in scans:
            angles=message.angle_min+np.arange(len(message.ranges))*message.angle_increment
            distances=np.asarray(message.ranges);filtered=np.asarray(mask_scan(message,spec).ranges)
            sector_rows=[]
            for (center,half),(lo,hi) in zip(spec['sectors_deg'],spec['range_limits_m']):
                offset=(angles-math.radians(center)+math.pi)%math.tau-math.pi
                angular=np.abs(offset)<=math.radians(half)+1e-12
                own=angular&(distances>=lo)&(distances<=hi)
                sector_rows.append(int(sum(own)))
                assert not np.any(np.isfinite(filtered[own]))
            # 멀리 있는 트랙 반환은 자기 구조 마스크로 지우지 않는다.
            distant=np.isfinite(distances)&(distances>.3)
            assert np.array_equal(filtered[distant],distances[distant])
            rows.append(sector_rows)
        np.savez_compressed(OUT/'raw_scans.npz',ranges=np.array([s.ranges for s in scans]),angles=angles,
                            stamps=np.array([[s.header.stamp.sec,s.header.stamp.nanosec] for s in scans]))
        report.update(scans=len(scans),self_hits_per_sector=rows,all_sections_seen=bool(np.all(np.sum(rows,axis=0)>0)),
            scan_count=len(angles),sensor_range_m=[scans[-1].range_min,scans[-1].range_max],
            range_filtered=True,valid_distant_preserved=True,
            planar_displacement_m=float(np.linalg.norm(np.array(state['odoms'][-1][:2])-state['odoms'][0][:2])))
        assert report['all_sections_seen'], 'one or more CAD/marker sectors not represented in renderer'
    except Exception as error:report['error']=repr(error)
    finally:
        if bridge is not None:
            report['bridge_cleanup']=stop_launch(bridge)
            report['bridge_remaining']=list(group_processes(bridge.pid))
        if simulation is not None:
            report['server_stop']=stop_gazebo_service(simulation,(OUT/'launch.log').read_text(errors='replace'),os.environ['GZ_PARTITION'])
            cleanup=stop_launch(simulation)
            report['shutdown']=audit_shutdown((OUT/'launch.log').read_text(errors='replace'),simulation.returncode,cleanup.get('forced_cleanup'))
            report['remaining_pids']=list(group_processes(simulation.pid))
        node.destroy_node();rclpy.shutdown();log.close()
        report['passed']=bool(not report.get('error') and report.get('all_sections_seen') and report.get('server_stop',{}).get('verified')
            and report.get('shutdown',{}).get('clean') and not report.get('remaining_pids') and not report.get('bridge_remaining'))
        (OUT/'report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps({k:report.get(k) for k in ('passed','error','scans','all_sections_seen','planar_displacement_m','remaining_pids')}))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
