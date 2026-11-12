"""현재 CAD 추정 카메라의 공식 6개 그리드 시야·가림·RGB 검출 검사.

정적 평가 전용. 실제 차량/주행/출발 허가를 실행하지 않는다. 외부 차량은
200x150 mm 불투명 외곽 상자이며 높이는 가정값이다. 정답은 평가에만 쓴다.
"""
import argparse
import copy
import hashlib
import importlib.util
import itertools
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import time
import threading
import selectors
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy')]
from audit_grid_start import camera_model
from audit_start_signal_geometry import rotation, pose
from arena_autonomy.core import image_rgb, marker_observations
from arena_autonomy.race_start_local import LocalOfficialStartSignal
from arena_autonomy.race_start_marker import MarkerAnchoredStartSignal, marker_geometry, late_green
from lidar_shutdown import audit_shutdown, group_processes, stop_gazebo_service, stop_launch
from validate_start_signal_render import cleanup_owned_residue, stop_bridge_first

BASE = ROOT/'artifacts/validation/2026-11-07/track_migration_v1/live_signal_render.py'
spec = importlib.util.spec_from_file_location('base_render', BASE)
base = importlib.util.module_from_spec(spec)
spec.loader.exec_module(base)
PROFILE = ROOT/'src/arena_description/config/b0390_c1_cad_20261112_frontwire_4mm.yaml'
MOUNT = yaml.safe_load(PROFILE.read_text())['camera']
WORLD = base.WORLD_DIR
SLOTS = json.loads((WORLD/'scene.json').read_text())['starting_grid']['slots']
SUPPORT = ROOT/'artifacts/validation/2026-11-12/signal_post_revision_r291_v1/supports_trial.sdf'
LEVELS = [('empty', 0.), ('same224', .224), ('partial230', .230), ('partial240', .240), ('partial250', .250), ('partial280', .280), ('tall350', .350)]
POSE_CLIENT = None


def save(path, data):
    path.write_text(json.dumps(data, indent=2, ensure_ascii=False, default=str)+'\n', encoding='utf-8')


def camera_pose(slot, dz=0., dx=0., pitch=0., yaw=0.):
    basis = rotation(0, 0, slot['yaw_rad'])
    position = np.array([slot['x'], slot['y'], .003])+basis@(np.array(MOUNT['xyz_m'])+[dx, 0, dz])
    return position, basis@rotation(0, -math.radians(pitch), math.radians(yaw))


def targets(head_shift=0.):
    model = ET.parse(WORLD/'world.sdf').getroot().find("world/model[@name='it_arena_track_static']")
    result = []
    for link in model.findall('link'):
        name = link.get('name')
        if not (name.startswith('lamp_') or name in ('aruco_0', 'aruco_45')):
            continue
        xyz, basis = pose(link)
        if name.startswith('lamp_'):
            xyz[2] += head_shift
            radius = float(link.findtext('visual/geometry/sphere/radius'))
            # 투영 경계와 광선 표본을 구분한다. 구의 외접 상자는 화각 평가에만 쓴다.
            bounds = xyz+np.array(list(itertools.product((-radius, radius), repeat=3)))
            points = [xyz]+[xyz+[x, 0, z] for x, z in ((-radius,0),(radius,0),(0,-radius),(0,radius))]
        else:
            size = np.array(list(map(float, link.findtext('visual/geometry/box/size').split())))
            bounds = xyz+(basis@np.array(list(itertools.product((size[0]/2,), (-size[1]/2,size[1]/2), (-size[2]/2,size[2]/2)))).T).T
            points = [xyz+basis@np.array([size[0]/2, y, z])
                      for y in np.linspace(-size[1]/2,size[1]/2,11)
                      for z in np.linspace(-size[2]/2,size[2]/2,11)]
        result.append(dict(name=name, xyz=xyz, bounds=bounds, samples=np.array(points)))
    return result


def ray_box(origin, target, slot, height):
    """선분과 수직 차량 외곽의 slab 교차. 지주/자기 차체 가림은 여기에는 미포함."""
    if height <= 0:
        return False
    basis = rotation(0, 0, slot['yaw_rad']).T
    center = np.array([slot['x'], slot['y'], .003+height/2])
    start = basis@(origin-center)
    delta = basis@(target-origin)
    half = np.array([.1, .075, height/2])
    enter, leave = 0., 1.
    for a,d,h in zip(start,delta,half):
        if abs(d)<1e-12:
            if abs(a)>h:
                return False
        else:
            near,far=sorted(((-h-a)/d,(h-a)/d))
            enter,leave=max(enter,near),min(leave,far)
    return leave>=enter and leave>0 and enter<1


def geometry():
    f=MOUNT['width_px']/(2*math.tan(math.radians(MOUNT['horizontal_fov_deg'])/2))
    all_rows=[]
    for shift in (0.,-.009):
        for slot in SLOTS:
            origin,basis=camera_pose(slot)
            rows=[]
            for target in targets(shift):
                p=(basis.T@(target['bounds']-origin).T).T
                uv=np.column_stack((640-f*p[:,1]/p[:,0],360-f*p[:,2]/p[:,0]))
                clearance=[]
                for dz,dx,pitch,yaw in itertools.product((-.01,.01),(-.02,.02),(-3.,3.),(-3.,3.)):
                    o,b=camera_pose(slot,dz,dx,pitch,yaw)
                    q=(b.T@(target['bounds']-o).T).T
                    v=np.column_stack((640-f*q[:,1]/q[:,0],360-f*q[:,2]/q[:,0]))
                    clearance.append(float(min(v[:,0].min(),v[:,1].min(),1280-v[:,0].max(),720-v[:,1].max())))
                visible={name:int(sum(not any(ray_box(origin,p,other,h) for other in SLOTS
                                               if other['index']!=slot['index']) for p in target['samples']))
                         for name,h in LEVELS}
                rows.append(dict(name=target['name'],bounds_px=[*uv.min(axis=0),*uv.max(axis=0)],
                                 min_margin_perturbed_px=min(clearance),samples=len(target['samples']),visible_samples=visible))
            all_rows.append(dict(slot=slot['index'],head_shift_m=shift,camera_world_xyz=origin.tolist(),targets=rows))
    result=dict(camera=MOUNT,body_footprint_m=[.2,.15],cases=LEVELS,
                perturbation_assumptions=dict(height_m=.01,longitudinal_m=.02,pitch_deg=3,yaw_deg=3),
                limitations='pinhole nominal FOV; projected samples not full visibility proof; no own CAD body or lens calibration',rows=all_rows)
    save(HERE/'geometry.json', result)
    print('geometry: '+json.dumps([dict(slot=r['slot'],shift=r['head_shift_m'],
          lamps={n:sum(t['visible_samples'][n]==t['samples'] for t in r['targets'] if t['name'].startswith('lamp')) for n,_ in LEVELS},
          minimum_margin_px=min(t['min_margin_perturbed_px'] for t in r['targets'])) for r in all_rows]),flush=True)


def build_fixture(path, shift):
    root=ET.parse(WORLD/'world.sdf').getroot();world=root.find('world')
    for p in list(world.findall('plugin')):
        if p.get('name')=='gz::sim::systems::Imu':world.remove(p)
    for element in world.iter():
        if element.tag in ('albedo_map','uri') and element.text and '://' not in element.text:
            asset=(WORLD/element.text).resolve();assert asset.is_file(),asset;element.text=str(asset)
    model=world.find("model[@name='it_arena_track_static']")
    for link in model.findall('link'):
        if link.get('name')=='tl_panel' or link.get('name').startswith('lamp_'):
            p=list(map(float,link.findtext('pose').split()));p[2]+=shift;link.find('pose').text=' '.join(map(str,p))
    support=ET.parse(SUPPORT).getroot().find("model[@name='eval_20261112_signal_support']")
    world.append(copy.deepcopy(support))
    views={}
    for slot in SLOTS:
        name='slot'+str(slot['index']);xyz,_=camera_pose(slot)
        views[name]=camera_model(world,name,xyz,[0,0,slot['yaw_rad']],MOUNT)
        sensor=world.find(f"model[@name='m5_camera_{name}']/link/sensor")
        sensor.find('update_rate').text='10'
        body=ET.SubElement(world,'model',name='occluder_'+str(slot['index']))
        ET.SubElement(body,'static').text='true'
        ET.SubElement(body,'pose').text=f"{50+slot['index']} 50 .178 0 0 {slot['yaw_rad']}"
        visual=ET.SubElement(ET.SubElement(body,'link',name='body'),'visual',name='body')
        ET.SubElement(ET.SubElement(ET.SubElement(visual,'geometry'),'box'),'size').text='.2 .15 .35'
        material=ET.SubElement(visual,'material')
        for tag in ('ambient','diffuse'):ET.SubElement(material,tag).text='.16 .18 .22 1'
    ET.indent(root);ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)
    return views


def set_body(slot,height,park):
    x,y,z=(50+slot['index'],50,.178) if park else (slot['x'],slot['y'],.003+height-.175)
    request=f'name: "occluder_{slot["index"]}", position: {{x: {x}, y: {y}, z: {z}}}, orientation: {{z: {math.sin(slot["yaw_rad"]/2)}, w: {math.cos(slot["yaw_rad"]/2)}}}'
    POSE_CLIENT.stdin.write(request+'\n');POSE_CLIENT.stdin.flush()
    with selectors.DefaultSelector() as poll:
        poll.register(POSE_CLIENT.stdout,selectors.EVENT_READ)
        if not poll.select(5):raise TimeoutError('persistent pose response missing')
    response=json.loads(POSE_CLIENT.stdout.readline())
    if not all(response.get(k) for k in ('parsed','executed','service_result','applied')):raise RuntimeError('actor placement: '+str(response))
    return dict(slot=slot['index'],xyz=[x,y,z],response=response)


def detect(frames):
    normal=LocalOfficialStartSignal();anchored=MarkerAnchoredStartSignal();late=MarkerAnchoredStartSignal()
    states=[]
    for frame in frames:
        rgb,stamp=frame['rgb'],frame['stamp'];observations=marker_observations(rgb,.02)
        normal.update(rgb,stamp);anchored.update(rgb,stamp,observations)
        if frame['stage']=='green':late.update(rgb,stamp,observations)
        _,reason=late_green(rgb,observations)
        states.append(dict(stage=frame['stage'],stamp=stamp,ids=sorted(set(int(m[0]) for m in observations)),
            corners=[dict(id=int(m[0]),corners=np.asarray(m[1]).tolist()) for m in observations],
            local=normal.observed,anchored=anchored.observed,late=late.observed,
            marker_geometry=marker_geometry(observations),late_reason=reason,
            local_started=normal.started,anchored_started=anchored.started,green_only_started=late.started))
    return dict(local_started=normal.started,anchored_started=anchored.started,green_only_started=late.started,
                off_false_start=any(x['local_started'] or x['anchored_started'] for x in states if x['stage']=='off'),frames=states)


def render(args):
    global POSE_CLIENT
    import rclpy
    from rclpy.qos import QoSProfile
    from sensor_msgs.msg import Image
    output=HERE/args.output;output.mkdir(exist_ok=False)
    fixture=output/'fixture.sdf';views=build_fixture(fixture,args.head_shift)
    launch_file=output/'fixture.launch.py';launch_file.write_text(base.launch_text(fixture,views))
    sources=[PROFILE,WORLD/'world.sdf',WORLD/'scene.json',SUPPORT,Path(__file__),BASE,
             ROOT/'src/arena_autonomy/arena_autonomy/race_start_marker.py',ROOT/'src/arena_autonomy/arena_autonomy/race_start_local.py',
             ROOT/'src/arena_autonomy/arena_autonomy/race_start.py',ROOT/'src/arena_autonomy/arena_autonomy/core.py']
    hashes={str(p.relative_to(ROOT)):hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    os.environ.update(GZ_PARTITION=f'arena_autonomy_test_{os.getpid()}',ROS_DOMAIN_ID=str(170+os.getpid()%20),ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',ROS_STATIC_PEERS='')
    report=dict(scope='static 6-grid camera rendering, no driving or operational start; opaque body envelopes',head_shift_m=args.head_shift,
                effective_camera=MOUNT,source_sha256=hashes,cases=[])
    rclpy.init();node=rclpy.create_node('r307_grid_camera');latest={name:None for name in views}
    capture=dict(name=None,stage='off',frames=[])
    def receive(msg,name):
        stamp=msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        if latest[name] is None or stamp>latest[name]['stamp']:
            latest[name]=dict(stamp=stamp,rgb=image_rgb(msg).copy())
            if name==capture['name']:capture['frames'].append(dict(stage=capture['stage'],**latest[name]))
    subs=[node.create_subscription(Image,v['topic'],lambda m,n=name:receive(m,n),QoSProfile(depth=1)) for name,v in views.items()]
    receiver_stop=threading.Event()
    def receive_loop():
        while not receiver_stop.is_set():rclpy.spin_once(node,timeout_sec=.02)
    # r307 v2의 배경 수신 첫 프레임 미도달을 보존하고 원 주 스레드 spin으로 복귀.
    log_path=output/'launch.log';log=log_path.open('wb');process=client=None
    deadline=time.monotonic()+args.timeout
    def spin():
        if process.poll() is not None or time.monotonic()>deadline:raise TimeoutError('render timeout or server exit')
        rclpy.spin_once(node,timeout_sec=.02)
    try:
        process=subprocess.Popen(['ros2','launch',str(launch_file)],cwd=output,stdin=subprocess.DEVNULL,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        while any(v is None for v in latest.values()):spin()
        POSE_CLIENT=subprocess.Popen([str(ROOT/'build/r307_pose_client'),'/world/it_arena_track/set_pose'],
                                      stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        client=base.load_controller().LampClient(str(ROOT/'install/arena_gazebo/lib/arena_gazebo/visual_material_client'),'/world/it_arena_track/visual_config/blocking')
        for label,height in LEVELS:
            if args.levels and label not in args.levels.split(','):continue
            for slot in SLOTS:
                if args.slots and str(slot['index']) not in args.slots.split(','):continue
                name='slot'+str(slot['index']);case=f'{label}_{name}'
                receipts=[set_body(s,height,height==0 or s['index']==slot['index']) for s in SLOTS]
                # Flush queued frames and let the scene update. Camera poses are fixed in SDF.
                time.sleep(.03)
                floor=latest[name]['stamp']+.4
                while latest[name]['stamp']<floor:spin()
                capture.update(name=name,stage='off',frames=[])
                frames=[];lamps=[]
                for stage in ('off','red4','green'):
                    capture['stage']=stage
                    applied=client.apply(stage);lamps.append(dict(stage=stage,applied=all(r['applied'] for r in applied)))
                    if not lamps[-1]['applied']:raise RuntimeError('lamp stage not applied')
                    floor=latest[name]['stamp']+.11;taken=0
                    while taken<5:
                        spin();value=latest[name]
                        if value['stamp']>=floor and (not frames or value['stamp']>frames[-1]['stamp']):
                            frames.append(dict(stage=stage,**value));taken+=1
                    cv2.imwrite(str(output/(case+'_'+stage+'.png')),cv2.cvtColor(frames[-1]['rgb'],cv2.COLOR_RGB2BGR))
                capture['name']=None
                result=detect(frames)
                continuous=detect(capture['frames'])
                report['cases'].append(dict(name=case,slot=slot['index'],height_m=height,placement=receipts,lamps=lamps,
                    continuous_detection=continuous,continuous_max_gap_s=max((b['stamp']-a['stamp'] for a,b in zip(capture['frames'],capture['frames'][1:])),default=0.),**result))
                save(output/'report.partial.json',report)
                print(json.dumps(dict(case=case,local=result['local_started'],anchored=result['anchored_started'],late=result['green_only_started'],ids=result['frames'][-1]['ids'])),flush=True)
        report['capture_complete']=True
    except Exception as error:
        report['error']=repr(error)
    finally:
        if client:client.close()
        if POSE_CLIENT:
            POSE_CLIENT.stdin.close()
            try:POSE_CLIENT.wait(timeout=4)
            except subprocess.TimeoutExpired:POSE_CLIENT.terminate();POSE_CLIENT.wait(timeout=3)
        receiver_stop.set()
        node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        before=group_processes(process.pid) if process else {}
        if process:
            report['bridge_shutdown']=stop_bridge_first(process,log_path.read_text(errors='replace'))
            report['server_stop']=stop_gazebo_service(process,log_path.read_text(errors='replace'),os.environ['GZ_PARTITION'])
            cleanup=stop_launch(process);log.close()
            report['shutdown']=audit_shutdown(log_path.read_text(errors='replace'),process.poll(),cleanup or None)
            report['residue_cleanup']=cleanup_owned_residue(process.pid,before)
        else:log.close()
        report['source_unchanged']=all(hashlib.sha256((ROOT/p).read_bytes()).hexdigest()==h for p,h in hashes.items())
        report['passed']=bool(report.get('capture_complete') and report['source_unchanged'] and report.get('shutdown',{}).get('clean') and 'error' not in report)
        save(output/'report.json',report)
    print(json.dumps({k:report.get(k) for k in ('passed','capture_complete','error','source_unchanged')},ensure_ascii=False),flush=True)
    return 0 if report['passed'] else 1


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--geometry-only',action='store_true')
    parser.add_argument('--output',default='render_current')
    parser.add_argument('--head-shift',type=float,default=0.)
    parser.add_argument('--levels',default='')
    parser.add_argument('--slots',default='')
    parser.add_argument('--camera-height',type=float,default=None)
    parser.add_argument('--timeout',type=float,default=1200.)
    args=parser.parse_args()
    if args.camera_height is not None:MOUNT['xyz_m'][2]=args.camera_height
    if args.geometry_only:geometry()
    if not args.geometry_only:raise SystemExit(render(args))
