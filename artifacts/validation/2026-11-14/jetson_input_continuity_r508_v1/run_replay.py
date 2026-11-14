"""r500 고정 설치본, simulation-clock ROS 재구성 20초 단1회. 물리 포트/host 없음."""
from collections import Counter
from array import array
import gzip
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import threading
import urllib.request
import zipfile

from node_runner import PREFIX, TOPICS, ros_arguments, write

A = Path(__file__).resolve().parent
INSTALLED = Path('/home/q/it-arena-field-r500-20261114-a')
SHA = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()


def source_snapshot():
    manifest = json.loads((INSTALLED / 'source_manifest.json').read_bytes())
    assert SHA(INSTALLED / 'runtime_handover_sources.zip') == '73bebdcdc176eef04b641dc78c986b76ec32431b194c787cba0f351ad6f95965'
    with zipfile.ZipFile(INSTALLED / 'runtime_handover_sources.zip') as archive:
        assert archive.read('source_manifest.json') == (INSTALLED / 'source_manifest.json').read_bytes()
    changed = [p for p, h in manifest['files'].items() if SHA(INSTALLED / p) != h]
    assert not changed, changed
    installed = {str(p.relative_to(INSTALLED)): SHA(p) for p in (INSTALLED / 'install').rglob('*')
                 if p.is_file() and p.suffix in ('.py', '.so')}
    return dict(source_files=len(manifest['files']), installed=installed)


def main():
    assert os.environ.get('ROS_DOMAIN_ID') == '158' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    manifest = json.loads((A / 'manifest.json').read_bytes())
    assert all(SHA(A / name) == h for name, h in manifest['files'].items())
    before = source_snapshot()
    data = json.loads(gzip.decompress((A / 'reconstructed_inputs.json.gz').read_bytes()))
    out = A / 'result'
    out.mkdir(exist_ok=False)
    write(out / 'source_before.json', before)
    write(out / 'registration.json', manifest)
    import yaml
    profiles = json.loads((A / 'profile.json').read_bytes())
    producer_imu_frame=profiles['local_pursuit']['ros__parameters']['motion_imu_frame_id']
    assert producer_imu_frame==profiles['lidar_safety']['ros__parameters']['motion_imu_frame_id']
    assert producer_imu_frame=='r506_reconstructed_sim_imu','keep the frozen r506 input frame'
    profiles['local_pursuit']['ros__parameters']['g1_trace_path'] = str(out / 'controller_trace.jsonl')
    for name in ('local_pursuit','lidar_safety'):
        profiles[name]['ros__parameters'].update(timing_probe=True,timing_probe_mode='buffered',
            timing_probe_buffer_path=str(out/(name+'_timing.jsonl')),timing_probe_capacity=30000)
    profiles['race_ops'] = dict(ros__parameters=dict(use_sim_time=True, vehicle_id='r508_sim_only',
        host='127.0.0.1', port=0, token_path=str(out / 'test_token.txt'), record_path=str(out / 'ops_record.json'),
        log_path=str(out / 'ops_events.jsonl'), operation_mode='autonomous'))
    profiles = {PREFIX + '/' + name: body for name, body in profiles.items()}
    (out / 'profile.yaml').write_text(yaml.safe_dump(profiles, sort_keys=False), encoding='utf8')
    (out / 'test_token.txt').write_text('r508-local-test-token\n')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import Imu, JointState, Image
    from std_msgs.msg import String
    from rosgraph_msgs.msg import Clock
    from std_srvs.srv import SetBool
    from ackermann_msgs.msg import AckermannDriveStamped
    from audit_start_signal_layout import light_image
    interrupted = False
    def stop(*_):
        nonlocal interrupted
        interrupted = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Node('r508_reconstructed_input', namespace=PREFIX)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    pubs = dict(scan=node.create_publisher(String, PREFIX+'/scan/native', qos),
        gyro=node.create_publisher(Imu, PREFIX+'/imu/data', qos),
        wheels=node.create_publisher(JointState, PREFIX+'/drive_motor/encoder', qos),
        rgb=node.create_publisher(Image, PREFIX+'/camera/color/image_raw', 2),
        clock=node.create_publisher(Clock, PREFIX+'/clock', 20))
    commands, statuses, safety, http, published, lateness = [], [], [], [], Counter(), []
    wall_origin = sim_origin = None
    def simulation_now():
        return data['first_time'] if wall_origin is None else sim_origin + time.monotonic() - wall_origin
    def drive(message, topic):
        source = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        commands.append(dict(topic=topic, source_s=source, received_s=simulation_now(),
            received_wall_s=time.monotonic(), speed=message.drive.speed, steering=message.drive.steering_angle))
    def status(message, rows):
        rows.append(dict(received_s=simulation_now(), received_wall_s=time.monotonic(), value=json.loads(message.data)))
    for topic in ('/drive', '/drive/safe'):
        node.create_subscription(AckermannDriveStamped, PREFIX+topic, lambda m,t=topic:drive(m,t), 100)
    node.create_subscription(String, PREFIX+'/autonomy/status', lambda m:status(m,statuses), 100)
    node.create_subscription(String, PREFIX+'/safety/status', lambda m:status(m,safety), 100)
    stop_client = node.create_client(SetBool, PREFIX+'/autonomy/enable')
    observer_thread=threading.Thread(target=executor.spin,name='r508-read-only-observer',daemon=True)
    observer_thread.start()
    def clock(at):
        message = Clock()
        message.clock.sec, message.clock.nanosec = divmod(round(at*1e9), 10**9)
        pubs['clock'].publish(message)
    def stamp(message, at):
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(round(at*1e9), 10**9)
    started = time.monotonic()
    children, logs, forced, failure = {}, [], [], None
    def health():
        if interrupted:
            raise InterruptedError('requested stop')
        if time.monotonic() - started > 60:
            raise TimeoutError('60s bounded trial deadline')
        for role, child in children.items():
            if child.poll() is not None:
                raise RuntimeError(f'{role} exited {child.returncode}')
    def spin(seconds):
        until = time.monotonic() + seconds
        while time.monotonic() < until:
            health()
            clock(simulation_now())
            time.sleep(.002)
    try:
        spin(.4)
        assert set(node.get_node_names()) == {'r508_reconstructed_input'}, 'domain not empty'
        for role in ('controller', 'guard', 'ops'):
            log = (out / (role+'.log')).open('x')
            logs.append(log)
            children[role] = subprocess.Popen([sys.executable, '-B', str(A/'node_runner.py'), role,
                '--output', str(out)], stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        deadline = time.monotonic()+15
        while not all((out/(role+'_ready.json')).exists() for role in children):
            if time.monotonic()>deadline:
                raise TimeoutError('node startup')
            spin(.02)
        # Both sides of each input/output edge must exist before any sensor publication.
        minima = dict(scan=2, gyro=2, wheels=2, rgb=1, clock=3)
        deadline = time.monotonic()+5
        while not all(pub.get_subscription_count() >= minima[k] for k,pub in pubs.items()):
            if time.monotonic()>deadline:
                raise TimeoutError('input subscriptions')
            spin(.02)
        def output_owners():
            return {topic:[dict(name=o.node_name,namespace=o.node_namespace)
                for o in node.get_publishers_info_by_topic(PREFIX+topic)]
                for topic in ('/drive','/drive/safe')}
        wanted={'/drive':[dict(name='local_pursuit',namespace=PREFIX)],
                '/drive/safe':[dict(name='lidar_safety',namespace=PREFIX)]}
        deadline=time.monotonic()+5
        while output_owners()!=wanted:
            if time.monotonic()>deadline:
                write(out/'output_discovery_failure.json',output_owners())
                raise TimeoutError('output publisher discovery: '+repr(output_owners()))
            spin(.02)
        graph = dict(topics=node.get_topic_names_and_types(),
            inputs={k:pub.get_subscription_count() for k,pub in pubs.items()},
            nodes=node.get_node_names_and_namespaces(),output_publishers=output_owners(),
            services=node.get_service_names_and_types())
        write(out/'graph.json',graph)
        contract_rows=[]
        for role in ('controller','guard'):
            ready=json.loads((out/(role+'_ready.json')).read_bytes())
            contract=ready['input_contract']
            assert ready['use_sim_time'] and contract['motion_verified']
            assert PREFIX+'/clock' in ready['subscriptions']
            assert contract['motion_time_policy']=='verified_only'
            assert contract['input_representation']=='native_simulation_v1'
            assert contract['imu_frame']==producer_imu_frame
            assert contract['feedback_mode']=='drive_motor_shaft' and contract['gear_ratio']==2.5*81/17
            for kind,topic,frame,basis in (
                ('gyro','/imu/data',producer_imu_frame,'original_simulation_stamp'),
                ('wheels','/drive_motor/encoder','r508_reconstructed_motor_velocity','original_simulation_stamp'),
                ('scan','/scan/native','laser_frame','simulation_acquisition')):
                assert PREFIX+topic in ready['subscriptions'],(role,kind,ready['subscriptions'])
                if kind=='scan':
                    assert contract['lidar_frame']==frame
                    assert all(e['value']['source_frame_id']==frame and e['value']['time_basis']==basis
                        for e in data['events'] if e['kind']=='scan')
                contract_rows.append(dict(consumer=role,kind=kind,topic=PREFIX+topic,
                    producer_frame=frame,time_basis=basis,contract=contract,
                    frame_validated=kind!='wheels'))
            if role=='controller':assert PREFIX+'/camera/color/image_raw' in ready['subscriptions']
        write(out/'input_contract_audit.json',dict(passed=True,rows=contract_rows,
            rgb=dict(topic=PREFIX+'/camera/color/image_raw',frame='r508_synthetic_signal_pixels',
                frame_validated=False,encoding='rgb8',size=[848,480],hz=20),use_sim_time=True))
        assert all(topic.startswith(PREFIX+'/') or topic in ('/rosout','/parameter_events') for topic,_ in graph['topics'])
        for topic, expected in (('/drive', 'local_pursuit'), ('/drive/safe', 'lidar_safety')):
            owners = node.get_publishers_info_by_topic(PREFIX+topic)
            assert len(owners)==1 and owners[0].node_name==expected and owners[0].node_namespace==PREFIX
        assert node.count_publishers('/drive')==node.count_publishers('/drive/safe')==0
        write(out/'graph.json',graph)
        ops = json.loads((out/'ops_ready.json').read_bytes())
        endpoint = 'http://127.0.0.1:'+str(ops['http_port'])+'/api/state?client=r508_probe'
        frames={}
        for stage in range(6):
            pixels=light_image(reds=stage if stage<5 else 0,green=stage==5,paired=False)
            message=Image();message.header.frame_id='r508_synthetic_signal_pixels'
            message.height,message.width=pixels.shape[:2];message.encoding='rgb8';message.step=message.width*3
            message.data=array('B',pixels.tobytes())
            frames[stage]=message
        wall_origin, sim_origin = time.monotonic(), data['first_time']
        write(out/'clock_mapping.json',dict(monotonic_origin_s=wall_origin,simulation_origin_s=sim_origin,
            monotonic_implementation=time.get_clock_info('monotonic').implementation,
            perf_counter_implementation=time.get_clock_info('perf_counter').implementation,
            producer_imu_frame=producer_imu_frame))
        events, index, next_image, next_http = data['events'], 0, sim_origin, sim_origin
        motion_gate_confirmed=False
        finish = data['last_reference']
        while simulation_now() < finish+1.:
            health()
            now = simulation_now()
            clock(now)
            if not motion_gate_confirmed and now>=data['first_reference']-.5:
                for role in ('controller','guard'):
                    evidence=json.loads((out/(role+'_motion_accepted.json')).read_bytes())
                    assert evidence['counts']['gyro']>0 and evidence['counts']['drive_motor']>0
                motion_gate_confirmed=True
                write(out/'pre_scan_motion_gate.json',dict(passed=True,simulation_s=now,
                    first_scan_reference_s=data['first_reference'],physical_output=False))
            while index<len(events) and events[index]['at']<=now:
                row = events[index]
                kind, at, value = row['kind'], row['at'], row['value']
                lateness.append(dict(kind=kind, at=at, late_ms=(time.monotonic()-wall_origin+sim_origin-at)*1000))
                if kind=='scan':
                    message = String(data=json.dumps(value, separators=(',', ':'), allow_nan=False))
                elif kind=='gyro':
                    message=Imu();stamp(message,at);message.header.frame_id=producer_imu_frame
                    message.angular_velocity.z=float(value)
                else:
                    message=JointState();stamp(message,at);message.header.frame_id='r508_reconstructed_motor_velocity'
                    message.name=['drive_motor_shaft'];message.position=[0.]
                    message.velocity=[float(value)/.0325*(2.5*81/17)]
                pubs[kind].publish(message);published[kind]+=1;index+=1
            if now<finish and now>=next_image:
                # Test stimulus only. The detector sees pixels; no signal state injection.
                elapsed=now-sim_origin
                stage=min(5,int(max(0.,elapsed)/.3))
                message=frames[stage];stamp(message,next_image)
                pubs['rgb'].publish(message);published['rgb']+=1
                next_image+=.05
                if next_image<now-.05:next_image=now+.05
            if now>=next_http:
                req=urllib.request.Request(endpoint,headers={'X-Arena-Token':'r508-local-test-token','X-Arena-Vehicle':'r508_sim_only'})
                began=time.monotonic()
                with urllib.request.urlopen(req,timeout=.5) as response:
                    value=json.loads(response.read())
                http.append(dict(at_s=now, request_ms=(time.monotonic()-began)*1000, state=value))
                next_http=now+.5
            time.sleep(.001)
        assert index==len(events)
        future=stop_client.call_async(SetBool.Request(data=False))
        deadline=time.monotonic()+2
        while not future.done():
            if time.monotonic()>deadline:raise TimeoutError('STOP service')
            spin(.01)
        assert future.result().success
        spin(.3)
        write(out/'stop.json',dict(service_success=True, message=future.result().message,
            tail_commands=commands[-15:], physical_stop_verified=False))
    except BaseException as error:
        failure=type(error).__name__+': '+str(error)
        raise
    finally:
        for role in ('controller','guard','ops'):
            child=children.get(role)
            if child and child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:child.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    forced.append(role);os.killpg(child.pid,signal.SIGKILL);child.wait(timeout=2)
        executor.shutdown(timeout_sec=1);observer_thread.join(timeout=1)
        assert not observer_thread.is_alive(),'observer did not stop'
        node.destroy_node();rclpy.shutdown()
        for log in logs:log.close()
        for name, rows in (('commands',commands),('autonomy_status',statuses),('guard_status',safety),('http',http),('publication_lateness',lateness)):
            with gzip.open(out/(name+'.jsonl.gz'),'wt',encoding='utf8') as stream:
                for row in rows:stream.write(json.dumps(row,allow_nan=False)+'\n')
        after=source_snapshot()
        residual=[]
        for role,child in children.items():
            try:os.killpg(child.pid,0);residual.append(role)
            except ProcessLookupError:pass
        write(out/'execution.json',dict(error=failure, elapsed_wall_s=time.monotonic()-started,
            children={r:dict(pid=c.pid,exitcode=c.returncode) for r,c in children.items()},
            forced=forced,residual_groups=residual,source_unchanged=after==before,
            source_files=after['source_files'],installed_files=len(after['installed']),published=dict(published),
            physical_output=False,physical_ports_opened=False,ros_domain=158,namespace=PREFIX))
        assert after==before and not residual and not forced
    subprocess.run([sys.executable,'-B',str(A/'analyze.py'),str(out)],check=True)


if __name__ == '__main__':
    main()
