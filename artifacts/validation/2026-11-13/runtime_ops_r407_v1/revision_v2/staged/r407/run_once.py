"""실제 ROS/HTTP/새 PTY 1회. 센서 발행·실물 장치·양수 명령 없음."""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback
import tty
import urllib.request

from registration import DOMAIN,read
from fake_mcu import FakeMcu
from operator_receipt import accepted_reply, policy_accepted, stopped_binding

HERE=Path(__file__).resolve().parent
ROOT=HERE.parent
TEST_SECONDS=45.


def write(path,value):
    path.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf8')


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args();out=args.output.resolve()
    assert out.parent==ROOT and not out.exists(),'new isolated result directory required'
    assert os.environ.get('ROS_DOMAIN_ID')==str(DOMAIN) and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    profile,bindings,registration=read(HERE)
    out.mkdir();write(out/'registration.json',dict(**registration,domain=DOMAIN,localhost_only=True,
        maximum_test_seconds=TEST_SECONDS,physical_ports_opened=False,physical_actuation=False,
        sensors_started=False,positive_drive_requested=False,synthetic_saved_race=True,
        protocol_arm_scope='explicit service on fresh PTY only; policy latch remains held'))
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from ackermann_msgs.msg import AckermannDriveStamped
    from arena_autonomy.g1_g2_node import G1G2Node
    from arena_autonomy.lidar_safety import LidarSafety
    started=time.monotonic();deadline=started+TEST_SECONDS
    node=guard=probe=executor=worker=vision=None;master=slave=None;fake=None;ros=False
    children={};logs=[];forced=[];cleanup_errors=[];checks={};failure=None;interrupted=None
    states=[];mcu=[];safety=[];drives=[];results=[];boundaries=[];race_commands=[];http=[]
    def stop_signal(number,_):
        nonlocal interrupted
        interrupted='signal_'+str(number)
    handlers={number:signal.signal(number,stop_signal) for number in (signal.SIGINT,signal.SIGTERM)}
    def launch(name,command):
        log=(out/(name+'.log')).open('x');logs.append(log)
        child=subprocess.Popen(command,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        children[name]=child;checks.setdefault('processes',{})[name]=dict(pid=child.pid,command=command)
        return child
    def pump_until(predicate,note,limit=3.):
        end=min(deadline,time.monotonic()+limit)
        while not predicate():
            if interrupted:raise InterruptedError(interrupted)
            if time.monotonic()>=end:raise TimeoutError(note)
            if fake and fake.error:raise RuntimeError('fake MCU: '+fake.error)
            for name,child in children.items():
                if child.poll() is not None:raise RuntimeError((name,'unexpected exit',child.returncode,note))
            executor.spin_once(timeout_sec=.005)
    def stop_child(name,timeout=2.):
        child=children.get(name)
        if child is None or child.poll() is not None:return
        child.send_signal(signal.SIGINT)
        try:child.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            forced.append(name);child.kill();child.wait(timeout=1.)
    endpoint=None
    def request(path,body=None):
        data=None if body is None else json.dumps(body).encode()
        req=urllib.request.Request(endpoint+path,data=data,headers={
            'X-Arena-Token':'r407-pty-only-test-token','X-Arena-Vehicle':'r407_pty_only',
            'Content-Type':'application/json'})
        with urllib.request.urlopen(req,timeout=.5) as response:
            value=json.loads(response.read())
            if body is not None:http.append(dict(request=body,response=value))
            return value
    def state():return request('/api/state?client=r407_probe')
    def command(op,identifier,*,bound_view=None,**fields):
        view=state()['binding'] if bound_view is None else bound_view
        return request('/api/command',dict(op=op,request=identifier,client='r407_probe',
            view=view,**fields))
    def entry(identifier):
        return next((c for c in state()['commands'] if c['request']==identifier),{})
    def policy(identifier):return policy_accepted(entry(identifier))
    def stop_evidence(identifier):
        observed=entry(identifier)
        step=next(s for s in observed['mcu_steps'] if s['step']=='stop' and s['success'])
        receipt=json.loads(step['message'])
        assert receipt['kind']=='STOP' and receipt['success'] and receipt['tx_written']
        assert receipt['ack']['request']==receipt['request'] and receipt['ack']['accepted']
        assert receipt['physical_stop_verified'] is False
        assert any(c['accepted'] and c['payload'].get('request')==receipt['request']
            and c['payload']['command']['kind']=='STOP' for c in fake.commands)
        return dict(operator_entry=observed,correlated_host_receipt=receipt)
    try:
        rosargs=['--ros-args','--params-file',str(HERE/'prepared/runtime_profile.yaml'),
            '-p','g1_trace_path:='+str(out/'trace.jsonl')]
        for source,target in bindings['ros_remappings']['local_pursuit']:
            rosargs+=['-r',source+':='+target]
        rclpy.init(args=rosargs,signal_handler_options=SignalHandlerOptions.NO);ros=True
        probe=Node('r407_pty_only_probe',use_global_arguments=False)
        executor=SingleThreadedExecutor();executor.add_node(probe)
        # Discovery window only; no command retry or clock/lease budget extension.
        discovery=time.monotonic()+.3
        while time.monotonic()<discovery:executor.spin_once(timeout_sec=.01)
        assert set(probe.get_node_names())=={'r407_pty_only_probe'},'domain is not exclusive'
        assert not any(n.startswith('/actuation/') for n,_ in probe.get_service_names_and_types())
        node=G1G2Node();guard=LidarSafety();worker=node.decision_delivery.worker
        vision=node.vision_pipeline.worker.process
        for owner in (node,guard):executor.add_node(owner)
        expected=json.loads((HERE/'reference/r401_expected_header.json').read_bytes())
        assert asdict(node.follow.config)==expected['follow_config']
        assert asdict(node.follow.execution)==expected['execution_config']
        assert asdict(node.motion.history.config)==asdict(guard.motion.history.config)==expected['motion_config']
        for name,owner in (('local_pursuit',node),('lidar_safety',guard)):
            for key,value in profile[name]['ros__parameters'].items():
                actual=owner.get_parameter(key).value
                assert type(actual) is type(value) and actual==value,(name,key,actual,value)
        assert node.race is not None and node.runtime_dual_selected
        assert node.motion.timing_policy==guard.motion.timing_policy=='host_estimated_v1'
        assert node.motion.verified is guard.motion.verified is False
        assert node.publisher.topic_name=='/drive' and guard.publisher.topic_name=='/drive/safe'
        checks['runtime_config_matches_r401']=True
        def receive_drive(message,topic):
            drives.append(dict(topic=topic,source_ns=message.header.stamp.sec*10**9+message.header.stamp.nanosec,
                speed=message.drive.speed,steering=message.drive.steering_angle))
        for topic,rows in (('/autonomy/status',states),('/actuation/status',mcu),('/safety/status',safety),
            ('/race/command_result',results),('/race/input_boundary',boundaries),('/race/command',race_commands)):
            probe.create_subscription(String,topic,lambda m,rows=rows:rows.append(json.loads(m.data)),20)
        for topic in ('/drive','/drive/safe'):
            probe.create_subscription(AckermannDriveStamped,topic,lambda m,topic=topic:receive_drive(m,topic),20)
        command_pub=probe.create_publisher(String,'/race/command',10)
        clients={name:probe.create_client(Trigger,'/actuation/'+name) for name in ('clear_stop','arm')}
        master,slave=os.openpty();tty.setraw(slave);port=os.ttyname(slave)
        assert port.startswith('/dev/pts/') and os.fstat(slave).st_rdev==os.stat(port).st_rdev
        checks['port']=dict(kind='fresh_PTY_fake_MCU',path=port,real_serial_device=False)
        fake=FakeMcu(master);fake.release_arm.set();fake.thread.start()
        host=launch('host',[sys.executable,'-B',str(ROOT/'scripts/run_runtime_actuation_host.py'),
            '--port',port,'--output',str(out/'host'),'--seconds',str(TEST_SECONDS),
            '--clock-policy',str(HERE/'prepared/clock_policy_pty.json'),
            '--max-speed-mps',str(bindings['host_arguments']['max_speed_mps']),
            '--max-steering-rad',str(bindings['host_arguments']['max_steering_rad']),
            '--imu-topic',bindings['host_arguments']['imu_topic'],'--enable-drive-wire'])
        # No --motor-feedback-config: fake stationary counts stay in protocol evidence,
        # never become /drive_motor/encoder or planner/guard motion samples.
        opsdir=out/'ops';opsdir.mkdir()
        (opsdir/'operator_token.txt').write_text('r407-pty-only-test-token\n')
        fixture=json.loads((HERE/'prepared/synthetic_saved_race.json').read_bytes())
        fixture['saved_wall_time']=time.time()
        write(out/'synthetic_saved_race_used.json',fixture);write(opsdir/'record.json',fixture)
        launch('ops',[sys.executable,'-B',str(HERE/'run_ops.py'),'--output',str(opsdir)])
        pump_until(lambda:(opsdir/'ready.json').is_file() and states and mcu and safety
            and all(c.service_is_ready() for c in clients.values()),'initial nodes/status/services',limit=12.)
        ready=json.loads((opsdir/'ready.json').read_bytes());assert ready['host']=='127.0.0.1'
        endpoint='http://127.0.0.1:'+str(ready['port'])
        pump_until(lambda:not state()['connection']['status_stale'] and
            state()['mcu'].get('state')=='ESTOP_LATCHED' and
            bool(states[-1]['race']['mcu']),'initial mapped MCU observation')
        initial=state();assert initial['race']['latch']=='process_start' and not initial['race']['started']
        assert initial['development']['limit_mps']==.3
        checks['initial_latched']=initial
        for topic in ('/drive','/drive/safe'):
            owners=probe.get_publishers_info_by_topic(topic)
            assert len(owners)==1 and owners[0].node_name==('local_pursuit' if topic=='/drive' else 'lidar_safety')
        assert probe.count_publishers('/drive_motor/encoder')==0
        for topic in ('/hardware_runtime/c1/native','/hardware_runtime/camera/image_raw'):
            assert probe.count_publishers(topic)==0
        checks['guard_only_safe_publisher']=True
        assert command('set_speed_limit','r407-limit',value=.2)['stage']=='queued'
        pump_until(lambda:policy('r407-limit') and state()['development']['limit_mps']==.2,'held speed cap')
        checks['held_speed_limit']=entry('r407-limit')
        assert state()['race']['latch']=='process_start'
        assert not any(c['payload']['command']['kind'] in ('CLEAR','ARM') for c in fake.commands)
        refused=command('manual_visual_start','r407-manual-http',confirm='SIGNAL_CONFIRMED')
        assert refused['stage']=='rejected_by_adapter' and refused['adapter_reason']=='wait_green_required'
        checks['manual_outside_wait_http']=refused
        # Same command interface as CLI; confirm the receiver also rejects a stale UI's claim.
        binding=state()['binding']
        manual=dict(v=1,request='r407-manual-node',op='manual_visual_start',signal_confirmed=True,
            **{k:binding[k] for k in ('session','race_epoch','intervention')})
        command_pub.publish(String(data=json.dumps(manual)))
        pump_until(lambda:any(r['request']=='r407-manual-node' for r in results),'manual node refusal')
        refusal=next(r for r in results if r['request']=='r407-manual-node')
        assert refusal['accepted'] is False and refusal['reason']=='wait_green_required'
        checks['manual_outside_wait_node']=refusal
        assert command('restore_race','r407-restore',confirm='SAME_RACE')['stage']=='queued'
        pump_until(lambda:policy('r407-restore') and state()['race']['started'],'synthetic record restoration')
        checks['synthetic_restore']=entry('r407-restore')
        assert state()['race']['start_source']=='restored_operator_confirmed'
        assert state()['race']['visual_start_fact'] is False
        # Explicit PTY-only service calls establish an armed protocol before the button STOP.
        # This does not clear the planner latch or provide any sensor/route data.
        for name in ('clear_stop','arm'):
            future=clients[name].call_async(Trigger.Request())
            pump_until(future.done,'PTY '+name)
            response=future.result();assert response.success,response.message
            checks['pty_service_'+name]=json.loads(response.message)
        pump_until(lambda:any(c['accepted'] and c['payload']['command']['kind']=='DRIVE' for c in fake.commands),
            'guard zero reaches PTY host',limit=2.)
        assert all(c['payload']['command'].get('speed_mps',0)==0 for c in fake.commands)
        stop_before=len(fake.commands)
        stop_request=command('stop','r407-stop');assert stop_request['stage']=='queued'
        stop_view=None
        def button_stop_ready():
            nonlocal stop_view
            current=state()
            observed=next((c for c in current['commands'] if c['request']=='r407-stop'),{})
            if not policy_accepted(observed):return False
            reply=accepted_reply(results,'r407-stop','stop')
            view=stopped_binding(current,reply,stop_request['view']['race_id'])
            if view is None:return False
            if not any(s['step']=='stop' and s['success'] for s in observed['mcu_steps']):return False
            if current['mcu']['state']!='ESTOP_LATCHED':return False
            stop_view=view
            checks['stop_binding_confirmed']=dict(reply=reply,binding=view,
                displayed_seq=current['seq'],displayed_latch=current['race']['latch'],
                displayed_sim_s=current['driving']['sim_s'],stop_request_view=stop_request['view'])
            return True
        pump_until(button_stop_ready,'button STOP receipt binding and mapped generation')
        assert any(c['accepted'] and c['payload']['command']['kind']=='STOP' for c in fake.commands[stop_before:])
        checks['button_stop']=stop_evidence('r407-stop')
        assert state()['development']['limit_mps']==.2
        arms_before=sum(c['payload']['command']['kind']=='ARM' for c in fake.commands)
        assert command('resume_start','r407-resume',bound_view=stop_view)['stage']=='queued'
        pump_until(lambda:policy('r407-resume') and (state()['race']['resume'] or {}).get('phase')=='inputs'
            and guard.input_floor_s==node.input_floor_s,'resume input boundary')
        observed_until=time.monotonic()+.4
        while time.monotonic()<observed_until:
            current=state();executor.spin_once(timeout_sec=.005)
            assert current['race']['latch']=='explicit_stop'
            assert (current['race']['resume'] or {}).get('phase')=='inputs'
        resume=state()
        assert set(resume['race']['resume']['missing'])=={'scan','rgb','gyro','encoder'}
        assert sum(c['payload']['command']['kind']=='ARM' for c in fake.commands)==arms_before==1
        assert not any(c['op'] in ('recovery_reset','new_race') for c in race_commands)
        assert node.race.permission.race_epoch==0 and not node.race.permission.last_visual_stamp
        assert not node.motion.history.wheels.values and not guard.motion.history.wheels.values
        checks['resume_without_reset']=dict(entry=entry('r407-resume'),state=resume,
            arm_count_before=arms_before,arm_count_after=arms_before,guard_input_floor_s=guard.input_floor_s,
            planner_input_floor_s=node.input_floor_s,physical_motion_permission=False)
        assert command('stop','r407-final-stop')['stage']=='queued'
        pump_until(lambda:policy('r407-final-stop') and any(s['step']=='stop' and s['success']
            for s in entry('r407-final-stop')['mcu_steps']) and state()['race']['resume'] is None,'final STOP')
        checks['final_button_stop']=stop_evidence('r407-final-stop')
        assert drives and all(row['speed']==0 for row in drives)
        assert Counter(row['topic'] for row in drives).keys()=={'/drive','/drive/safe'}
        checks['all_observed_commands_zero']=True
        checks['no_sensor_or_encoder_samples']=not node.scan and not guard.scan and not node.motion.history.wheels.values
        checks['policy_faults']=dict(runtime=worker.fault,vision=node.vision_pipeline.fault)
        assert not worker.fault and not node.vision_pipeline.fault
    except BaseException as error:
        failure=type(error).__name__+': '+str(error)
        (out/'traceback.txt').write_text(traceback.format_exc(),encoding='utf8')
    finally:
        # Stop adapter first so no pending operator operation can race host shutdown.
        try:stop_child('ops')
        except Exception as error:cleanup_errors.append('ops: '+repr(error))
        try:stop_child('host')
        except Exception as error:cleanup_errors.append('host: '+repr(error))
        if fake is not None:
            fake.stop.set()
            if fake.thread.ident is not None:fake.thread.join(timeout=.5)
        for fd in (master,slave):
            if fd is not None:
                try:os.close(fd)
                except Exception as error:cleanup_errors.append('PTY: '+repr(error))
        if executor is not None:
            try:executor.shutdown(timeout_sec=.5)
            except Exception as error:cleanup_errors.append('executor: '+repr(error))
        for owner in (node,guard,probe):
            if owner is not None:
                try:owner.destroy_node()
                except Exception as error:cleanup_errors.append('ROS node: '+repr(error))
        if ros:
            try:rclpy.shutdown()
            except Exception as error:cleanup_errors.append('ROS context: '+repr(error))
        for number,previous in handlers.items():signal.signal(number,previous)
        for log in logs:log.close()
        for name,rows in (('states',states),('mcu',mcu),('safety',safety),('drives',drives),('results',results),
            ('boundaries',boundaries),('race_commands',race_commands),('http',http),
            ('fake_mcu',[] if fake is None else fake.events)):
            with (out/(name+'.jsonl')).open('x',encoding='utf8') as stream:
                for row in rows:stream.write(json.dumps(row,ensure_ascii=False,allow_nan=False)+'\n')
        host_summary=None
        try:
            host_summary=json.loads((out/'host/summary.json').read_bytes())
            assert children['host'].returncode==1 and host_summary['interruption']=='signal_'+str(signal.SIGINT)
            assert host_summary['error'] is None and not host_summary['cleanup_errors']
            shutdown=host_summary['shutdown']
            assert shutdown['matching_stop_tx_receipt'] and shutdown['stop_ack_confirmed']
            assert any(c['accepted'] and c['payload'].get('request')==shutdown['request'] and
                c['payload']['command']['kind']=='STOP' for c in fake.commands)
            assert not host_summary['encoder_motion_published'] and not host_summary['source_timestamp_rewritten']
            events=[json.loads(line) for line in (out/'host/events.jsonl').read_text().splitlines()]
            transmitted=[e for e in events if e['event']=='drive_prepared' and e['prepared'].get('wire')]
            safe_sources={d['source_ns'] for d in drives if d['topic']=='/drive/safe'}
            matched=[e for e in transmitted if e['source']['source_ros_ns'] in safe_sources and any(
                c['payload']==e['prepared']['envelope'] and c['accepted'] for c in fake.commands)]
            assert matched and all(not e['source']['source_retimed'] for e in transmitted)
            checks['guard_source_preserved_to_host_wire']=dict(matched_count=len(matched),
                example_source_ns=matched[0]['source']['source_ros_ns'])
            assert children['ops'].returncode==0
            assert not worker.cleanup['forced'] and not worker.cleanup['alive'] and worker.cleanup['exitcode']==0
            assert not vision.is_alive() and vision.exitcode==0
            assert fake.error is None and not fake.thread.is_alive() and not forced
        except BaseException as error:cleanup_errors.append('completion evidence: '+repr(error))
        imports={};external=[]
        for name,module in sorted(sys.modules.items()):
            if name.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(module,'__file__',None):
                path=Path(module.__file__).resolve()
                if not path.is_relative_to(ROOT):external.append(dict(module=name,path=str(path)))
                else:imports[name]=dict(path=path.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        report=dict(passed=failure is None and not cleanup_errors and not external,error=failure,
            cleanup_errors=cleanup_errors,checks=checks,elapsed_s=time.monotonic()-started,
            children={name:child.returncode for name,child in children.items()},forced=forced,
            host_shutdown=None if host_summary is None else host_summary['shutdown'],
            host_complete=None if host_summary is None else host_summary['complete'],
            host_expected_signal_exit_separate_from_stop_receipt=True,
            worker_cleanup=None if worker is None else worker.cleanup,
            vision_exitcode=None if vision is None else vision.exitcode,imports=imports,external_project_imports=external,
            domain=DOMAIN,physical_actuation_verified=False,physical_stop_verified=False,
            real_sensor_inputs=False,synthetic_saved_race=True,synthetic_encoder_published=False,
            positive_drive=False,scope='B3 operating requests and PTY protocol only; no motion authorization')
        write(out/'report.json',report)
    print(json.dumps({k:v for k,v in report.items() if k not in ('checks','imports')},ensure_ascii=False))
    if not report['passed']:raise SystemExit(1)


if __name__=='__main__':main()
