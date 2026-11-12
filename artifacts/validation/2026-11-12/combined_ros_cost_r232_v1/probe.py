"""동결 계획기 프로세스+부모 재검사와 ROS 합성 IMU의 수신 경합. 구동 없음."""
from pathlib import Path
import argparse,csv,hashlib,json,multiprocessing as mp,os,sys,time
from copy import copy
ROOT=Path(__file__).resolve().parents[4]; A=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
from publisher_source import publisher,TOPIC,quantiles

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--label',required=True);parser.add_argument('--limit',type=int)
    args=parser.parse_args();assert args.label.isidentifier()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='198':
        raise ValueError('explicit localhost diagnostic domain198 required')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from arena_autonomy.decision_process import DecisionProcess,make_controller
    from arena_autonomy.follow_trace import restore_call,snapshot_call,freeze,equal_output,comparable_decision
    from arena_autonomy.scene_motion import motion_scene
    from arena_autonomy import guard_observation_policy as policy
    from arena_autonomy.lidar_motion import MotionUnavailable
    target=A/args.label;target.mkdir(exist_ok=False)
    manifest=json.loads((A/'bundle_manifest.json').read_bytes())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    unchanged=lambda:all(sha(ROOT/name)==info['sha256'] for name,info in manifest['files'].items())
    assert unchanged()
    trace=ROOT/manifest['trace'];events=[json.loads(line) for line in trace.open()]
    assert all(e['seq']==i for i,e in enumerate(events))
    assert events[-1]['kind']=='footer'
    footer=events[-1]['data'];assert footer['worker_alive'] is False
    assert all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    assert footer['accepted']==footer['written']==len(events)-1
    header=events[0]['data'];owner,motion=make_controller(header)
    calls=[];resets=[]
    for event in events[1:]:
        if event['kind']=='reset':resets.append(event['data'].get('reason'))
        if event['kind']!='call':continue
        data=event['data'];current=data.get('current_rebase')
        calls.append((event['seq'],resets,data,restore_call(current['input'],motion) if current else None))
        resets=[]
    if args.limit is not None:
        assert 0<args.limit<=len(calls);calls=calls[:args.limit]
    worker=DecisionProcess(header)
    context=mp.get_context('spawn');stop=context.Event();parent,child=context.Pipe()
    pub=context.Process(target=publisher,args=(stop,child,str(target/'sent.csv')),name='r232_diagnostic_imu')
    rx=[];latencies=[];ticks=[];work=[];answers=[];fault=None
    state=dict(start=None,index=0,active=False,finished=False)
    ref0=calls[0][2]['input']['reference_s']
    def tick():
        if state['start'] is None or state['finished']:return
        ticks.append(time.monotonic())
        result=worker.pump()
        if worker.fault:raise RuntimeError(worker.fault)
        if result is not None:
            seq,_,data,current=calls[state['index']]
            before=time.perf_counter();answer=None
            if current is not None:
                info,scan,history=current
                snapshot=snapshot_call(scan,history,info['reference_s'],info['epoch'],info['speed_mps'],
                    info['side'],info['previous_steering_rad'],info['dt_s'],info['timing_verified'],info['rear_half_angle_deg'])
                try:
                    scene=motion_scene(scan,history,info['reference_s'],timing_verified=info['timing_verified'],
                        epoch=info['epoch'],rear_half_angle_deg=info['rear_half_angle_deg'])
                    answer=policy.rebase(result['output'],data['input'],snapshot,scene,history,owner.config)
                except (MotionUnavailable,ValueError) as e:
                    answer=dict(speed_mps=0.,steering_rad=info['previous_steering_rad'],reference_s=info['reference_s'],
                        reason='rebase_input_stop',input_error=str(e),independent_guard_required=True,runtime_authorized=False)
                answer=freeze(answer)
            ended=time.perf_counter()
            work.append(dict(seq=seq,child_ms=1000*(result['completed_wall_s']-result['began_wall_s']),
                recheck_ms=1000*(ended-before),has_current=current is not None,
                completed_lateness_ms=1000*(time.monotonic()-state['start']-(data['input']['reference_s']-ref0))))
            answers.append((seq,result['output'],answer))
            state['active']=False;state['index']+=1
            if state['index']==len(calls):state['finished']=True;return
        if not state['active']:
            _,reasons,data,_=calls[state['index']]
            if time.monotonic()<state['start']+data['input']['reference_s']-ref0:return
            for reason in reasons:
                worker.invalidate()
                if reason=='decision_source_expired':worker.mark_source_expiry()
            worker.offer(data['input']);state['active']=True
    rclpy.init(args=[]);node=Node('r232_diagnostic_receiver')
    def imu(msg):
        now=node.get_clock().now().nanoseconds
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        rx.append((int(msg.angular_velocity.x),stamp,now));latencies.append((now-stamp)/1e6)
    qos=copy(qos_profile_sensor_data);qos.depth=64
    node.create_subscription(Imu,TOPIC,imu,qos);node.create_timer(.02,tick)
    executor=SingleThreadedExecutor(context=node.context);executor.add_node(node)
    started=time.monotonic();pub.start();child.close()
    try:
        while not parent.poll():
            if time.monotonic()-started>20:raise RuntimeError('diagnostic publisher readiness timeout')
            executor.spin_once(timeout_sec=.01)
        assert parent.recv()=={'ready':True};state['start']=time.monotonic()
        while not state['finished']:
            executor.spin_once(timeout_sec=.01)
            if time.monotonic()-started>240:raise RuntimeError('bounded diagnostic deadline')
    except Exception as e:fault=type(e).__name__+': '+str(e)
    finally:
        state['finished']=True;stop.set();deadline=time.monotonic()+.3
        while time.monotonic()<deadline:executor.spin_once(timeout_sec=.01)
        pub.join(timeout=3.);forced=pub.is_alive()
        if forced:pub.terminate();pub.join(timeout=2.)
        cleanup=worker.close()
        executor.remove_node(node);executor.shutdown();node.destroy_node();rclpy.shutdown();parent.close()
    differences=[];commands=[]
    for (seq,out,current),(_,_,data,_) in zip(answers,calls):
        actual=comparable_decision(out);expected=comparable_decision(data['output'])
        if not equal_output(actual,expected):differences.append(dict(seq=seq,stage='planner',actual=actual,expected=expected))
        if not equal_output(actual[:2],expected[:2]):commands.append(dict(seq=seq,stage='planner'))
        if current is not None:
            expected=data['current_rebase']['output']
            if not equal_output(current,expected):differences.append(dict(seq=seq,stage='current',actual=current,expected=expected))
            keys=('speed_mps','steering_rad','reason','reference_s')
            if not equal_output([current.get(k) for k in keys],[expected.get(k) for k in keys]):commands.append(dict(seq=seq,stage='current'))
    sent=list(csv.DictReader((target/'sent.csv').open()))
    tx={int(r['seq']) for r in sent};ids={r[0] for r in rx}
    with (target/'received.csv').open('x',newline='') as f:
        writer=csv.writer(f);writer.writerow(['seq','stamp_ns','received_ns']);writer.writerows(rx)
    (target/'differences.json').write_text(json.dumps(differences,indent=2)+'\n')
    (target/'durations.json').write_text(json.dumps(work,indent=2)+'\n')
    report=dict(calls=len(answers),expected_calls=len(calls),fault=fault,source_unchanged=unchanged(),
        full_difference_count=len(differences),command_differences=commands,
        sent=len(sent),received=len(rx),missing=len(tx-ids),unexpected=len(ids-tx),duplicates=len(rx)-len(ids),
        callback_latency_ms=quantiles(latencies),source_gap_ms=quantiles([(b[1]-a[1])/1e6 for a,b in zip(rx,rx[1:])]),
        parent_tick_ms=quantiles([1000*(b-a) for a,b in zip(ticks,ticks[1:])]),
        child_ms=quantiles([r['child_ms'] for r in work]),current_ms=quantiles([r['recheck_ms'] for r in work if r['has_current']]),
        recorded_schedule_lateness_ms=quantiles([r['completed_lateness_ms'] for r in work]),
        publisher_exitcode=pub.exitcode,publisher_forced=forced,cleanup=cleanup,elapsed_s=time.monotonic()-started,
        imu_history_queue_depth=64,
        scope='recorded planner process and current recheck with synthetic200Hz DDS, localhost198; no delivery/expiry deadline enforcement, no live sensors/guard/vision/actuation')
    report['execution_complete']=not fault and len(answers)==len(calls) and not forced and pub.exitcode==0 and report['source_unchanged']
    (target/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report),flush=True)
    return 0 if report['execution_complete'] and not commands else 1

if __name__=='__main__':raise SystemExit(main())
