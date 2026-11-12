"""ROS 200Hz 합성 IMU 수신과 같은610판단의 경합 비교. 구동 출력 없음."""
import argparse
import csv
import hashlib
import json
import multiprocessing as mp
import os
from pathlib import Path
import signal
import sys
import time

ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
TOPIC='/arena_probe_r235/imu'


def publisher(stop, ready, path):
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    rclpy.init(args=[])
    node=Node('r100_publisher')
    pub=node.create_publisher(Imu,TOPIC,qos_profile_sensor_data)
    deadline=time.monotonic()+15.
    try:
        while pub.get_subscription_count()==0:
            if stop.is_set() or time.monotonic()>=deadline:raise RuntimeError('no diagnostic subscriber')
            time.sleep(.01)
        sequence=0;next_tick=time.monotonic()
        ready.send({'ready':True})
        with Path(path).open('x',newline='') as stream:
            out=csv.writer(stream);out.writerow(['seq','stamp_ns','monotonic_s'])
            while not stop.is_set():
                remaining=next_tick-time.monotonic()
                if remaining>0:time.sleep(min(remaining,.005));continue
                now=time.monotonic();next_tick=now+.005
                msg=Imu();msg.header.stamp=node.get_clock().now().to_msg()
                msg.header.frame_id='diagnostic_only'
                # 진단 전용 순번. MotionInput/차량 토픽에는 연결하지 않는다.
                msg.angular_velocity.x=float(sequence)
                pub.publish(msg)
                out.writerow([sequence,msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,now])
                sequence+=1
        ready.send({'sent':sequence})
    finally:
        node.destroy_node();rclpy.shutdown();ready.close()


def quantiles(values):
    import numpy as np
    return dict(count=len(values),p50=float(np.percentile(values,50)),
        p95=float(np.percentile(values,95)),maximum=max(values)) if values else dict(count=0)


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from arena_autonomy.decision_process import DecisionProcess,make_controller
    from arena_autonomy.follow_trace import replay_call,freeze,comparable_decision,equal_output
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode',required=True,choices=('inline','process'))
    parser.add_argument('--prefix',required=True)
    args=parser.parse_args()
    assert args.prefix.isidentifier()
    if os.environ.get('ROS_LOCALHOST_ONLY')!='1' or os.environ.get('ROS_DOMAIN_ID')!='199':
        raise ValueError('explicit localhost diagnostic domain199 required')
    target=A/(args.prefix+'_'+args.mode)
    target.mkdir(exist_ok=False)
    manifest=json.loads((ROOT/'artifacts/validation/2026-11-11/control_10hz_cost_r98_v1/bundle_manifest.json').read_bytes())
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    unchanged=lambda:all(sha(ROOT/n)==v['sha256'] for n,v in manifest['files'].items())
    assert unchanged()
    extra_hashes={str(p.relative_to(ROOT)):sha(p) for p in
        (Path(__file__),ROOT/'src/arena_autonomy/arena_autonomy/decision_process.py')}
    trace=ROOT/manifest['trace'];trace_hash=sha(trace)
    calls=[];generation=0;footer=None;count=0
    with trace.open(encoding='utf8') as stream:
        for line in stream:
            event=json.loads(line)
            assert footer is None and event['seq']==count
            count+=1
            if event['kind']=='header':header=event['data']
            elif event['kind']=='reset':generation+=1
            elif event['kind']=='call':calls.append((event['seq'],generation,event['data']))
            elif event['kind']=='footer':footer=event['data']
    assert footer and footer.get('worker_alive') is False and all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending'))
    assert footer['written']==footer['accepted']==count-1 and len(calls)==610
    context=mp.get_context('spawn');stop=context.Event();parent,child=context.Pipe()
    pub=context.Process(target=publisher,args=(stop,child,str(target/'sent.csv')),name='arena-ingress-publisher')
    received=[];delays=[];compute=[];mismatches=[];commands=[];fault=None;cleanup=None
    worker=DecisionProcess(header) if args.mode=='process' else None
    local,motion=make_controller(header)
    state={'index':0,'active':False,'generation':None,'running':False,'finished':False}
    def accept(seq, data, result):
        actual=comparable_decision(result);expected=comparable_decision(data['output'])
        if not equal_output(actual,expected):mismatches.append(seq)
        if not equal_output(actual[:2],expected[:2]):commands.append(seq)
        state['index']+=1;state['active']=False
        if state['index']==len(calls):state['finished']=True
    def tick():
        if not state['running'] or state['finished'] or state['active']:return
        seq,gen,data=calls[state['index']]
        if state['generation']!=gen:
            if worker:worker.invalidate()
            else:local.reset()
            state['generation']=gen
        if worker:
            worker.offer(data['input']);state['active']=True
        else:
            began=time.monotonic();out=freeze(replay_call(local,data['input'],motion))
            compute.append((time.monotonic()-began)*1000.);accept(seq,data,out)
    def poll():
        if not worker or not state['running']:return
        result=worker.pump()
        if worker.fault:raise RuntimeError(worker.fault)
        if result is not None:
            seq,_,data=calls[state['index']]
            compute.append((result['completed_wall_s']-result['began_wall_s'])*1000.)
            accept(seq,data,result['output'])
    rclpy.init(args=[]);node=Node('r100_receiver')
    def imu(msg):
        now=node.get_clock().now().nanoseconds
        stamp=msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        received.append((int(msg.angular_velocity.x),stamp,now))
        delays.append((now-stamp)/1e6)
    node.create_subscription(Imu,TOPIC,imu,qos_profile_sensor_data)
    node.create_timer(.1,tick)
    if worker:node.create_timer(.002,poll)
    executor=SingleThreadedExecutor(context=node.context);executor.add_node(node)
    began=time.monotonic();pub.start();child.close()
    try:
        while not parent.poll():
            if time.monotonic()-began>20.:raise RuntimeError('publisher readiness deadline')
            executor.spin_once(timeout_sec=.01)
        assert parent.recv()=={'ready':True}
        state['running']=True
        while not state['finished']:
            executor.spin_once(timeout_sec=.01)
            if time.monotonic()-began>240.:raise RuntimeError('bounded diagnostic deadline')
    except Exception as error:
        fault=type(error).__name__+': '+str(error)
    finally:
        state['running']=False;stop.set()
        drain=time.monotonic()+.3
        while time.monotonic()<drain:executor.spin_once(timeout_sec=.01)
        pub.join(timeout=3.)
        publisher_forced=pub.is_alive()
        if publisher_forced:pub.terminate();pub.join(timeout=2.)
        if worker:cleanup=worker.close()
        executor.remove_node(node);executor.shutdown();node.destroy_node();rclpy.shutdown()
        parent.close()
    sent=list(csv.DictReader((target/'sent.csv').open())) if (target/'sent.csv').exists() else []
    with (target/'received.csv').open('x',newline='') as stream:
        out=csv.writer(stream);out.writerow(['seq','stamp_ns','received_ns']);out.writerows(received)
    ids={r[0] for r in received};sent_ids={int(r['seq']) for r in sent}
    gaps=[(b[1]-a[1])/1e6 for a,b in zip(received,received[1:])]
    sent_gaps=[(float(b['monotonic_s'])-float(a['monotonic_s']))*1000. for a,b in zip(sent,sent[1:])]
    report=dict(scope='synthetic200Hz ROS callback under actual recorded planning workload; no MotionInput/live sensors/actuation',
        mode=args.mode,calls=state['index'],full_mismatch_seqs=mismatches,command_difference_seqs=commands,
        sent=len(sent),received=len(received),missing=len(sent_ids-ids),unexpected=len(ids-sent_ids),
        duplicate=len(received)-len(ids),latency_ms=quantiles(delays),source_gap_ms=quantiles(gaps),
        sent_gap_ms=quantiles(sent_gaps),sent_hz=(1000./(sum(sent_gaps)/len(sent_gaps))) if sent_gaps else None,
        source_gaps_over_50ms=sum(g>50. for g in gaps),compute_ms=quantiles(compute),
        publisher_exitcode=pub.exitcode,publisher_forced=publisher_forced,cleanup=cleanup,fault=fault,
        input_unchanged=trace_hash==sha(trace),source_unchanged=unchanged() and all(sha(ROOT/n)==h for n,h in extra_hashes.items()),
        diagnostic_sources=extra_hashes,
        trace_sha256=trace_hash,elapsed_s=time.monotonic()-began)
    (target/'report.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in report.items() if k!='full_mismatch_seqs'}))
    return 0 if fault is None and not publisher_forced and pub.exitcode==0 and state['finished'] else 1

if __name__=='__main__':raise SystemExit(main())
