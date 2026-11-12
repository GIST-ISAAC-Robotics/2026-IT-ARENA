"""r248 부모/동일 자식 재검사 비교. 저장 입력+합성 IMU만, 명령 발행 없음."""
from pathlib import Path
import argparse
import csv
import gc
import hashlib
import json
import multiprocessing as mp
import os
import sys
import time
from copy import copy

ROOT = Path(__file__).resolve().parents[4]
A = Path(__file__).resolve().parent
sys.path[:0] = [str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
from publisher_source import publisher, TOPIC, quantiles


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--label', required=True)
    parser.add_argument('--mode', required=True, choices=('parent', 'two_stage'))
    parser.add_argument('--limit', type=int)
    parser.add_argument('--numeric', required=True, choices=('python','native_v1'))
    args = parser.parse_args()
    assert args.label.isidentifier()
    assert args.mode == 'two_stage', 'this candidate tests completion notices only'
    if os.environ.get('ROS_LOCALHOST_ONLY') != '1' or os.environ.get('ROS_DOMAIN_ID') != '187':
        raise ValueError('explicit localhost diagnostic domain187 required')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Imu
    from arena_autonomy.decision_process import DecisionProcess, make_controller
    from arena_autonomy.follow_trace import restore_call, snapshot_call, freeze, equal_output, comparable_decision
    from arena_autonomy.scene_motion import motion_scene
    from arena_autonomy import guard_observation_policy as policy
    from arena_autonomy.lidar_motion import MotionUnavailable
    from arena_autonomy.recheck_process import TwoStageProcess
    from arena_autonomy.completion_notice import CompletionNotice

    target = A/args.label
    target.mkdir(exist_ok=False)
    manifest = json.loads((A/'bundle_manifest.json').read_bytes())
    sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
    unchanged = lambda: all(sha(ROOT/name) == info['sha256'] for name, info in manifest['files'].items())
    assert unchanged()
    trace = ROOT/manifest['trace']
    assert sha(trace) == manifest['trace_sha256']
    events = [json.loads(line) for line in trace.open()]
    assert all(event['seq'] == i for i, event in enumerate(events))
    assert events[-1]['kind'] == 'footer'
    footer = events[-1]['data']
    assert footer['worker_alive'] is False
    assert all(footer[k] == 0 for k in ('capture_errors', 'failed', 'dropped', 'pending'))
    assert footer['accepted'] == footer['written'] == len(events)-1
    header = events[0]['data']
    header['numeric_execution'] = args.numeric
    owner, motion = make_controller(header)
    calls, resets = [], []
    for event in events[1:]:
        if event['kind'] == 'reset':
            resets.append(event['data'].get('reason'))
        if event['kind'] != 'call':
            continue
        data = event['data']
        current = data.get('current_rebase')
        calls.append((event['seq'], resets, data, restore_call(current['input'], motion) if current else None))
        resets = []
    if args.limit is not None:
        assert 0 < args.limit <= len(calls)
        calls = calls[:args.limit]
    worker = (DecisionProcess if args.mode == 'parent' else TwoStageProcess)(header)
    context = mp.get_context('spawn')
    stop = context.Event()
    parent, child = context.Pipe()
    pub = context.Process(target=publisher, args=(stop, child, str(target/'sent.csv')), name='r248_diagnostic_imu')
    rx, latencies, ticks, callback_durations, work, answers = [], [], [], [], [], []
    fault = None
    notice = None
    gc_started, gc_events = [], []
    def gc_trace(phase, info):
        if phase == 'start':
            gc_started.append((time.monotonic(), info['generation']))
        elif gc_started:
            began, generation = gc_started.pop()
            gc_events.append((began, time.monotonic(), generation))
    gc.callbacks.append(gc_trace)
    state = dict(start=None, index=0, active=False, finished=False, stage='plan', plan=None)
    ref0 = calls[0][2]['input']['reference_s']

    def complete(answer, current_ms, parent_ms, end_to_end_ms):
        seq, _, data, current = calls[state['index']]
        plan = state['plan']
        work.append(dict(seq=seq, child_ms=1000*(plan['completed_wall_s']-plan['began_wall_s']),
            recheck_ms=current_ms, parent_recheck_ms=parent_ms, recheck_end_to_end_ms=end_to_end_ms,
            has_current=current is not None,
            completed_lateness_ms=1000*(time.monotonic()-state['start']-(data['input']['reference_s']-ref0))))
        answers.append((seq, plan['output'], answer))
        state['active'] = False
        state['index'] += 1
        if state['index'] == len(calls):
            state['finished'] = True

    def work_tick():
        if state['start'] is None or state['finished']:
            return
        pump_start = time.perf_counter()
        result = worker.pump()
        if notice is not None:
            notice.acknowledge()
            if notice.fault:
                raise RuntimeError(notice.fault)
        pump_ms = 1000*(time.perf_counter()-pump_start)
        if worker.fault:
            raise RuntimeError(worker.fault)
        if result is not None:
            if state['stage'] == 'recheck':
                assert result['stage'] == 'recheck'
                complete(result['output'], 1000*(result['completed_wall_s']-result['began_wall_s']),
                    state['snapshot_parent_ms']+pump_ms, 1000*(time.perf_counter()-state['snapshot_started']))
            else:
                seq, _, data, current = calls[state['index']]
                state['plan'] = result
                if current is None:
                    complete(None, 0., 0., 0.)
                else:
                    before = time.perf_counter()
                    info, scan, history = current
                    snapshot = snapshot_call(scan, history, info['reference_s'], info['epoch'], info['speed_mps'],
                        info['side'], info['previous_steering_rad'], info['dt_s'], info['timing_verified'], info['rear_half_angle_deg'])
                    if args.mode == 'two_stage':
                        worker.offer_recheck(snapshot, result)
                        assert worker.pump() is None
                        state['stage'] = 'recheck'
                        state['snapshot_started'] = before
                        state['snapshot_parent_ms'] = 1000*(time.perf_counter()-before)
                    else:
                        try:
                            scene = motion_scene(scan, history, info['reference_s'], timing_verified=info['timing_verified'],
                                epoch=info['epoch'], rear_half_angle_deg=info['rear_half_angle_deg'])
                            answer = policy.rebase(result['output'], data['input'], snapshot, scene, history, owner.config)
                        except (MotionUnavailable, ValueError) as error:
                            answer = dict(speed_mps=0., steering_rad=info['previous_steering_rad'],
                                reference_s=info['reference_s'], reason='rebase_input_stop', input_error=str(error),
                                independent_guard_required=True, runtime_authorized=False)
                        answer = freeze(answer)
                        spent_ms = 1000*(time.perf_counter()-before)
                        complete(answer, spent_ms, spent_ms, spent_ms)
        if not state['active'] and not state['finished']:
            _, reasons, data, _ = calls[state['index']]
            if time.monotonic() < state['start']+data['input']['reference_s']-ref0:
                return
            for reason in reasons:
                worker.invalidate()
                if reason == 'decision_source_expired':
                    worker.mark_source_expiry()
            worker.offer(data['input'])
            state['active'], state['stage'] = True, 'plan'

    def tick(kind='timer'):
        if kind == 'timer' and state['start'] is not None and not state['finished']:
            ticks.append(time.monotonic())
        started = time.perf_counter()
        try:
            work_tick()
        finally:
            callback_durations.append(1000*(time.perf_counter()-started))

    rclpy.init(args=[])
    node = Node('r248_diagnostic_receiver')
    def imu(msg):
        now = node.get_clock().now().nanoseconds
        stamp = msg.header.stamp.sec*10**9+msg.header.stamp.nanosec
        rx.append((int(msg.angular_velocity.x), stamp, now, time.monotonic()))
        latencies.append((now-stamp)/1e6)
    qos = copy(qos_profile_sensor_data)
    qos.depth = 64
    node.create_subscription(Imu, TOPIC, imu, qos)
    node.create_timer(.02, tick)
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    guard = node.create_guard_condition(lambda: tick('ready'))
    notice = CompletionNotice(worker.connection, guard.trigger)
    started = time.monotonic()
    pub.start()
    child.close()
    try:
        while not parent.poll():
            if time.monotonic()-started > 20:
                raise RuntimeError('diagnostic publisher readiness timeout')
            executor.spin_once(timeout_sec=.01)
        assert parent.recv() == {'ready': True}
        state['start'] = time.monotonic()
        while not state['finished']:
            executor.spin_once(timeout_sec=.01)
            if time.monotonic()-started > 240:
                raise RuntimeError('bounded diagnostic deadline')
    except Exception as error:
        fault = type(error).__name__+': '+str(error)
    finally:
        state['finished'] = True
        stop.set()
        deadline = time.monotonic()+.3
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=.01)
        pub.join(timeout=3.)
        forced = pub.is_alive()
        if forced:
            pub.terminate()
            pub.join(timeout=2.)
        notice_cleanup = notice.close()
        cleanup = worker.close()
        node.destroy_guard_condition(guard)
        gc.callbacks.remove(gc_trace)
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
        parent.close()

    differences, commands = [], []
    for (seq, output, current), (_, _, data, _) in zip(answers, calls):
        actual, expected = comparable_decision(output), comparable_decision(data['output'])
        if not equal_output(actual, expected):
            differences.append(dict(seq=seq, stage='planner', actual=actual, expected=expected))
        if not equal_output(actual[:2], expected[:2]):
            commands.append(dict(seq=seq, stage='planner'))
        if current is not None:
            expected = data['current_rebase']['output']
            if not equal_output(current, expected):
                differences.append(dict(seq=seq, stage='current', actual=current, expected=expected))
            keys = ('speed_mps', 'steering_rad', 'reason', 'reference_s')
            if not equal_output([current.get(k) for k in keys], [expected.get(k) for k in keys]):
                commands.append(dict(seq=seq, stage='current'))
    sent = list(csv.DictReader((target/'sent.csv').open()))
    tx, ids = {int(row['seq']) for row in sent}, {row[0] for row in rx}
    with (target/'received.csv').open('x', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['seq', 'stamp_ns', 'received_ns', 'received_monotonic_s'])
        writer.writerows(rx)
    (target/'differences.json').write_text(json.dumps(differences, indent=2)+'\n')
    (target/'durations.json').write_text(json.dumps(work, indent=2)+'\n')
    (target/'answers.json').write_text(json.dumps(answers, separators=(',', ':'))+'\n')
    long_delays = []
    for row, delay in zip(rx, latencies):
        if delay < 30:
            continue
        receipt = row[3]
        overlaps = [(start, end, generation) for start, end, generation in gc_events
                    if start <= receipt and end >= receipt-delay/1000.]
        long_delays.append(dict(seq=row[0], delay_ms=delay, gc_overlap=overlaps))
    (target/'gc_events.json').write_text(json.dumps(gc_events, indent=2)+'\n')
    (target/'long_delays.json').write_text(json.dumps(long_delays, indent=2)+'\n')
    report = dict(completion_notice=notice_cleanup, gc_events=len(gc_events),
        gc_generation2_ms=[1000*(end-start) for start,end,generation in gc_events if generation==2],
        delays_30ms_or_more=len(long_delays), delays_30ms_without_gc=sum(not row['gc_overlap'] for row in long_delays),
        numeric_execution=args.numeric, mode=args.mode, calls=len(answers), expected_calls=len(calls), fault=fault, source_unchanged=unchanged(),
        full_difference_count=len(differences), command_differences=commands,
        sent=len(sent), received=len(rx), missing=len(tx-ids), unexpected=len(ids-tx), duplicates=len(rx)-len(ids),
        callback_latency_ms=quantiles(latencies), source_gap_ms=quantiles([(b[1]-a[1])/1e6 for a,b in zip(rx,rx[1:])]),
        parent_tick_ms=quantiles([1000*(b-a) for a,b in zip(ticks,ticks[1:])]),
        parent_callback_ms=quantiles(callback_durations),
        child_ms=quantiles([row['child_ms'] for row in work]),
        current_ms=quantiles([row['recheck_ms'] for row in work if row['has_current']]),
        parent_recheck_ms=quantiles([row['parent_recheck_ms'] for row in work if row['has_current']]),
        recheck_end_to_end_ms=quantiles([row['recheck_end_to_end_ms'] for row in work if row['has_current']]),
        recorded_schedule_lateness_ms=quantiles([row['completed_lateness_ms'] for row in work]),
        publisher_exitcode=pub.exitcode, publisher_forced=forced, cleanup=cleanup, worker_stats=worker.stats,
        elapsed_s=time.monotonic()-started, imu_history_queue_depth=64,
        scope='same saved input, synthetic200Hz DDS localhost187; no delivery/expiry enforcement, live sensors, guard, vision or actuation')
    report['execution_complete'] = not fault and not notice_cleanup['alive'] and not notice_cleanup['fault'] and len(answers)==len(calls) and not forced and pub.exitcode==0 and report['source_unchanged']
    (target/'report.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps(report), flush=True)
    return 0 if report['execution_complete'] and not commands else 1


if __name__ == '__main__':
    raise SystemExit(main())
