"""G1 시험기에 선택적으로 붙이는 추가 actor 실행·회수 경계.

기존 주 actor와 제어기의 구독/출력 경로는 건드리지 않는다. ROS 의존성은 실행 시만
읽는다. source/topic별 원시 기록·ACK·누락은 독립적으로 검사한다.
"""
import json
import math
import subprocess
import time

import g1_traffic_eval as ev
from traffic_actors import ActorRecords, audit_actor, contact_events


class ExtraActors:
    def __init__(self, specs, start_positions, centerline, pose_at, output, began):
        self.specs = specs
        self.starts = dict(zip((a['model_name'] for a in specs), start_positions))
        self.line, self.pose_at, self.output, self.began = centerline, pose_at, output, began
        self.records = ActorRecords(specs)
        self.contacts = []
        self.pending = {a['model_name']: None for a in specs}
        self.last_command = {a['model_name']: None for a in specs}
        self.readers, self.reader_audit, self.connections, self.subscribers = {}, {}, {}, {}
        self.subscriptions = []

    @property
    def ready(self):
        return all(self.records.poses[a['model_name']] for a in self.specs)

    @property
    def has_pending(self):
        return any(f is not None for f in self.pending.values())

    def bridge_arguments(self):
        args, params = [], []
        for spec in self.specs:
            topic = f"/model/{spec['model_name']}/pose"
            args.append(f'{topic}@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V')
            params.extend(['-p', f'qos_overrides.{topic}.publisher.reliability:=reliable',
                           '-p', f'qos_overrides.{topic}.publisher.depth:=200'])
        return args, params

    def connect(self, node, qos, json_reader, capture_gate, log, process_readers):
        from tf2_msgs.msg import TFMessage
        for spec in self.specs:
            name = spec['model_name']
            def receive(message, expected=name):
                for transform in message.transforms:
                    if transform.child_frame_id != expected:
                        self.records.invalid_frames.append([transform.header.frame_id, transform.child_frame_id])
                        continue
                    q, p = transform.transform.rotation, transform.transform.translation
                    when = transform.header.stamp.sec+transform.header.stamp.nanosec*1e-9
                    self.records.pose(expected, transform.header.frame_id,
                        (when, p.x, p.y, math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
            self.subscriptions.append(node.create_subscription(TFMessage, f'/model/{name}/pose', receive, qos))
            audit = {'json_messages': 0, 'handled_messages': 0, 'parse_errors': 0,
                     'handler_errors': 0, 'last_error': None}
            self.reader_audit[name] = audit
            def contact(data, source=name, topic=spec['contact_topic']):
                stamp = data.get('header', {}).get('stamp', {})
                self.contacts.append({'actor': source, 'topic': topic,
                    't': float(stamp.get('sec', 0))+float(stamp.get('nsec', 0))*1e-9,
                    'pairs': [[c['collision1']['name'], c['collision2']['name']] for c in data.get('contact', [])]})
            reader = json_reader(['gz', 'topic', '-e', '-t', spec['contact_topic'], '--json-output'],
                                 capture_gate.wrap(contact), log, audit)
            self.readers[name] = reader
            process_readers.append(reader)

    def complete(self):
        for name, future in tuple(self.pending.items()):
            if future is None:
                continue
            row = self.records.commands[name][-1]
            if future.done():
                response = future.result()
                row.update(response_wall_s=time.monotonic()-self.began,
                           acknowledged=response is not None and response.success)
                self.pending[name] = None
                if not row['acknowledged']:
                    raise RuntimeError(f'actor set_pose not acknowledged: {name}')
            elif time.monotonic()-row['requested_wall_absolute_s'] > 8:
                raise TimeoutError(f'actor set_pose timeout: {name}')

    def tick(self, client, request_type, now, green):
        self.complete()
        if green is None:
            return
        for spec in self.specs:
            name = spec['model_name']
            last = self.last_command[name]
            if (not spec['schedule'] or self.pending[name] is not None
                    or (last is not None and now-last < 1/spec['command_hz'])):
                continue
            x, y, yaw = self.pose_at(self.line, self.starts[name], spec['schedule'], now-green, spec['lateral_offset_m'])
            request = request_type()
            request.entity.name, request.entity.type = name, request.entity.MODEL
            request.pose.position.x, request.pose.position.y = x, y
            request.pose.position.z = spec['bottom_m']+spec['height_m']/2
            request.pose.orientation.z, request.pose.orientation.w = math.sin(yaw/2), math.cos(yaw/2)
            self.pending[name] = client.call_async(request)
            self.records.commands[name].append({'requested_sim_s': now, 'elapsed_s': now-green,
                'x_m': x, 'y_m': y, 'requested_wall_s': time.monotonic()-self.began,
                'requested_wall_absolute_s': time.monotonic(), 'acknowledged': False})
            self.last_command[name] = now

    def graph_audit(self, node):
        for spec in self.specs:
            name = spec['model_name']
            self.subscribers[name] = sorted(i.node_name for i in node.get_subscriptions_info_by_topic(f'/model/{name}/pose'))
            result = subprocess.run(['gz', 'topic', '-i', '-t', spec['contact_topic']],
                                    capture_output=True, text=True, timeout=10)
            self.connections[name] = {'topic': spec['contact_topic'], 'returncode': result.returncode,
                'output': result.stdout, 'stderr': result.stderr,
                'reader_alive_at_graph_audit': self.readers[name].poll() is None}

    def save_and_evaluate(self, *, ego, green, duration, config, runtime_world, known_contact_source, digest):
        if not self.specs:
            return None
        raw = self.output/'additional_actors.json'
        raw.write_text(json.dumps({'poses': self.records.poses, 'commands': self.records.commands,
            'contacts': self.contacts}, ensure_ascii=False), encoding='utf-8')
        result = {'raw_sha256': digest(raw), 'actor_count': len(self.specs),
                  'actors': {}, 'harness_valid': False, 'behavior_gates_passed': False,
                  'rejected_pose_times': self.records.rejected_times, 'invalid_frames': self.records.invalid_frames}
        if green is None or not ego:
            result['error'] = 'missing green or ego stream'
            return result
        end = green+duration
        span = ev.window_tuples(ego, green-1., ego[-1][0])
        contact_summary = contact_events(self.contacts, self.specs)
        result['contacts'] = contact_summary
        for spec in self.specs:
            name = spec['model_name']
            poses, commands = self.records.poses[name], self.records.commands[name]
            def schedule(t):
                return self.pose_at(self.line, self.starts[name], spec['schedule'], min(t, end)-green,
                                    spec['lateral_offset_m'])[:2]
            audit = audit_actor(spec, poses, commands, green, end, schedule, runtime_world, config['acceptance']['harness'])
            audit['checks'].update(
                through_stop_coverage=ev.stream_coverage([r[0] for r in poses], green-1., ego[-1][0], ev.POSE_MAX_GAP_S)['valid'],
                evaluator_only=self.subscribers.get(name) == ['g1b_traffic_validator'],
                contact_source=known_contact_source and ev.contact_connection_valid(self.connections.get(name, {})),
                reader_clean=all(self.reader_audit.get(name, {}).get(k, 1) == 0 for k in ('parse_errors', 'handler_errors')),
                no_pending_command=self.pending[name] is None)
            audit['valid'] = all(audit['checks'].values())
            sampled, swept = [], []
            if poses and ev._pose_rows_valid(poses) and ev._pose_rows_valid(span):
                sampled = ev.sampled_gaps(span, poses, spec['length_m'], spec['width_m'], ev.POSE_MAX_GAP_S)
                swept = ev.swept_gap_lower_bounds(span, poses, spec['length_m'], spec['width_m'],
                    1/spec['pose_publish_hz'], config['acceptance']['swept_max_curvature_per_m'], ev.POSE_MAX_GAP_S)
            distances = [r['gap_m'] for r in sampled if r['gap_m'] is not None]
            lower = [r['lower_bound_m'] for r in swept if r['lower_bound_m'] is not None]
            threshold = spec.get('minimum_body_gap_m')
            gates = {
                'registered_gap': isinstance(threshold, (float, int)) and math.isfinite(threshold) and threshold >= 0,
                'complete_gap_samples': bool(sampled) and len(distances) == len(sampled),
                'complete_swept_intervals': bool(swept) and len(lower) == len(swept),
                'no_ego_contact': not any(name in [p.split('::')[0] for p in e['pair']]
                    and 'arena_car' in [p.split('::')[0] for p in e['pair']] for e in contact_summary['events']),
            }
            gates['sampled_gap'] = gates['registered_gap'] and bool(distances) and min(distances) >= threshold
            gates['swept_gap'] = gates['registered_gap'] and bool(lower) and min(lower) >= threshold
            result['actors'][name] = {'harness': audit, 'gates': gates,
                'minimum_body_gap_m': threshold, 'min_sampled_gap_m': min(distances, default=None),
                'min_swept_lower_bound_m': min(lower, default=None), 'pose_samples': len(poses),
                'command_count': len(commands), 'contact_connection': self.connections.get(name)}
            if spec.get('route_kind') == 'side_road_straight':
                from traffic_encounter import encounter_metrics
                if spec['length_m'] != .20 or spec['width_m'] != .15:
                    raise ValueError('encounter diagnosis currently requires 20x15 cm actor')
                encounter = encounter_metrics(ego, poses, self.line, green, end, minimum_gap_m=threshold)
                result['actors'][name]['encounter'] = encounter
                audit['checks']['encounter_stream'] = encounter['valid']
                audit['valid'] = all(audit['checks'].values())
        result['harness_valid'] = (contact_summary['valid'] and not self.records.invalid_frames
                                  and all(a['harness']['valid'] for a in result['actors'].values()))
        result['behavior_gates_passed'] = all(all(a['gates'].values()) for a in result['actors'].values())
        result['scope'] = 'additional actor stimulus and per-actor evidence; not interacting vehicle dynamics or on-road passing proof'
        return result
