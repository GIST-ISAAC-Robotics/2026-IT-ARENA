"""평가 전용 paused 시작 관문. 연결 준비와 실제 첫 물리 갱신은 별도로 감사한다."""
import os
import re
import subprocess
import time

TRACE_TOPIC = '/model/arena_car/eval/command_trace'
STATS_TOPIC = '/world/it_arena_track/stats'
# 같은 토픽으로 네 바퀴의 contact 센서가 각각 advertise한다. 제어 발행자는 여전히 하나다.
EXPECTED_GZ_PUBLISHERS = {TRACE_TOPIC: 1, '/eval/wheels/contact': 4}


def topic_connections(text):
    """gz topic -i의 주소 행만 센다. 알 수 없는/빈 출력은 준비로 인정하지 않는다."""
    counts = {'publishers': 0, 'subscribers': 0}
    section = None
    for line in text.splitlines():
        if line.startswith('Publishers ['):
            section = 'publishers'
        elif line.startswith('Subscribers ['):
            section = 'subscribers'
        elif line.strip().startswith(('tcp://', 'udp://', 'ipc://')) and section:
            counts[section] += 1
        elif line.strip():
            section = None
    return counts


def startup_arguments(mode, grid, trace):
    if mode == 'service':
        return []
    if mode != 'embedded_paused' or not grid or not trace:
        raise ValueError('embedded_paused requires sim_grid and plugin command trace')
    return ['evaluation_startup:=embedded_paused']


def stats_zero_paused(stats):
    try:
        sim = stats.get('sim_time', stats.get('simTime'))
        return (stats.get('paused') is True and int(stats.get('iterations', 0)) == 0
                and isinstance(sim, dict) and int(sim.get('sec', 0)) == 0 and int(sim.get('nsec', 0)) == 0)
    except (TypeError, ValueError):
        return False


def graph_snapshot(node, arm_client, stop_client):
    return {
        'publishers': {topic: node.count_publishers(topic) for topic in
                       ('/clock', '/sim/drivetrain', '/actuation/status', '/actuation/output_status',
                        '/drive_motor/encoder', '/scan', '/imu/data')},
        'drive_subscribers': sorted(i.node_name for i in node.get_subscriptions_info_by_topic('/drive')),
        'safe_publishers': sorted(i.node_name for i in node.get_publishers_info_by_topic('/drive/safe')),
        'arm_service': arm_client.service_is_ready(), 'stop_service': stop_client.service_is_ready(),
    }


def graph_ready(graph):
    return (bool(graph['publishers']) and all(v == 1 for v in graph['publishers'].values())
            and graph['drive_subscribers'] == ['lidar_safety']
            and graph['safe_publishers'] == ['lidar_safety']
            and graph['arm_service'] and graph['stop_service'])


class PausedStartup:
    """최대 90초 discovery 대기. 고정 sleep 대신 관측된 stats/graph/transport를 요구한다."""
    def __init__(self, partition, runner=subprocess.run, clock=time.monotonic, scheduled=False):
        if partition != f'arena_autonomy_test_{os.getpid()}' or os.environ.get('GZ_PARTITION') != partition:
            raise ValueError('owned test partition required')
        self.runner, self.clock = runner, clock
        self.publishers=dict(EXPECTED_GZ_PUBLISHERS)
        if scheduled:
            self.publishers.update({'/model/arena_car/eval/scheduled_command':1,
                                    '/model/arena_car/eval/scheduled_events':1})
        self.began, self.next_probe = clock(), 0.
        self.stats, self.record = [], {'mode': 'embedded_paused', 'released': False, 'probes': []}

    def observe(self, data):
        # Gazebo stats는 저빈도이다. 해제 전 전체 관측을 보존한다.
        if not self.record.get('release_requested'):
            self.stats.append(data)

    def poll(self, node, arm_client, stop_client, sim_ns, plugin_events):
        if self.record['released']:
            return True
        elapsed = self.clock()-self.began
        if sim_ns != 0 or plugin_events or any(not stats_zero_paused(s) for s in self.stats):
            raise RuntimeError('world advanced or was not paused before readiness release')
        if elapsed > 90:
            raise TimeoutError('paused startup readiness timeout')
        if elapsed < self.next_probe:
            return False
        self.next_probe = elapsed+1.
        graph = graph_snapshot(node, arm_client, stop_client)
        observation = {'elapsed_s': elapsed, 'graph': graph, 'stats_samples': len(self.stats)}
        self.record['probes'].append(observation)
        if len(self.stats) < 2 or not graph_ready(graph):
            return False
        for topic, expected_publishers in self.publishers.items():
            result = self.runner(['gz', 'topic', '-i', '-t', topic], capture_output=True, text=True, timeout=4)
            counts = topic_connections(result.stdout)
            observation[topic] = {'rc': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr, **counts}
            if result.returncode or counts['publishers'] != expected_publishers or counts['subscribers'] < 1:
                return False
        # 서비스 성공은 시작 수락일 뿐. 실제 첫 갱신/센서/정지 감사는 실행 후 별도로 한다.
        self.record.update(release_requested=True, pre_release_sim_ns=sim_ns,
                           pre_release_trace_events=len(plugin_events), stats_before_release=list(self.stats),
                           readiness=observation)
        result = self.runner(['gz', 'service', '-s', '/world/it_arena_track/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean', '--timeout', '3000',
            '--req', 'pause: false'], capture_output=True, text=True, timeout=5)
        self.record['release_response'] = {'rc': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr}
        if result.returncode or not re.search(r'\bdata:\s*true\b', result.stdout):
            raise RuntimeError('unpause request not acknowledged')
        self.record.update(released=True, release_wall_s=self.clock()-self.began)
        return True


def audit_startup(record, events):
    first = [e for e in events if e.get('type') == 'first_update']
    checks = {'release_ack': record.get('released') is True,
              'pre_release_zero': record.get('pre_release_sim_ns') == 0,
              'no_early_trace': record.get('pre_release_trace_events') == 0,
              'paused_stats': len(record.get('stats_before_release', [])) >= 2 and all(
                  stats_zero_paused(s) for s in record['stats_before_release']),
              'first_update_unique': len(first) == 1,
              'first_update_at_origin': len(first) == 1 and first[0].get('event_seq') == 1
                  and first[0].get('iteration') == 1 and first[0].get('sim_ns') == 1_000_000
                  and first[0].get('dt_ns') == 1_000_000}
    return {'checks': checks, 'passed': all(checks.values()), 'first_update': first[0] if len(first) == 1 else None}
