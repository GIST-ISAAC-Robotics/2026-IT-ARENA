"""재생 발행과 별도 프로세스에서 출력/진단만 수집한다."""
from collections import Counter, defaultdict
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time


def sequence_coverage(rows, completed):
    """종료 전 스냅샷까지의 연속 번호를 검사한다. 이후 콜백은 별도로 센다."""
    observed = defaultdict(Counter)
    for row in rows:
        observed[row['node']+'/'+row['callback']][row['sequence']] += 1
    result = {}
    for name, count in completed.items():
        values = observed[name]
        missing = [i for i in range(1, count+1) if i not in values]
        duplicates = sum(n-1 for i, n in values.items() if 1 <= i <= count)
        invalid = sum(n for i, n in values.items() if i < 1)
        result[name] = dict(expected_through_snapshot=count, missing=len(missing),
            first_missing=missing[:10], duplicates=duplicates, invalid=invalid,
            after_snapshot=sum(n for i, n in values.items() if i > count),
            complete=not missing and not duplicates and not invalid)
    return result


class ReplayObserver:
    """파일 IPC: 제어/센서 발행 없음. 종료 확인 뒤에만 최종 기록을 읽는다."""
    def __init__(self, output, env=None):
        self.output = Path(output)
        self.env = env
        self.process = None
        self.log = None

    def start(self):
        self.output.mkdir(exist_ok=False)
        self.log = (self.output/'process.log').open('w')
        self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
            '--output', str(self.output)], env=self.env, stdout=self.log, stderr=self.log,
            stdin=subprocess.DEVNULL, start_new_session=True)
        deadline = time.monotonic()+10
        while not (self.output/'ready.json').exists():
            self.check()
            if time.monotonic() > deadline:
                raise RuntimeError('Replay observer readiness timeout')
            time.sleep(.02)

    def check(self):
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError('Replay observer exited early: '+str(self.process.returncode))

    def wait_for_publishers(self, node, expected, timeout=10.):
        """발행자 쪽 발견만으로 충분하지 않다. 수집기 쪽 발견도 확인한다."""
        from rclpy.executors import SingleThreadedExecutor
        from std_srvs.srv import Trigger
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        client = node.create_client(Trigger, '/replay/sensor_replay_observer/ready')
        deadline = time.monotonic()+timeout
        try:
            while time.monotonic() < deadline:
                self.check()
                if not client.wait_for_service(timeout_sec=.1):
                    continue
                future = client.call_async(Trigger.Request())
                while not future.done() and time.monotonic() < deadline:
                    executor.spin_once(timeout_sec=.01)
                    self.check()
                if future.done():
                    response = future.result()
                    if response.success:
                        seen = json.loads(response.message)
                        if all(seen.get(topic, 0) >= count for topic, count in expected.items()):
                            return seen
                time.sleep(.01)
            raise RuntimeError('Replay observer publisher discovery timeout')
        finally:
            node.destroy_client(client)
            executor.remove_node(node)
            executor.shutdown(timeout_sec=1.)

    def snapshot(self):
        if self.process is not None and self.process.poll() is None:
            raise RuntimeError('Cannot read final observer records while running')
        rows = defaultdict(list)
        path = self.output/'observations.jsonl'
        if path.exists():
            with path.open() as stream:
                for line in stream:
                    record = json.loads(line)
                    rows[record['key']].append(record['row'])
        return rows

    def close(self):
        try:
            if self.process is None:
                return
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3.)
                raise RuntimeError('Replay observer forced cleanup')
            if self.process.returncode != 0:
                raise RuntimeError('Replay observer failed: '+str(self.process.returncode))
        finally:
            if self.log:
                self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from ackermann_msgs.msg import AckermannDriveStamped
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('sensor_replay_observer', namespace='/replay')
    from replay_transport import audit_current
    try:
        audit_current('sensor_replay_observer')
    except BaseException:
        node.destroy_node()
        rclpy.shutdown()
        raise
    executor = SingleThreadedExecutor(context=node.context)
    executor.add_node(node)
    stopped_at = None
    last_receive = time.monotonic()
    counts, errors = Counter(), []
    def stop(*_):
        nonlocal stopped_at
        if stopped_at is None:
            stopped_at = time.monotonic()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    with (args.output/'observations.jsonl').open('x', buffering=1024*1024) as stream:
        def record(key, row):
            nonlocal last_receive
            stream.write(json.dumps(dict(key=key, row=row))+'\n')
            counts[key] += 1
            last_receive = time.monotonic()
        def json_callback(key):
            def callback(message):
                record(key, json.loads(message.data))
            return callback
        def drive_callback(key):
            def callback(message):
                stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
                record(key, [stamp, message.drive.speed, message.drive.steering_angle])
            return callback
        node.create_subscription(String, '/replay/diagnostics/timing', json_callback('timing'), 1000)
        for topic in ('/autonomy/status', '/safety/status'):
            node.create_subscription(String, '/replay'+topic, json_callback(topic), 100)
        for topic in ('/drive', '/drive/safe'):
            node.create_subscription(AckermannDriveStamped, '/replay'+topic, drive_callback(topic), 100)
        observed_topics = ['/replay/diagnostics/timing', '/replay/drive', '/replay/drive/safe',
                           '/replay/autonomy/status', '/replay/safety/status']
        def ready(request, response):
            response.success = True
            response.message = json.dumps({topic: node.count_publishers(topic) for topic in observed_topics})
            return response
        node.create_service(Trigger, '~/ready', ready)
        (args.output/'ready.json').write_text(json.dumps(dict(pid=os.getpid(), domain=os.environ.get('ROS_DOMAIN_ID'))))
        try:
            while True:
                now = time.monotonic()
                if stopped_at is not None and ((now-stopped_at >= .25 and now-last_receive >= .2) or now-stopped_at >= 2.):
                    break
                # rclpy.spin_once(node)는 매 호출 노드를 executor에 다시 붙였다 뗀다.
                # 고빈도 수집은 한 번 등록한 executor를 계속 사용한다.
                executor.spin_once(timeout_sec=.02)
        except Exception as error:
            errors.append(repr(error))
        finally:
            executor.shutdown(timeout_sec=1.)
            executor.remove_node(node)
            node.destroy_node()
            rclpy.shutdown()
    (args.output/'result.json').write_text(json.dumps(dict(counts=dict(counts), errors=errors)))
    return int(bool(errors))


if __name__ == '__main__':
    raise SystemExit(main())
