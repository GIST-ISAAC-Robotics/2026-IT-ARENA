"""재생 전용 최소 IMU 수신자. 제어/시계 입력 없이 시각만 메모리에 기록한다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

NODE_NAME = 'minimal_imu_probe'
TOPIC = '/replay/imu/data'
MAX_SAMPLES = 250_000  # 250 Hz에서 약 16분. 넘치면 조용히 버리지 않고 시험 실패.


class StampBuffer:
    def __init__(self, limit=MAX_SAMPLES):
        if limit < 1:
            raise ValueError('Positive sample limit required')
        self.limit = limit
        self.rows = []

    def append(self, source_ns, begin_monotonic_ns):
        if len(self.rows) >= self.limit:
            raise RuntimeError('IMU probe sample limit exceeded')
        self.rows.append((source_ns, begin_monotonic_ns))

    def save(self, path):
        with Path(path).open('x', encoding='utf-8') as stream:
            for stamp, began in self.rows:
                stream.write(json.dumps(dict(source_ns=stamp, begin_monotonic_ns=began))+'\n')


def read_result(output):
    """정상 종료 및 파일 무결성과 센서 무손실을 혼동하지 않는다."""
    output = Path(output)
    result = json.loads((output/'result.json').read_text())
    payload = (output/'samples.jsonl').read_bytes()
    if not result['complete'] or result['errors']:
        raise ValueError('Incomplete IMU probe capture')
    if hashlib.sha256(payload).hexdigest() != result['samples_sha256']:
        raise ValueError('IMU probe file digest mismatch')
    rows = [json.loads(line) for line in payload.splitlines()]
    if len(rows) != result['count']:
        raise ValueError('IMU probe count mismatch')
    if any(b['begin_monotonic_ns'] < a['begin_monotonic_ns'] for a, b in zip(rows, rows[1:])):
        raise ValueError('IMU probe callback clock reversed')
    if result['topic'] != TOPIC or result['qos'] != dict(depth=5, reliability='BEST_EFFORT', history='KEEP_LAST'):
        raise ValueError('Unexpected IMU probe subscription contract')
    if result['executor_mode'] != 'legacy':
        raise ValueError('Unexpected IMU probe executor')
    return result, rows


class ImuProbe:
    def __init__(self, output):
        self.output = Path(output)
        self.process = None
        self.log = None

    def start(self):
        self.output.mkdir(exist_ok=False)
        self.log = (self.output/'process.log').open('x')
        self.process = subprocess.Popen([sys.executable, str(Path(__file__).resolve()),
            '--output', str(self.output)], stdout=self.log, stderr=self.log,
            stdin=subprocess.DEVNULL, start_new_session=True)
        deadline = time.monotonic()+15.
        while not (self.output/'ready.json').exists():
            self.check()
            if time.monotonic() >= deadline:
                raise RuntimeError('IMU probe readiness timeout')
            time.sleep(.02)

    def check(self):
        if self.process is not None and self.process.poll() is not None:
            raise RuntimeError('IMU probe exited early: '+str(self.process.returncode))

    def close(self):
        try:
            if self.process is None:
                return None
            if self.process.poll() is None:
                self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=5.)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3.)
                raise RuntimeError('IMU probe forced cleanup')
            if self.process.returncode != 0:
                raise RuntimeError('IMU probe failed: '+str(self.process.returncode))
            result, _ = read_result(self.output)
            return result
        finally:
            if self.log:
                self.log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from sensor_msgs.msg import Imu
    from replay_transport import audit_current
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node(NODE_NAME, namespace='/replay',
                             enable_rosout=False, start_parameter_services=False)
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    buffer = StampBuffer()
    errors, ready, qos = [], False, None
    try:
        audit_current(NODE_NAME)
        def callback(message):
            began = time.perf_counter_ns()
            stamp = message.header.stamp.sec*1_000_000_000+message.header.stamp.nanosec
            buffer.append(stamp, began)
        subscription = node.create_subscription(Imu, TOPIC, callback, qos_profile_sensor_data)
        actual = subscription.qos_profile
        qos = dict(depth=actual.depth, reliability=actual.reliability.name, history=actual.history.name)
        deadline = time.monotonic()+10.
        while not stopped:
            # 기존 제어기와 같은 legacy spin_once. 별도 최적화는 적용하지 않는다.
            rclpy.spin_once(node, timeout_sec=.02)
            if not ready:
                count = node.count_publishers(TOPIC)
                if count > 1:
                    raise RuntimeError('Unexpected multiple IMU publishers')
                if count == 1:
                    (args.output/'ready.json').write_text(json.dumps(dict(pid=os.getpid(),
                        domain=os.environ.get('ROS_DOMAIN_ID'), qos=qos)))
                    ready = True
                elif time.monotonic() >= deadline:
                    raise RuntimeError('IMU publisher discovery timeout')
    except Exception as error:
        errors.append(repr(error))
    finally:
        node.destroy_node()
        rclpy.shutdown()
    path = args.output/'samples.jsonl'
    buffer.save(path)
    result = dict(complete=bool(ready and stopped and not errors), errors=errors,
        count=len(buffer.rows), samples_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        topic=TOPIC, qos=qos, executor_mode='legacy', max_samples=buffer.limit,
        clock='same-host perf_counter_ns; source header is not wall time',
        scope='Passive additional subscriber; changes diagnostic load, not production controller')
    (args.output/'result.json').write_text(json.dumps(result, indent=2))
    return int(not result['complete'])


if __name__ == '__main__':
    raise SystemExit(main())
