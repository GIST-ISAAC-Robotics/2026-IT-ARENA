"""기존 실물 진단을 함께 실행한다. 출력/차량 제어 토픽은 만들지 않는다."""
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parent
OUT = ROOT / 'capture'
C1 = Path('/home/q/it_arena_c1_20261110/speed_retry/probe_c1_rpm_delayed')
IMU = Path('/home/q/it_arena_imu_20261111/probe_esp32_imu_v3.py')
CAMERA = ROOT / 'probe_ros_image.py'


def main():
    OUT.mkdir(exist_ok=False)
    sources = (C1, IMU, CAMERA, Path(__file__), ROOT/'cost_probe.py')
    hashes = {str(p): hashlib.sha256(p.read_bytes()).hexdigest() for p in sources}
    jobs = {}
    logs = []
    begun = time.monotonic()
    error = None
    cost_started = False

    def start(name, args):
        log = (OUT/(name+'.log')).open('x')
        logs.append(log)
        p = subprocess.Popen(args, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True)
        jobs[name] = dict(process=p, pid=p.pid, started_monotonic_s=time.monotonic(),
                          started_wall_ns=time.time_ns())

    def interrupted(signum, frame):
        raise KeyboardInterrupt('diagnostic interrupted')

    signal.signal(signal.SIGTERM, interrupted)
    signal.signal(signal.SIGINT, interrupted)
    try:
        start('c1', [str(C1), '/dev/ttyUSB0', str(OUT/'c1'), '75', '5', 'Standard'])
        start('imu', [sys.executable, str(IMU), '--port', '/dev/ttyACM0',
                      '--output', str(OUT/'imu'), '--seconds', '75', '--warmup', '5'])
        start('camera', [sys.executable, str(CAMERA), str(OUT/'camera'), '--duration', '60'])
        while any(v['process'].poll() is None for v in jobs.values()):
            now = time.monotonic()
            if now-begun > 120:
                raise TimeoutError('bounded capture exceeded 120 seconds')
            for value in jobs.values():
                if value['process'].poll() is not None and 'ended_monotonic_s' not in value:
                    value['ended_monotonic_s'] = now
                    value['returncode'] = value['process'].returncode
            if not cost_started and now-begun >= 20:
                cost_started = True
                if not all(jobs[n]['process'].poll() is None for n in ('c1','imu','camera')):
                    raise RuntimeError('a sensor ended before concurrent CPU probe')
                start('cost', [sys.executable, str(ROOT/'cost_probe.py')])
            time.sleep(.2)
    except BaseException as exc:
        error = repr(exc)
    finally:
        active = [v['process'] for v in jobs.values() if v['process'].poll() is None]
        for p in active:
            os.killpg(p.pid, signal.SIGINT)
        for p in active:
            try:
                p.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(p.pid, signal.SIGKILL)
                p.wait()
                error = (error or '')+' forced cleanup'
        for value in jobs.values():
            p = value.pop('process')
            value['returncode'] = p.returncode
            value.setdefault('ended_monotonic_s', time.monotonic())
        for log in logs:
            log.close()
        result = dict(jobs=jobs, error=error, cost_started=cost_started,
                      elapsed_s=time.monotonic()-begun, source_sha256=hashes,
                      sources_unchanged=all(hashlib.sha256(Path(p).read_bytes()).hexdigest()==h
                                            for p,h in hashes.items()),
                      vehicle_actuation=False)
        (OUT/'orchestration.json').write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps(result), flush=True)
    return int(error is not None or not result['sources_unchanged'] or
               any(v['returncode'] != 0 for v in jobs.values()))


if __name__ == '__main__':
    raise SystemExit(main())
