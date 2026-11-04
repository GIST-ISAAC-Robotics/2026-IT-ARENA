"""차량 없이 운영 어댑터의 실행 래퍼/직접 노드 SIGINT 차이를 한 번 비교한다."""
import json
import os
from pathlib import Path
import signal
import subprocess
import time
import urllib.request

from ament_index_python.packages import get_package_prefix

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent


def members(pgid):
    rows = subprocess.check_output(['ps', '-eo', 'pid,pgid,args'], text=True).splitlines()[1:]
    return [row.strip() for row in rows if len(row.split(None, 2)) == 3 and row.split(None, 2)[1] == str(pgid)]


def probe(name, command, port):
    output = OUT/name
    output.mkdir(exist_ok=False)
    token = output/'token.txt'
    token.write_text('local-diagnostic-token', encoding='utf-8')
    args = ['--ros-args', '-p', 'vehicle_id:=probe', '-p', f'port:={port}',
            '-p', f'token_path:={token}', '-p', f'log_path:={output/"ops.jsonl"}']
    report = dict(command=command, port=port)
    with (output/'console.log').open('w') as log:
        process = subprocess.Popen(command+args, stdin=subprocess.DEVNULL, stdout=log, stderr=log,
                                   start_new_session=True, cwd=ROOT)
        report['pid'] = process.pid
        try:
            deadline = time.monotonic()+25.
            while True:
                try:
                    with urllib.request.urlopen(f'http://127.0.0.1:{port}/', timeout=.5) as response:
                        assert response.status == 200
                    break
                except OSError:
                    if process.poll() is not None or time.monotonic() > deadline:
                        raise RuntimeError('HTTP node did not start')
                    time.sleep(.1)
            report['ready_members'] = members(process.pid)
            begin = time.monotonic()
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=3.)
            except subprocess.TimeoutExpired:
                pass
            report['parent_signal_elapsed_s'] = time.monotonic()-begin
            report['after_parent_exit'] = process.poll()
            report['after_parent_members'] = members(process.pid)
            if report['after_parent_members']:
                begin = time.monotonic()
                os.killpg(process.pid, signal.SIGINT)
                process.wait(timeout=5.)
                report['group_signal_elapsed_s'] = time.monotonic()-begin
            report['exit_code'] = process.returncode
        except Exception as error:
            report['error'] = repr(error)
        finally:
            if members(process.pid):
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5.)
                report['forced_kill'] = True
            report['remaining_members'] = members(process.pid)
    (output/'report.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    return report


if __name__ == '__main__':
    os.environ.update(ROS_DOMAIN_ID='179', ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST', ROS_STATIC_PEERS='')
    node = str(Path(get_package_prefix('arena_autonomy'))/'lib/arena_autonomy/race_ops')
    results = [probe('wrapper', ['ros2', 'run', 'arena_autonomy', 'race_ops'], 18782),
               probe('direct', [node], 18783)]
    (OUT/'lifecycle_report.json').write_text(json.dumps(results, indent=2), encoding='utf-8')
    print(json.dumps(results, indent=2))
