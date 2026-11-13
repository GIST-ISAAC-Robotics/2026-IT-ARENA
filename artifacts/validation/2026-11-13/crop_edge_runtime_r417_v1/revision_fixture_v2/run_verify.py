"""지정 실행 담당자용 한 묶음. 기존 입력 불변/실패 보존/새 출력 경로를 강제한다."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

R = Path(__file__).resolve().parent
A = R.parent
ROOT = A.parents[3]
REGRESSION = ['test_partial_observation.py', 'test_g1_partial_policy.py',
    'test_known_structure_policy.py', 'test_traffic_recovery_contract.py',
    'test_native_scan_pipeline.py', 'test_native_nonreturn_roundtrip.py',
    'test_runtime_node_backend.py', 'test_follow_trace.py']


def frozen_inputs():
    value = json.loads((R/'manifest.json').read_bytes())
    for item in value['files']:
        actual = hashlib.sha256((ROOT/item['path']).read_bytes()).hexdigest()
        assert actual == item['sha256'], 'frozen input changed: '+item['path']
    return len(value['files'])


def write(path, value):
    with path.open('x', encoding='utf8') as stream:
        json.dump(value, stream, indent=2, ensure_ascii=False)
        stream.write('\n')


def execute(command, environment, log):
    with log.open('x', encoding='utf8') as stream:
        process = subprocess.Popen(command, cwd=ROOT, env=environment, stdout=stream,
            stderr=subprocess.STDOUT, start_new_session=True)
        try:
            return process.wait(timeout=600), process.pid
        except subprocess.TimeoutExpired:
            # 이 시험이 시작한 process group만 회수한다. 기존 ROS/장치는 건드리지 않는다.
            for sig, wait in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 3)):
                try:
                    os.killpg(process.pid, sig)
                except ProcessLookupError:
                    break
                try:
                    process.wait(timeout=wait)
                    break
                except subprocess.TimeoutExpired:
                    pass
            raise


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    destination = Path(args.output).resolve()
    assert destination.parent == A and not destination.exists(), 'new r417 output directory required'
    count = frozen_inputs()
    destination.mkdir()
    environment = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', PYTHONHASHSEED='0',
        ROS_DOMAIN_ID='223', ROS_LOCALHOST_ONLY='1')
    # ROS setup이 제공한 rclpy 경로를 제거하지 않는다.
    paths = [A/'test_support', ROOT/'src/arena_autonomy', ROOT/'src/arena_vehicle_interface', ROOT/'scripts', ROOT/'tests']
    environment['PYTHONPATH'] = os.pathsep.join([*(str(p) for p in paths), environment.get('PYTHONPATH', '')])
    report = dict(scope='configuration + isolated native spawn + two actual ROS nodes; no control/spin/drive',
                  sources=count, results=[], passed=False)
    try:
        groups = [('related', [str(ROOT/'tests'/name) for name in REGRESSION] +
                   [str(R/'test_crop_edge_runtime_v2.py'), str(A/'test_launch_contract.py')]),
                  ('actual_ros', [str(R/'test_crop_edge_ros_v2.py')])]
        for label, files in groups:
            command = [sys.executable, '-B', '-m', 'pytest', '-q', '-s', '-p', 'no:cacheprovider',
                       '--junitxml='+str(destination/(label+'.xml')), *files]
            began = time.monotonic()
            code, pid = execute(command, environment, destination/(label+'.log'))
            item = dict(name=label, command=command, pid=pid, exit_code=code,
                        elapsed_s=time.monotonic()-began)
            report['results'].append(item)
            assert frozen_inputs() == count
            assert code == 0, (label, code)
            suites = ET.parse(destination/(label+'.xml')).getroot().iter('testsuite')
            counts = {key: 0 for key in ('tests', 'failures', 'errors', 'skipped')}
            for suite in suites:
                for key in counts:
                    counts[key] += int(suite.attrib.get(key, 0))
            item['counts'] = counts
            assert counts['tests'] > 0 and not any(counts[k] for k in ('failures','errors','skipped')), counts
        report['passed'] = True
    finally:
        try:
            report['source_files_unchanged'] = frozen_inputs()
        finally:
            write(destination/'report.json', report)


if __name__ == '__main__':
    main()
