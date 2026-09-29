#!/usr/bin/env python3
"""구동부 없는 A3 재생 1회와 자원 사용 기록. 전력/클록 설정은 변경하지 않는다."""
import argparse
import json
import os
from pathlib import Path
import platform
import signal
import subprocess
import sys
import time
from replay_transport import MODES


def process_samples(root_pid):
    processes = {}
    for path in Path('/proc').glob('[0-9]*/stat'):
        try:
            text = path.read_text()
            fields = text[text.rfind(')') + 2:].split()
            pid = int(path.parent.name)
            processes[pid] = dict(pid=pid, ppid=int(fields[1]),
                cpu_ticks=int(fields[11]) + int(fields[12]), rss_bytes=int(fields[21])*os.sysconf('SC_PAGE_SIZE'),
                command=(path.parent/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace'))
        except (OSError, ValueError, IndexError):
            continue
    included = {root_pid}
    while True:
        children = {p['pid'] for p in processes.values() if p['ppid'] in included}
        if children <= included:
            break
        included |= children
    return [processes[p] for p in sorted(included) if p in processes]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('recording', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--rate', type=float, default=1.)
    parser.add_argument('--timing-mode', choices=('topic', 'buffered'), default='topic')
    parser.add_argument('--timeout', type=float, default=300.)
    parser.add_argument('--executor-mode', choices=('legacy', 'retained'))
    parser.add_argument('--trace-wheels-publication', action='store_true')
    parser.add_argument('--trace-imu-publication', action='store_true')
    parser.add_argument('--imu-probe', action='store_true')
    parser.add_argument('--dds-transport', choices=MODES, default='system')
    parser.add_argument('--dds-audit-library', type=Path)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    output = args.output.resolve()
    if not output.is_relative_to(root/'artifacts'):
        raise ValueError('Trial output must be under artifacts')
    output.mkdir(parents=True, exist_ok=False)
    command = [sys.executable, str(root/'scripts/replay_sensor_bag.py'), str(args.recording.resolve()),
               '--output', str(output/'replay'), '--rate', str(args.rate), '--timing-mode', args.timing_mode]
    if args.executor_mode is not None:
        command += ['--executor-mode', args.executor_mode]
    if args.trace_wheels_publication:
        command += ['--trace-wheels-publication']
    if args.trace_imu_publication:
        command += ['--trace-imu-publication']
    if args.imu_probe:
        command += ['--imu-probe']
    command += ['--dds-transport', args.dds_transport]
    if args.dds_audit_library is not None:
        command += ['--dds-audit-library', str(args.dds_audit_library.resolve())]
    started = time.monotonic()
    errors = []
    requested_stop = False
    def stop(*_):
        nonlocal requested_stop
        requested_stop = True
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, stop)
    with (output/'replay_stdout.log').open('wb') as logfile, \
         (output/'tegrastats.log').open('wb') as teglog, \
         (output/'resources.jsonl').open('w') as samples:
        telemetry = subprocess.Popen(['tegrastats', '--interval', '500'], stdin=subprocess.DEVNULL,
                                      stdout=teglog, stderr=subprocess.STDOUT)
        process = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=logfile,
                                   stderr=subprocess.STDOUT, start_new_session=True)
        known_children = set()
        try:
            while process.poll() is None:
                now = time.monotonic()
                rows = process_samples(process.pid)
                known_children.update(p['pid'] for p in rows)
                samples.write(json.dumps(dict(monotonic_s=now, elapsed_s=now-started, processes=rows))+'\n')
                samples.flush()
                if requested_stop or now-started > args.timeout:
                    errors.append('interrupted' if requested_stop else 'trial timeout')
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=30)
                    except subprocess.TimeoutExpired:
                        # 각 제어기는 독립 세션이므로 재생기 강제 종료 전에 현재 자손을 확인한다.
                        current = process_samples(process.pid)
                        for row in reversed(current):
                            try:
                                os.kill(row['pid'], signal.SIGKILL)
                            except ProcessLookupError:
                                pass
                        process.wait(timeout=10)
                        errors.append('forced trial cleanup')
                    break
                time.sleep(.5)
        finally:
            telemetry.terminate()
            try:
                telemetry.wait(timeout=5)
            except subprocess.TimeoutExpired:
                telemetry.kill()
                telemetry.wait()
                errors.append('forced telemetry cleanup')
    report_path = output/'replay/report.json'
    report = json.loads(report_path.read_text()) if report_path.exists() else {}
    remaining = []
    for pid in known_children:
        try:
            cmd = Path(f'/proc/{pid}/cmdline').read_bytes().decode(errors='replace')
            if any(name in cmd for name in ('arena_autonomy.', 'replay_sensor_bag.py', 'replay_observer.py', 'replay_imu_probe.py',
                                            'multiprocessing.spawn', 'multiprocessing.resource_tracker')):
                remaining.append(pid)
        except OSError:
            pass
    if remaining:
        errors.append('remaining replay/controller processes')
    result = dict(command=command, python=sys.version, platform=platform.platform(),
                  clock_ticks_per_second=os.sysconf('SC_CLK_TCK'), rate=args.rate,
                  elapsed_wall_s=time.monotonic()-started, returncode=process.returncode,
                  telemetry_returncode=telemetry.returncode, remaining_pids=remaining, errors=errors,
                  replay_passed=report.get('passed'), functional_passed=report.get('functional_passed'),
                  schedule_passed=report.get('schedule_passed'),
                  measurement_passed=report.get('measurement_passed'),
                  operational_passed=report.get('operational_passed'),
                  scope='Open-loop compute replay with telemetry, no simulator, MCU or motors')
    (output/'trial.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result))
    return int(bool(errors) or process.returncode != 0)


if __name__ == '__main__':
    raise SystemExit(main())
