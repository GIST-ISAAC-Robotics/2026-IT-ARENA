"""운영 종료 코드는 바꾸지 않고 같은 카메라 시험의 종료 대기를 계측합니다."""
import argparse
import ctypes
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import audit_gantry_camera_views as capture
from audit_official_world_load import descendants_alive


def snapshot(pgid):
    rows = []
    for pid in descendants_alive(pgid):
        root = Path(f'/proc/{pid}')
        try:
            rows.append({'pid': pid, 'comm': (root/'comm').read_text().strip(),
                'cmdline': (root/'cmdline').read_bytes().replace(b'\0', b' ').decode(errors='replace'),
                'cwd': str((root/'cwd').resolve()), 'stat': (root/'stat').read_text(),
                'wchan': (root/'wchan').read_text(),
                'threads': [{'tid': int(t.name), 'comm': (t/'comm').read_text().strip(),
                             'wchan': (t/'wchan').read_text(), 'stat': (t/'stat').read_text()}
                            for t in (root/'task').iterdir()]})
        except (FileNotFoundError, ProcessLookupError):
            pass
    return rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--name', required=True)
    parser.add_argument('--grace', type=float, default=80)
    parser.add_argument('--signal-mode', choices=['group', 'leader', 'service'], default='group')
    parser.add_argument('--stacks', action='store_true')
    parser.add_argument('--capture-mode', choices=['route','overview'], default='route')
    args = parser.parse_args()
    original_stop = capture.stop_test
    original_popen = subprocess.Popen
    output = capture.REPO/'artifacts/validation/2026-09-15/official_update'/args.name
    def traced_popen(command, *a, **kw):
        if args.stacks and command[:2] == ['gz', 'sim']:
            # Only this disposable diagnostic server allows sibling GDB attachment.
            # No global ptrace policy or system configuration is changed.
            def allow_debugger():
                if ctypes.CDLL(None).prctl(0x59616d61, ctypes.c_ulong(-1).value, 0, 0, 0) != 0:
                    raise RuntimeError('PR_SET_PTRACER failed')
            kw['preexec_fn'] = allow_debugger
        return original_popen(command, *a, **kw)
    subprocess.Popen = traced_popen
    def stop(process):
        if process.args[0] != 'gz':
            return original_stop(process)
        trace = {'signal_mode': args.signal_mode, 'grace_s': args.grace, 'pgid': process.pid,
                 'samples': [], 'signals': [], 'started_epoch': time.time()}
        start = time.monotonic()
        initial = snapshot(process.pid)
        trace['samples'].append({'t': 0, 'processes': initial})
        target = process.pid
        if args.signal_mode == 'service':
            # capture.main assigns this run a unique GZ_PARTITION. The request
            # therefore cannot stop another user's or test's Gazebo server.
            command = ['gz', 'service', '-s', '/server_control',
                       '--reqtype', 'gz.msgs.ServerControl', '--reptype', 'gz.msgs.Boolean',
                       '--timeout', '5000', '--req', 'stop: true']
            try:
                result = subprocess.run(command, capture_output=True, text=True, timeout=10)
                trace['service_result'] = {'command': command, 'return_code': result.returncode,
                                           'stdout': result.stdout, 'stderr': result.stderr,
                                           'partition': os.environ['GZ_PARTITION']}
            except subprocess.TimeoutExpired:
                trace['service_result'] = {'timeout': True}
            trace['signals'].append({'signal': 'SERVER_CONTROL_STOP', 't': 0})
        elif args.signal_mode == 'group':
            os.killpg(target, signal.SIGINT)
        else:
            os.kill(target, signal.SIGINT)
        if args.signal_mode != 'service':
            trace['signals'].append({'signal': 'SIGINT', 'target': target, 't': 0})
        stack_times = [10, 35] if args.stacks else []
        while descendants_alive(process.pid) and time.monotonic()-start < args.grace:
            now = time.monotonic()-start
            trace['samples'].append({'t': now, 'processes': snapshot(process.pid)})
            (output/'shutdown_trace.json').write_text(json.dumps(trace, indent=2))
            if stack_times and now >= stack_times[0]:
                label = stack_times.pop(0)
                for row in snapshot(process.pid):
                    pid = row['pid']
                    if Path(row['cwd']) != capture.REPO/'build/official_update_20260915/render'/args.name:
                        continue
                    (output/f'maps_{label}_{pid}.txt').write_text(Path(f'/proc/{pid}/maps').read_text())
                    with (output/f'gdb_{label}_{pid}.log').open('w') as handle:
                        try:
                            subprocess.run(['gdb','-batch','-nx','-ex','set debuginfod enabled off',
                                '-ex','set pagination off','-ex','thread apply all bt 18',
                                '-ex','detach','-p',str(pid)],stdout=handle,stderr=subprocess.STDOUT,timeout=20)
                        except subprocess.TimeoutExpired:
                            trace['gdb_timeout'] = True
            time.sleep(2)
        for sig, grace in ((signal.SIGTERM, 5), (signal.SIGKILL, 3)):
            if not descendants_alive(process.pid):
                break
            trace['signals'].append({'signal': sig.name, 't': time.monotonic()-start})
            os.killpg(process.pid, sig)
            deadline = time.monotonic()+grace
            while descendants_alive(process.pid) and time.monotonic()<deadline:
                time.sleep(.1)
        process.wait(timeout=3)
        trace.update(return_code=process.returncode, elapsed_s=time.monotonic()-start,
                     remaining_test_pids=descendants_alive(process.pid))
        (output/'shutdown_trace.json').write_text(json.dumps(trace, indent=2))
        return {'return_code': process.returncode, 'signals': [s['signal'] for s in trace['signals']],
                'remaining_test_pids': trace['remaining_test_pids'], 'elapsed_s': trace['elapsed_s']}
    capture.stop_test = stop
    sys.argv = ['audit_gantry_camera_views.py', '--name', args.name, '--marker-face', 'sdf_cells',
                '--mode', args.capture_mode, '--timeout', '180']
    return capture.main()


if __name__ == '__main__':
    raise SystemExit(main())
