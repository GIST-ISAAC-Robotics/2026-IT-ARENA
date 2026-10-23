#!/usr/bin/env python3
"""선택형 센서 기록 실행: 자기 자식만 제한 종료하며 성공과 강제 종료를 구분한다."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

from recording_guard import validate_limits
ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from sensor_recording_profile import select_profile


def bounded_seconds(value, name, low, high):
    if type(value) not in (int, float) or not math.isfinite(value) or not low <= value <= high:
        raise ValueError(f'{name} must be within {low}..{high} seconds')
    return float(value)


def signal_owned_group(process, sig):
    """Only the fresh session leader returned by this supervisor is eligible."""
    if process.poll() is not None:
        return False
    try:
        if os.getpgid(process.pid) != process.pid:
            raise RuntimeError('recorder is no longer the owned process-group leader')
        os.killpg(process.pid, sig)
        return True
    except ProcessLookupError:
        return False


def supervise(command, ready_path, log, *, duration, startup_timeout=30., stop_timeout=75.,
              term_timeout=5., kill_timeout=5., cancelled=lambda: False):
    """No ROS calls; testable child lifecycle. Nonzero/early/forced exits stay failed.

    The CLI constructs the only production command (sensor_probe recorder).
    An OS-uninterruptible process can outlive SIGKILL; report it, never claim exit.
    """
    if os.name != 'posix':
        raise RuntimeError('Linux/WSL process sessions required')
    limits = dict(duration=bounded_seconds(duration, 'duration', .05, 600.),
                  startup_timeout=bounded_seconds(startup_timeout,'startup_timeout',.05,120.),
                  stop_timeout=bounded_seconds(stop_timeout,'stop_timeout',.05,120.),
                  term_timeout=bounded_seconds(term_timeout,'term_timeout',.05,30.),
                  kill_timeout=bounded_seconds(kill_timeout,'kill_timeout',.05,30.))
    started = time.monotonic()
    ready_path = Path(ready_path)
    if ready_path.exists():
        raise FileExistsError('existing readiness file must not be reused')
    child = subprocess.Popen(command, stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
                             start_new_session=True)
    ready_at = None
    events, errors = [], []
    reason = None
    signal_sent = False
    normal_stop = False
    try:
        deadline = started+limits['startup_timeout']
        while True:
            now = time.monotonic()
            if child.poll() is not None:
                reason = 'premature_exit'
                break
            if cancelled():
                reason = 'cancelled'
                break
            if ready_at is None and ready_path.exists():
                # Writer opens readiness once; a concurrent partial JSON is not ready.
                try:
                    with ready_path.open('rb') as stream:
                        data = stream.read(4097)
                    if len(data) > 4096:
                        raise ValueError('oversized ready file')
                    ready = json.loads(data)
                except json.JSONDecodeError:
                    ready = None
                if ready is not None:
                    if (type(ready) is not dict or ready.get('ready') is not True
                            or type(ready.get('pid')) is not int or ready['pid'] != child.pid):
                        raise ValueError('readiness does not identify the owned recorder')
                    ready_at, deadline = now, now+limits['duration']
                    events.append(dict(event='ready', elapsed_s=now-started))
            if now >= deadline:
                reason = 'planned_stop' if ready_at is not None else 'startup_timeout'
                normal_stop = ready_at is not None
                break
            time.sleep(min(.02, max(0.,deadline-now)))
    except Exception as error:
        errors.append(repr(error))
        reason = 'supervisor_error'
    finally:
        # Use one process group created above; never enumerate or kill other ROS nodes.
        for sig, grace in ((signal.SIGINT,limits['stop_timeout']),
                           (signal.SIGTERM,limits['term_timeout']),
                           (signal.SIGKILL,limits['kill_timeout'])):
            if child.poll() is not None:
                break
            try:
                delivered = signal_owned_group(child,sig)
                events.append(dict(event='signal', signal=signal.Signals(sig).name,
                                   delivered=delivered, elapsed_s=time.monotonic()-started))
                signal_sent |= delivered
                child.wait(timeout=grace)
            except subprocess.TimeoutExpired:
                events.append(dict(event='wait_timeout', signal=signal.Signals(sig).name,
                                   elapsed_s=time.monotonic()-started))
            except Exception as error:
                errors.append(repr(error))
                break
    code = child.poll()
    forced = any(e.get('event')=='signal' and e.get('delivered')
                 and e.get('signal') in ('SIGTERM','SIGKILL') for e in events)
    return dict(lifecycle_complete=normal_stop and signal_sent and not forced and code==0 and not errors,
                stop_reason=reason, exit_code=code, process_exited=code is not None,
                forced_termination=forced, ready_observed=ready_at is not None,
                pid=child.pid, elapsed_s=time.monotonic()-started, limits=limits,
                events=events, errors=errors)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True,help='새 세션 폴더; 원시 기록은 sensor_recording 하위')
    parser.add_argument('--clock-mode',choices=('system','simulation'),required=True)
    parser.add_argument('--sensor-topic',action='append',required=True)
    parser.add_argument('--duration',type=float,required=True,help='ready 이후 계획 기록 시간 1..600 s')
    parser.add_argument('--max-raw-gib',type=float,default=.1)
    args = parser.parse_args(argv)
    select_profile('sensor_probe',args.clock_mode,args.sensor_topic,'drive_motor_shaft')
    duration = bounded_seconds(args.duration,'duration',1.,600.)
    # Record limit is an incomplete-data bound, longer than the planned clean stop.
    wall_limit = duration+35.
    validate_limits(args.max_raw_gib,wall_limit)
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts') or output==ROOT/'artifacts':
        raise ValueError('new session directory beneath artifacts required')
    if os.name != 'posix':
        raise RuntimeError('Linux/WSL only; no cross-shell process termination')
    output.mkdir(parents=True,exist_ok=False)
    recording = output/'sensor_recording'
    command = [sys.executable,str(ROOT/'scripts/record_sensor_bag.py'),'--output',str(recording),
               '--profile','sensor_probe','--clock-mode',args.clock_mode,
               '--drive-feedback-mode','drive_motor_shaft','--max-wall-seconds',str(wall_limit),
               '--max-raw-gib',str(args.max_raw_gib)]
    for topic in args.sensor_topic:
        command += ['--sensor-topic',topic]
    cancelled = False
    def cancel(*_):
        nonlocal cancelled
        cancelled = True
    previous = {sig:signal.signal(sig,cancel) for sig in (signal.SIGINT,signal.SIGTERM)}
    result = None
    try:
        with (output/'recorder.log').open('x') as log:
            result = supervise(command,recording/'ready.json',log,duration=duration,cancelled=lambda:cancelled)
        # Close success is necessary, not sufficient. Preserve the recorder's manifest.
        result['record_integrity'] = None
        if result['lifecycle_complete']:
            try:
                from audit_sensor_probe import audit
                result['record_integrity'] = audit(recording)
            except Exception as error:
                result['errors'].append('record audit failed: '+repr(error))
        result.update(schema='arena_sensor_probe_session_v1', command=command,
                      complete=result['lifecycle_complete'] and result['record_integrity'] is not None and not result['errors'] and not cancelled,
                      cancellation_requested=cancelled,
                      controller_replay_eligible=False, hardware_verified=False, ready_to_drive=False)
        with (output/'session.json').open('x') as stream:
            json.dump(result,stream,indent=2,allow_nan=False)
        print(json.dumps({k:result[k] for k in ('complete','stop_reason','exit_code','process_exited','forced_termination')}))
        return 0 if result['complete'] else 1
    finally:
        for sig, handler in previous.items():
            signal.signal(sig,handler)


if __name__ == '__main__':
    raise SystemExit(main())
