#!/usr/bin/env python3
"""Supervised 5-second motor probe; keep one USB connection through power changes."""
import argparse
import json
from pathlib import Path
import time

from probe_esp32_motor import Bench


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--motor-power-disconnected', action='store_true', required=True)
    parser.add_argument('--start15', action='store_true',
                        help='explicit 15%% long-pulse firmware; no automatic output escalation')
    args = parser.parse_args()
    b = Bench(args.port, args.output, firmware=(
        'motor_probe_start15_5s_v1' if args.start15 else 'motor_probe_guarded_5s_v1'))
    try:
        b.send(b'?\n')
        b.wait(lambda: b.info is not None and b.motor is not None and b.encoder is not None, 5)
        b.off()
        if b.info['stop_gpio8'] != 0:
            raise RuntimeError('physical STOP is open; select I for the no-power check')
        b.send(b'S\n')
        b.phase = 'no_power_guard_check'
        test = b.pulse(150 if args.start15 else 50, 5000, observe_guard=True)
        b.save('no-power-guard.json', test)
        if (test['output_end']['reason'] != 'no_encoder_start' or
                not 330000 <= test['output_end']['ended_us'] - test['output_end']['started_us'] <= 420000):
            raise RuntimeError('unexpected no-power guard result; do not connect motor')
        b.collect(3.05)
        b.phase = 'waiting_for_ready'
        print('NO_POWER_GUARD_CONFIRMED', flush=True)
        seen = set()
        end = time.monotonic() + 250
        while time.monotonic() < end:
            b.receive()
            if (args.output / 'STOP').exists():
                break
            req = args.output / 'request.json'
            if not req.exists():
                continue
            command = json.loads(req.read_text(encoding='utf-8'))
            action = command['action']
            if action in seen:
                continue
            if action == 'verify-stop-open':
                previous = b.info
                b.send(b'?\n')
                b.wait(lambda: b.info is not previous)
                if b.info['stop_gpio8'] != 1:
                    raise RuntimeError('STOP O was not read on GPIO8')
                reply = b.request('ARM')
                if reply['result'] != 'physical_stop' or b.motor['pwm_pin'] != 0:
                    raise RuntimeError('physical STOP did not inhibit ARM')
                b.save('physical-stop-open.json', dict(physical_stop=True, arm_rejected=True))
                print('PHYSICAL_STOP_OPEN_CONFIRMED', flush=True)
            elif action in ('powered-5s', 'powered-start-75', 'powered-start-15-5s'):
                if action == 'powered-start-15-5s' and not args.start15:
                    raise ValueError('15 percent requires the explicit start15 firmware')
                previous = b.info
                b.send(b'?\n')
                b.wait(lambda: b.info is not previous)
                if b.info['stop_gpio8'] != 0:
                    raise RuntimeError('STOP has not returned to I')
                b.phase = action
                if action == 'powered-5s':
                    result = b.pulse(50, 5000, observe_guard=True)
                elif action == 'powered-start-15-5s':
                    result = b.pulse(150, 5000, observe_guard=True)
                else:
                    result = b.pulse(75, 500)
                b.save(action + '.json', result)
                print('POWERED_RESULT ' + json.dumps(result), flush=True)
                b.phase = 'idle_after_powered'
            else:
                raise ValueError('unexpected guarded bench request')
            seen.add(action)
    except BaseException as exc:
        b.save('error.json', dict(phase=b.phase, error=repr(exc)))
        raise
    finally:
        b.close()


if __name__ == '__main__':
    main()
