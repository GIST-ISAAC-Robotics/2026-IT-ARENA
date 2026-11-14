"""Exercise the mount tool against an isolated Linux PTY, never a USB device."""
import json
import os
from pathlib import Path
import select
import struct
import subprocess
import sys
import tempfile
import threading
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from mounted_servo_setup import summarize_manual, target_count
from probe_bus_servo import packet


class BoundsTests(unittest.TestCase):
    def test_small_return_and_no_wrap(self):
        self.assertEqual(target_count(3860, 3879, None, 0, 4095), 3872)
        self.assertEqual(target_count(3884, 3879, None, 0, 4095), 3879)
        for current, neutral, delta in [(3879, 3879, 13), (3929, 3879, 12),
                                        (2000, 3879, None), (4090, 4090, 12)]:
            with self.assertRaises(RuntimeError):
                target_count(current, neutral, delta, 0, 4095)

    def test_manual_limits_are_withheld_for_wrap_and_same_side(self):
        for counts, warning in [((3880, 3610, 50), 'count_wrap_between_neutral_and_endpoint'),
                                ((2000, 1800, 1900), 'endpoints_do_not_show_distinct_opposite_directions'),
                                ((2000, 1700, 2300), 'measurement_outside_servo_command_bounds')]:
            points = {name: {'shaft_count': count} for name, count in
                      zip(('straight', 'left_max', 'right_max'), counts)}
            summary = summarize_manual(points, 1750, 4095)
            self.assertIn(warning, summary['issues'])
            self.assertIsNone(summary['measured_count_interval'])
            self.assertIsNone(summary['count_direction_per_cad_positive'])
        points = {name: {'shaft_count': count} for name, count in
                  zip(('straight', 'left_max', 'right_max'), (2000, 2200, 1790))}
        self.assertEqual(summarize_manual(points, 0, 4095)['count_direction_per_cad_positive'], -1)


@unittest.skipUnless(sys.platform.startswith('linux'), 'requires Linux PTY')
class SerialPathTests(unittest.TestCase):
    def test_read_zero_jog_and_fault_release(self):
        import pty
        import tty
        master, slave = pty.openpty()
        tty.setraw(slave)
        self.addCleanup(os.close, master)
        self.addCleanup(os.close, slave)
        config = bytes.fromhex('0309000903010000010000ff0f468c32e8030c2c2f20200000000101e6000100000014c8320afa0a')
        mem = bytearray(73)
        mem[:40] = config
        mem[48:50] = struct.pack('<H', 1000)
        current = 3879  # deliberately synthetic; not an installed calibration
        armed = False
        fail_move = False
        writes = []
        arms = []
        running = threading.Event()
        running.set()

        def relay():
            nonlocal armed, current
            pending = b''
            while running.is_set():
                if not select.select([master], [], [], .05)[0]:
                    continue
                pending += os.read(master, 4096)
                while b'\n' in pending:
                    raw, pending = pending.split(b'\n', 1)
                    line = raw.decode()
                    response = {'boot': 10}
                    if line == 'INFO':
                        response.update(event='info', firmware='servo_probe_v1', motor_pwm=0,
                                        motor_dir=0, tx_gpio=17, rx_gpio=18, armed=armed)
                    elif line == 'B 1000000':
                        response.update(event='baud')
                    elif line.startswith('ARM '):
                        arms.append(line)
                        armed = not armed and int(line.split()[2]) == current
                        response.update(event='armed' if armed else 'rejected')
                    elif line == 'X':
                        if armed:
                            mem[40] = 0
                        armed = False
                        response.update(event='disarmed')
                    else:
                        sent = bytes.fromhex(line.removeprefix('Q '))
                        instruction, address = sent[4:6]
                        payload = b''
                        if instruction == 2:
                            mem[56:58] = struct.pack('<H', current)
                            mem[62:64] = bytes((121, 29))
                            payload = mem[address:address+sent[6]]
                        else:
                            assert instruction == 3 and armed and address in (40, 41, 48)
                            payload_in = sent[6:-1]
                            writes.append((address, payload_in))
                            mem[address:address+len(payload_in)] = payload_in
                            if address == 41 and mem[40]:
                                current = struct.unpack('<H', mem[42:44])[0]
                                if fail_move:
                                    mem[65] = 32
                        response.update(event='reply', rx=(sent + packet(1, 0, payload)).hex(),
                                        begin_us=1, end_us=2, io_error=False)
                    os.write(master, (json.dumps(response) + '\n').encode())

        thread = threading.Thread(target=relay, daemon=True)
        thread.start()
        self.addCleanup(thread.join, 1)
        self.addCleanup(running.clear)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            profile = folder / 'straight.json'

            def call(label, *args, rc=0):
                log = folder / label
                process = subprocess.run([sys.executable, str(ROOT / 'scripts/mounted_servo_setup.py'),
                                          '--port', os.ttyname(slave), '--log', str(log), *args],
                                         capture_output=True, text=True, timeout=12)
                self.assertEqual(process.returncode, rc, process.stdout + process.stderr)
                return json.loads((log / 'summary.json').read_text())

            manual_args = ['measure', '--assembly-id', 'pty', '--wheels-raised', '--motor-power-disconnected']
            # No hand movement should be requested while torque is enabled.
            mem[40] = 1
            self.assertFalse(call('torque-on', *manual_args, rc=2)['completed'])
            mem[40] = 0

            log = folder / 'manual'
            proc = subprocess.Popen([sys.executable, '-u', str(ROOT / 'scripts/mounted_servo_setup.py'),
                                     '--port', os.ttyname(slave), '--log', str(log), *manual_args],
                                    stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)

            def wait_for(prompt):
                output = b''
                while prompt.encode() not in output:
                    self.assertTrue(select.select([proc.stdout], [], [], 5)[0], output)
                    chunk = os.read(proc.stdout.fileno(), 4096)
                    self.assertTrue(chunk, output)
                    output += chunk

            def respond(line):
                proc.stdin.write(line.encode() + b'\n')
                proc.stdin.flush()

            try:
                wait_for('앞바퀴 직진:')
                current = 2050
                respond('')
                wait_for('차량 왼쪽 최대:')
                current = 1780
                respond('')
                wait_for('각도계로')
                respond('29 23')
                wait_for('차량 오른쪽 최대:')
                current = 2310
                respond('')
                wait_for('각도계로')
                respond('22 28')
                output, _ = proc.communicate(timeout=5)
                self.assertEqual(proc.returncode, 0, output)
            finally:
                if proc.poll() is None:
                    proc.kill()
                    proc.communicate()
            measured = json.loads((log / 'summary.json').read_text())
            self.assertEqual(measured['torque_after'], 0)
            self.assertEqual(measured['measured_count_interval'], [1780, 2310])
            self.assertEqual(measured['manual_points']['right_max']['wheel_angle_deg'],
                             {'left_wheel': -22, 'right_wheel': -28})
            self.assertEqual(measured['manual_points']['left_max']['shaft_delta_deg_from_straight'], -23.73)
            self.assertFalse(measured['operating_limits_confirmed'])
            self.assertEqual(json.loads((log / 'straight.json').read_text())['zero_count'], 2050)
            self.assertEqual((arms, writes), ([], []))
            current = 3879

            zero = call('zero', 'zero', '--assembly-id', 'pty', '--wheels-straight',
                        '--evidence', 'Synthetic PTY', '--output', str(profile))
            self.assertEqual(zero['position_counts'], 3879)
            self.assertEqual(json.loads(profile.read_text())['status'], 'zero_recorded_pending_mount_checks')
            self.assertEqual(writes, [])
            move = ['jog', '--profile', str(profile), '--wheels-raised', '--motor-power-disconnected']
            result = call('jog', *move, '--counts=12')
            self.assertEqual((result['position_before_release'], result['torque_after']), (3891, 0))
            self.assertEqual(writes[:4], [(48, struct.pack('<H', 300)),
                                         (41, struct.pack('<BHHH', 5, 3879, 0, 100)),
                                         (40, b'\1'), (41, struct.pack('<BHHH', 5, 3891, 0, 100))])
            fail_move = True
            result = call('failure', *move, '--toward-zero', rc=2)
            self.assertFalse(result['completed'])
            self.assertEqual(result['torque_after'], 0)
            self.assertEqual((mem[40], armed), (0, False))
            self.assertEqual(mem[:40], config)


if __name__ == '__main__':
    unittest.main()
