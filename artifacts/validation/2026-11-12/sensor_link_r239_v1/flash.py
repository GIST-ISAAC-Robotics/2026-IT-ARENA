"""사용자 무전력 확인 후 해시가 고정된 출력 비활성 앱을 설치/복구한다. 구동 명령 없음."""
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys

TARGET = Path(sys.argv[2])
PYTHON = '/home/q/it_arena_imu_20261111/tools_venv/bin/python'
PORT = '/dev/serial/by-id/usb-1a86_USB_Single_Serial_5CE5031468-if00'
EXPECTED = sys.argv[3]


def main():
    out = Path(sys.argv[1])
    out.mkdir(parents=True, exist_ok=False)
    sha = lambda path: hashlib.sha256(path.read_bytes()).hexdigest()
    report = dict(user_confirmed_motor_disconnected_and_no_battery=True, actuation_commands=0,
                  firmware=TARGET.name, app_only=True, complete=False)
    def command(label, *parts):
        with (out/(label+'-private.log')).open('xb') as stream:
            subprocess.run([PYTHON, '-m', 'esptool', '--chip', 'esp32s3', '--port', PORT,
                            '--baud', '921600', *parts], check=True, timeout=90,
                           stdout=stream, stderr=subprocess.STDOUT)
    try:
        assert sha(TARGET) == EXPECTED and 0 < TARGET.stat().st_size <= 0x100000
        assert str(Path(PORT).resolve()) == '/dev/ttyACM0'
        ownership = subprocess.run(['fuser', '/dev/ttyACM0'], capture_output=True)
        if ownership.returncode != 1:
            raise RuntimeError('serial port is owned or ownership check failed')
        command('partition', 'read-flash', '0x8000', '0x1000', str(out/'partition-private.bin'))
        table = (out/'partition-private.bin').read_bytes()
        app = []
        for offset in range(0, len(table), 32):
            magic, kind, subtype, start, size, label, flags = struct.unpack('<HBBII16sI', table[offset:offset+32])
            if magic == 0xffff:
                break
            if magic == 0xebeb:
                continue
            assert magic == 0x50aa, 'invalid partition entry'
            if kind == 0:
                app.append((subtype, start, size, label.rstrip(b'\0').decode()))
        assert app == [(0, 0x10000, 0x100000, 'factory')], app
        report['partition'] = app
        command('backup', 'read-flash', '0x10000', '0x100000', str(out/'before-app-private.bin'))
        report['before_app_sha256'] = sha(out/'before-app-private.bin')
        report['partition_sha256'] = sha(out/'partition-private.bin')
        command('flash', 'write-flash', '0x10000', str(TARGET))
        command('readback', 'read-flash', '0x10000', str(TARGET.stat().st_size), str(out/'readback-private.bin'))
        report['target_sha256'] = sha(TARGET)
        report['readback_sha256'] = sha(out/'readback-private.bin')
        assert report['readback_sha256'] == EXPECTED
        report['complete'] = True
    except Exception as error:
        report['error'] = type(error).__name__+': '+str(error)
        raise
    finally:
        (out/'restore_summary.json').write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(report), flush=True)

if __name__ == '__main__':
    main()
