"""PC byte stream only. tty/serial/ROS/실물 MCU를 열지 않는다."""
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time

import pytest

from actuation_raw_capture import capture, audit, Limits, FdSource, open_regular, encoded
from arena_vehicle_interface.actuation_wire import encode
from test_actuation_host_observer import offer, state
from audit_actuation_tx_policy import compile_driver
from test_actuation_tx_policy import Mcu

ROOT = Path(__file__).resolve().parents[1]
CLI = ROOT/'scripts/actuation_raw_capture.py'


class Source:
    kind = 'synthetic'

    def __init__(self, data=b'', *, split=13, error=None):
        self.data = data
        self.split = split
        self.error = error
        self.calls = 0

    def read(self, size, wait):
        self.calls += 1
        if self.error:
            raise self.error
        count = min(size, self.split, len(self.data))
        data, self.data = self.data[:count], self.data[count:]
        return data


class Clock:
    def __init__(self, step=10):
        self.now = 1000
        self.step = step

    def __call__(self):
        value = self.now
        self.now += self.step
        return value


def record(tmp_path, data=b'', **options):
    output = tmp_path/'capture'
    result = capture(Source(data), output, clock=Clock(), origin='synthetic', **options)
    return output, result


def test_split_combined_duplicate_boot_and_partial_preserved(tmp_path):
    frames = [offer(), offer(), state(boot='boot-B')]
    data = b'console text\r\n' + b''.join(encode(p) for p in frames) + encode(state())[:17]
    folder, result = record(tmp_path, data)
    assert result['completed'] and (folder/'raw.bin').read_bytes() == data
    report = audit(folder)
    assert report['record_integrity_valid'] and report['capture_completed'] and report['timing_valid']
    assert report['decoded_messages'] == report['schema_accepted'] == 3
    assert report['adjacent_duplicate_messages'] == report['observed_boot_changes'] == 1
    assert report['decoder_pending_bytes'] == 17 and report['decoder_errors'] >= 1
    for flag in ('host_observer_applied', 'frame_offsets_available', 'upstream_loss_checked',
                 'complete_capture_verified', 'timestamps_are_sensor_time', 'firmware_verified', 'ready_to_drive'):
        assert report[flag] is False


def test_crc_json_schema_and_noise_differ_from_artifact_integrity(tmp_path):
    bad = bytearray(encode(offer()))
    bad[-1] ^= 0x40
    data = b'unframed console only' + bytes(bad) + encode({'unknown': True}) + encode(offer())
    folder, _ = record(tmp_path, data)
    report = audit(folder)
    assert report['record_integrity_valid'] and report['decoder_errors'] > 0
    assert report['schema_rejected'] == report['schema_accepted'] == 1
    assert report['examples'][0]['kind'] == 'schema_rejected'


@pytest.mark.parametrize('limits,reason', [
    (Limits(max_bytes=30, chunk_bytes=9), 'byte_limit'),
    (Limits(max_chunks=2, chunk_bytes=10), 'chunk_limit'),
    (Limits(duration_ns=10), 'time_limit'),
])
def test_limits_are_closed_incomplete_not_eof(tmp_path, limits, reason):
    folder, result = record(tmp_path, b'x'*200, limits=limits)
    report = audit(folder)
    assert not result['completed'] and report['stop_reason'] == reason
    assert report['record_integrity_valid'] and not report['capture_completed']
    assert report['raw_bytes'] <= limits.max_bytes and report['chunks'] <= limits.max_chunks


@pytest.mark.parametrize('exc,reason', [(KeyboardInterrupt(), 'interrupted'), (OSError('injected'), 'read_error')])
def test_read_failures_keep_closed_incomplete_record(tmp_path, exc, reason):
    folder = tmp_path/'capture'
    capture(Source(error=exc), folder, clock=Clock(), origin='synthetic')
    report = audit(folder)
    assert not report['capture_completed'] and report['stop_reason'] == reason


def test_clock_equal_valid_regression_raw_is_not_deleted(tmp_path):
    good = tmp_path/'equal'
    capture(Source(b'123'), good, clock=Clock(0), origin='synthetic')
    assert audit(good)['timing_valid']
    times = iter([1000, 1100, 1099])
    bad = tmp_path/'regress'
    capture(Source(b'abc'), bad, clock=lambda: next(times), origin='synthetic')
    report = audit(bad)
    assert report['record_integrity_valid'] and not report['timing_valid']
    assert report['stop_reason'] == 'clock_error' and (bad/'raw.bin').read_bytes() == b'abc'


@pytest.mark.parametrize('part', ['raw', 'metadata', 'footer', 'marker', 'missing_marker', 'missing_footer',
                                 'raw_tail', 'meta_tail', 'duplicate_chunk', 'offset', 'swap'])
def test_artifact_damage_rejected(tmp_path, part):
    folder, _ = record(tmp_path, encode(offer()))
    raw, meta, marker = folder/'raw.bin', folder/'chunks.jsonl', folder/'closed.json'
    lines = meta.read_bytes().splitlines(keepends=True)
    if part == 'raw':
        raw.write_bytes(b'X'+raw.read_bytes()[1:])
    elif part == 'metadata':
        meta.write_bytes(b' '.join(lines))
    elif part == 'footer':
        footer = json.loads(lines[-1]); footer['reason'] = 'interrupted'; footer['completed'] = False
        meta.write_bytes(b''.join(lines[:-1])+encoded(footer))
    elif part == 'marker':
        marker.write_bytes(marker.read_bytes()[:-2])
    elif part == 'missing_marker':
        marker.unlink()
    elif part == 'missing_footer':
        meta.write_bytes(b''.join(lines[:-1]))
    elif part == 'raw_tail':
        raw.write_bytes(raw.read_bytes()+b'X')
    elif part == 'meta_tail':
        meta.write_bytes(meta.read_bytes()+b'X')
    elif part == 'duplicate_chunk':
        meta.write_bytes(b''.join(lines[:2]+[lines[1]]+lines[2:]))
    elif part == 'offset':
        row = json.loads(lines[1]); row['offset'] = 1; lines[1] = encoded(row)
        meta.write_bytes(b''.join(lines))
    else:
        other = tmp_path/'other'
        capture(Source(encode(state())), other, clock=Clock(), origin='synthetic')
        marker.write_bytes((other/'closed.json').read_bytes())
    with pytest.raises((OSError, ValueError, TypeError)):
        audit(folder)


def test_output_exists_is_never_replaced(tmp_path):
    folder, _ = record(tmp_path, b'unchanged')
    original = {p.name: p.read_bytes() for p in folder.iterdir()}
    with pytest.raises(FileExistsError):
        capture(Source(b'bad'), folder, origin='synthetic')
    assert original == {p.name: p.read_bytes() for p in folder.iterdir()}


class FailingFile:
    def __init__(self, path, *, limit=0):
        self.stream = path.open('xb', buffering=0)
        self.remaining = limit

    def __enter__(self): return self
    def __exit__(self, *args): self.stream.close()
    def write(self, data):
        if not self.remaining: raise OSError('injected disk full')
        n = self.stream.write(data[:self.remaining]); self.remaining -= n
        return n
    def flush(self): self.stream.flush()
    def fileno(self): return self.stream.fileno()


@pytest.mark.parametrize('target', ['raw.bin', 'chunks.jsonl', 'closed.json'])
def test_partial_write_failure_has_no_valid_completion(tmp_path, target):
    folder = tmp_path/'capture'
    def opener(p):
        return FailingFile(p, limit=5) if p.name == target else p.open('xb', buffering=0)
    with pytest.raises(OSError):
        capture(Source(b'0123456789'), folder, origin='synthetic', clock=Clock(), open_file=opener)
    with pytest.raises((OSError, ValueError)):
        audit(folder)


def test_idle_pipe_times_out_without_waiting_for_eof(tmp_path):
    read_fd, write_fd = os.pipe()
    try:
        start = time.monotonic()
        capture(FdSource(read_fd), tmp_path/'idle', Limits(duration_ns=50_000_000))
        assert time.monotonic()-start < 2
        assert audit(tmp_path/'idle')['stop_reason'] == 'time_limit'
    finally:
        os.close(read_fd); os.close(write_fd)


def test_cli_regular_file_and_empty_pipe(tmp_path):
    src = tmp_path/'fixture.bin'; src.write_bytes(encode(offer()))
    run = subprocess.run([sys.executable, str(CLI), 'capture', '--input', str(src), '--output', str(tmp_path/'file')],
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 0, run.stdout+run.stderr
    check = subprocess.run([sys.executable, str(CLI), 'audit', str(tmp_path/'file')],
                           capture_output=True, text=True, timeout=5)
    assert check.returncode == 0 and json.loads(check.stdout)['schema_accepted'] == 1
    run = subprocess.run([sys.executable, str(CLI), 'capture', '--stdin', '--output', str(tmp_path/'pipe')],
                         input=b'', capture_output=True, timeout=5)
    assert run.returncode == 0 and audit(tmp_path/'pipe')['raw_bytes'] == 0


def test_rejects_fifo_and_symlink_without_read_open(tmp_path):
    fifo = tmp_path/'fifo'; os.mkfifo(fifo)
    src = tmp_path/'data'; src.write_bytes(b'x')
    link = tmp_path/'link'; link.symlink_to(src)
    for path in (fifo, link):
        with pytest.raises(ValueError): open_regular(path)


def test_source_bounds_and_kind(tmp_path):
    with pytest.raises(ValueError):
        capture(Source(), tmp_path/'invalid_kind')
    class Over(Source):
        def read(self, n, wait): return b'x'*(n+1)
    capture(Over(), tmp_path/'over', origin='synthetic')
    assert audit(tmp_path/'over')['stop_reason'] == 'read_error'


@pytest.mark.parametrize('limits', [Limits(max_bytes=True), Limits(max_chunks=100001), Limits(chunk_bytes=0),
                                  Limits(duration_ns=0), Limits(max_bytes=16*1024*1024+1)])
def test_invalid_limits_create_nothing(tmp_path, limits):
    with pytest.raises(ValueError): record(tmp_path, limits=limits)
    assert not (tmp_path/'capture').exists()


def test_real_cpp_sender_bytes_can_be_replayed_without_serial(tmp_path):
    if not shutil.which('g++'): pytest.skip('g++ required')
    binary = compile_driver(tmp_path/'tx')
    mcu = Mcu(binary)
    try:
        mcu.cmd('C 0')
        mcu.cmd('T 0')
        data = bytes.fromhex(mcu.pump(0)['written'])
    finally:
        mcu.close()
    assert data
    folder, _ = record(tmp_path, data)
    report = audit(folder)
    assert report['schema_accepted'] > 0 and report['schema_rejected'] == 0
    assert report['decoder_errors'] == 0 and report['decoder_pending_bytes'] == 0


def test_raw_files_are_locally_ignored(tmp_path):
    folder, _ = record(tmp_path)
    assert (folder/'.gitignore').read_text().splitlines() == ['raw.bin', 'chunks.jsonl', 'closed.json']


def test_has_no_serial_or_command_output_api():
    text = CLI.read_text()
    for api in ('import serial', 'serial.Serial', 'termios', 'setDTR', 'setRTS', 'os.write(', 'CommandProducer', 'HostControlSession'):
        assert api not in text
