#!/usr/bin/env python3
"""명령 없는 PC 바이트 기록·오프라인 감사. serial/ROS 드라이버나 구동 허가기가 아니다."""
import argparse
from collections import Counter
from dataclasses import asdict, dataclass
import hashlib
import json
import os
from pathlib import Path
import select
import stat
import time
import uuid

from arena_vehicle_interface.actuation_wire import Decoder, _pairs, _constant
from actuation_host_observer import validate_payload

SCHEMA = 'arena_actuation_raw_v1'
MAX_BYTES = 16 * 1024 * 1024
MAX_CHUNKS = 100_000
MAX_LINE = 4096
REASONS = {'eof', 'time_limit', 'byte_limit', 'chunk_limit', 'interrupted', 'read_error', 'clock_error'}


def integer(value, low=0, high=2**63-1):
    if type(value) is not int or not low <= value <= high:
        raise ValueError('bounded integer required')
    return value


def exact(row, keys):
    if type(row) is not dict or set(row) != set(keys):
        raise ValueError('exact object fields required')


def encoded(row):
    return (json.dumps(row, allow_nan=False, separators=(',', ':')) + '\n').encode('ascii')


def write_all(stream, data):
    view = memoryview(data)
    while view:
        count = stream.write(view)
        if type(count) is not int or not 0 < count <= len(view):
            raise OSError('incomplete artifact write')
        view = view[count:]


def open_regular(path):
    # Linux/WSL 전용: O_PATH로 대상을 고정하고 regular 판별 뒤 같은 inode를 읽는다.
    # 먼저 read-open한 뒤 fstat하면 장치 open 자체가 부작용을 만들 수 있다.
    if not hasattr(os, 'O_PATH'):
        raise ValueError('Linux/WSL O_PATH is required; no fallback device open')
    fd = os.open(path, os.O_PATH | os.O_NOFOLLOW)
    try:
        if not stat.S_ISREG(os.fstat(fd).st_mode):
            raise ValueError('regular artifact/input file required, no links/devices')
        return os.fdopen(os.open(f'/proc/self/fd/{fd}', os.O_RDONLY), 'rb')
    finally:
        os.close(fd)


@dataclass(frozen=True)
class Limits:
    max_bytes: int = 1024 * 1024
    max_chunks: int = 10_000
    chunk_bytes: int = 4096
    duration_ns: int = 60_000_000_000

    def validate(self):
        integer(self.max_bytes, 1, MAX_BYTES)
        integer(self.max_chunks, 1, MAX_CHUNKS)
        integer(self.chunk_bytes, 1, 65536)
        integer(self.duration_ns, 1, 600_000_000_000)


class FdSource:
    """소유자가 독점해서 읽는 일반 파일 또는 이미 연결된 pipe만. 포트를 열지 않는다."""
    def __init__(self, fd):
        mode = os.fstat(fd).st_mode
        if not (stat.S_ISREG(mode) or stat.S_ISFIFO(mode)):
            raise ValueError('only a regular file or existing pipe is accepted; no terminal/device')
        self.fd = fd
        self.kind = 'regular_file' if stat.S_ISREG(mode) else 'pipe'

    def read(self, size, wait_seconds):
        if self.kind == 'pipe' and not select.select([self.fd], [], [], wait_seconds)[0]:
            return None
        # 단일 reader 전제. 일반 파일의 filesystem read 지연은 강제 중단하지 못한다.
        return os.read(self.fd, size)


def capture(source, output, limits=Limits(), *, clock=time.monotonic_ns, origin='pc_stream',
            open_file=None):
    """bounded source.read -> bytes/None(idle)/b''(EOF). 쓰기 실패는 footer 없이 원본을 남긴다."""
    limits.validate()
    if origin not in {'pc_stream', 'synthetic'} or source.kind not in {'regular_file', 'pipe', 'synthetic'}:
        raise ValueError('explicit supported source/origin required')
    if origin == 'pc_stream' and source.kind == 'synthetic':
        raise ValueError('synthetic source needs synthetic origin')
    output = Path(output)
    output.mkdir(exist_ok=False)
    with (output/'.gitignore').open('x', encoding='ascii') as ignore:
        ignore.write('raw.bin\nchunks.jsonl\nclosed.json\n')
    opener = open_file or (lambda p: p.open('xb', buffering=0))
    capture_id = str(uuid.uuid4())
    start = integer(clock())
    header = dict(kind='header', schema=SCHEMA, origin=origin, source_kind=source.kind,
                  capture_id=capture_id, host_epoch='process-'+capture_id,
                  connection_id='capture-'+capture_id, clock='time.monotonic_ns',
                  start_ns=start, limits=asdict(limits), commands_sent=False,
                  timing='host_read_call_window_not_line_or_sensor_time')
    raw_hash, meta_hash = hashlib.sha256(), hashlib.sha256()
    count = offset = 0
    last = start
    end = start
    reason = 'read_error'
    with opener(output/'raw.bin') as raw, opener(output/'chunks.jsonl') as meta:
        line = encoded(header)
        write_all(meta, line)
        meta_hash.update(line)
        while True:
            # byte/chunk 상한에서는 EOF 확인용 추가 read를 하지 않는다. 정확히 끝나도 limit로 기록한다.
            if offset >= limits.max_bytes:
                reason = 'byte_limit'
                break
            if count >= limits.max_chunks:
                reason = 'chunk_limit'
                break
            try:
                before = integer(clock())
                end = before
                if before < last:
                    reason = 'clock_error'
                    break
                if before-start >= limits.duration_ns:
                    reason = 'time_limit'
                    break
                want = min(limits.chunk_bytes, limits.max_bytes-offset)
                data = source.read(want, min(0.1, (limits.duration_ns-(before-start))/1e9))
                if data is not None and (type(data) is not bytes or len(data) > want):
                    raise OSError('source violated bounded read')
                after = integer(clock())
                end = after
            except KeyboardInterrupt:
                reason = 'interrupted'
                break
            except OSError:
                reason = 'read_error'
                break
            if data:
                # 중간에 시계가 역행해도 이미 읽은 raw를 지우지 않는다. 감사가 timing_valid=false로 구별한다.
                write_all(raw, data)
                raw_hash.update(data)
                row = dict(kind='chunk', seq=count, offset=offset, length=len(data),
                           before_ns=before, after_ns=after, sha256=hashlib.sha256(data).hexdigest())
                line = encoded(row)
                write_all(meta, line)
                meta_hash.update(line)
                offset += len(data)
                count += 1
            if after < before:
                reason = 'clock_error'
                break
            last = after
            if data == b'':
                reason = 'eof'
                break
        # 정상 파일 저장의 순서 보장일 뿐 전원 차단 원자성·무결성 인증은 아니다.
        raw.flush()
        os.fsync(raw.fileno())
        footer = dict(kind='footer', reason=reason, completed=reason == 'eof', chunks=count,
                      bytes=offset, end_ns=end, raw_sha256=raw_hash.hexdigest(),
                      metadata_sha256=meta_hash.hexdigest())
        line = encoded(footer)
        write_all(meta, line)
        meta_hash.update(line)
        meta.flush()
        os.fsync(meta.fileno())
    # footer를 포함한 metadata의 accidental corruption도 검사한다. 인증/변조방지 서명은 아니다.
    with opener(output/'closed.json') as closed:
        write_all(closed, encoded(dict(schema=SCHEMA, capture_id=capture_id,
                                      raw_sha256=raw_hash.hexdigest(), metadata_sha256=meta_hash.hexdigest())))
        closed.flush()
        os.fsync(closed.fileno())
    return dict(**footer, record_closed=True, ready_to_drive=False, firmware_verified=False)


def load_line(stream):
    data = stream.readline(MAX_LINE+1)
    if not data or len(data) > MAX_LINE or not data.endswith(b'\n'):
        raise ValueError('missing, truncated or oversized metadata line')
    return json.loads(data.decode('ascii'), object_pairs_hook=_pairs, parse_constant=_constant), data


def digest_text(value):
    if type(value) is not str or len(value) != 64 or any(c not in '0123456789abcdef' for c in value):
        raise ValueError('SHA-256 hex required')


def audit(folder, *, max_lease_us=100_000):
    """원본/metadata를 streaming 대조. 모든 관측은 저장 구간에 한정하며 허가/인증을 만들지 않는다."""
    integer(max_lease_us, 1)
    root = Path(folder)
    raw_hash, meta_hash = hashlib.sha256(), hashlib.sha256()
    decoder = Decoder()
    count = offset = message_count = schema_accepted = schema_rejected = 0
    adjacent_duplicates = boot_changes = 0
    kinds = Counter()
    examples = []
    previous_message = previous_boot = None
    # 중단/잘린 기록은 예외로 거절한다. CRC 오류가 있는 완전한 기록과 파일 손상을 구별한다.
    with open_regular(root/'raw.bin') as raw, open_regular(root/'chunks.jsonl') as meta:
        header, line = load_line(meta)
        exact(header, {'kind', 'schema', 'origin', 'source_kind', 'capture_id', 'host_epoch', 'connection_id',
                       'clock', 'start_ns', 'limits', 'commands_sent', 'timing'})
        if (header['kind'] != 'header' or header['schema'] != SCHEMA
                or header['origin'] not in {'pc_stream', 'synthetic'}
                or header['source_kind'] not in {'regular_file', 'pipe', 'synthetic'}
                or (header['source_kind'] == 'synthetic' and header['origin'] != 'synthetic')
                or header['clock'] != 'time.monotonic_ns' or header['commands_sent'] is not False
                or header['timing'] != 'host_read_call_window_not_line_or_sensor_time'):
            raise ValueError('unsupported recording contract')
        if type(header['capture_id']) is not str or str(uuid.UUID(header['capture_id'])) != header['capture_id']:
            raise ValueError('capture UUID required')
        if header['host_epoch'] != 'process-'+header['capture_id'] or header['connection_id'] != 'capture-'+header['capture_id']:
            raise ValueError('capture context mismatch')
        exact(header['limits'], asdict(Limits()))
        limits = Limits(**header['limits'])
        limits.validate()
        start = last = integer(header['start_ns'])
        timing_valid = True
        meta_hash.update(line)
        while True:
            row, line = load_line(meta)
            if type(row) is not dict:
                raise ValueError('metadata object required')
            if row.get('kind') == 'footer':
                footer = row
                break
            exact(row, {'kind', 'seq', 'offset', 'length', 'before_ns', 'after_ns', 'sha256'})
            if row['kind'] != 'chunk' or integer(row['seq']) != count or integer(row['offset']) != offset:
                raise ValueError('chunk order/offset mismatch')
            n = integer(row['length'], 1, limits.chunk_bytes)
            if count >= limits.max_chunks or offset+n > limits.max_bytes:
                raise ValueError('recording exceeds declared limit')
            before, after = integer(row['before_ns']), integer(row['after_ns'])
            timing_valid = timing_valid and last <= before <= after
            last = after
            data = raw.read(n)
            digest_text(row['sha256'])
            if len(data) != n or hashlib.sha256(data).hexdigest() != row['sha256']:
                raise ValueError('raw chunk length/hash mismatch')
            meta_hash.update(line)
            raw_hash.update(data)
            for payload in decoder.feed(data):
                message_count += 1
                canonical = json.dumps(payload, sort_keys=True, separators=(',', ':'), ensure_ascii=True)
                fingerprint = hashlib.sha256(canonical.encode('ascii')).digest()
                adjacent_duplicates += fingerprint == previous_message
                previous_message = fingerprint
                try:
                    kind, _ = validate_payload(payload, max_lease_us)
                    schema_accepted += 1
                    kinds[kind] += 1
                    if kind != 'ack':
                        boot = payload['boot']
                        boot_changes += previous_boot is not None and boot != previous_boot
                        previous_boot = boot
                except (TypeError, ValueError, OverflowError, RecursionError):
                    schema_rejected += 1
                    if len(examples) < 8:
                        examples.append(dict(emitted_by_chunk=count, raw_chunk_offset=offset, kind='schema_rejected'))
            offset += n
            count += 1
        exact(footer, {'kind', 'reason', 'completed', 'chunks', 'bytes', 'end_ns', 'raw_sha256', 'metadata_sha256'})
        if (footer['reason'] not in REASONS or type(footer['completed']) is not bool
                or footer['completed'] != (footer['reason'] == 'eof')
                or integer(footer['chunks']) != count or integer(footer['bytes']) != offset):
            raise ValueError('footer mismatch')
        end = integer(footer['end_ns'])
        timing_valid = timing_valid and end >= last and footer['reason'] != 'clock_error'
        for key in ('raw_sha256', 'metadata_sha256'):
            digest_text(footer[key])
        if footer['raw_sha256'] != raw_hash.hexdigest() or footer['metadata_sha256'] != meta_hash.hexdigest():
            raise ValueError('whole record hash mismatch')
        if ((footer['reason'] == 'byte_limit' and offset != limits.max_bytes)
                or (footer['reason'] == 'chunk_limit' and count != limits.max_chunks)
                or (footer['reason'] == 'time_limit' and end-start < limits.duration_ns)
                or (footer['reason'] == 'eof' and (offset >= limits.max_bytes or count >= limits.max_chunks))):
            raise ValueError('stop reason and bounds disagree')
        meta_hash.update(line)  # footer도 closed.json의 해시에 포함
        if raw.read(1) or meta.read(1):
            raise ValueError('unexpected trailing record data')
    with open_regular(root/'closed.json') as closed:
        marker, _ = load_line(closed)
        exact(marker, {'schema', 'capture_id', 'raw_sha256', 'metadata_sha256'})
        if marker != dict(schema=SCHEMA, capture_id=header['capture_id'], raw_sha256=raw_hash.hexdigest(),
                          metadata_sha256=meta_hash.hexdigest()) or closed.read(1):
            raise ValueError('closure marker mismatch')
    return dict(schema=SCHEMA, record_integrity_valid=True, record_closed=True,
                capture_completed=footer['completed'], stop_reason=footer['reason'], timing_valid=timing_valid,
                raw_bytes=offset, chunks=count, duration_ns=end-start if timing_valid else None,
                decoded_messages=message_count, decoder_errors=decoder.errors,
                decoder_pending_bytes=len(decoder.buffer), schema_accepted=schema_accepted,
                schema_rejected=schema_rejected, message_kinds=dict(kinds),
                adjacent_duplicate_messages=adjacent_duplicates, observed_boot_changes=boot_changes,
                examples=examples, frame_offsets_available=False, host_observer_applied=False,
                upstream_loss_checked=False, timestamps_are_sensor_time=False,
                complete_capture_verified=False, firmware_verified=False, ready_to_drive=False)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    get = sub.add_parser('capture')
    src = get.add_mutually_exclusive_group(required=True)
    src.add_argument('--stdin', action='store_true')
    src.add_argument('--input', type=Path)
    get.add_argument('--output', type=Path, required=True)
    get.add_argument('--max-bytes', type=int, default=Limits.max_bytes)
    get.add_argument('--max-chunks', type=int, default=Limits.max_chunks)
    get.add_argument('--chunk-bytes', type=int, default=Limits.chunk_bytes)
    get.add_argument('--duration-ms', type=int, default=60000)
    check = sub.add_parser('audit')
    check.add_argument('folder', type=Path)
    args = parser.parse_args()
    opened = None
    try:
        if args.action == 'audit':
            report = audit(args.folder)
            result = 0 if report['capture_completed'] and report['timing_valid'] else 2
        else:
            limits = Limits(args.max_bytes, args.max_chunks, args.chunk_bytes, args.duration_ms*1_000_000)
            limits.validate()
            # 파일 모드는 O_PATH로 고정한 regular file만. 장치·FIFO read-open 없이 거절한다.
            if args.input is not None:
                opened = open_regular(args.input)
            source = FdSource(0 if opened is None else opened.fileno())
            report = capture(source, args.output, limits)
            result = 0 if report['completed'] else 2
        print(json.dumps(report, allow_nan=False))
        return result
    except (OSError, ValueError, TypeError, RecursionError) as error:
        print(json.dumps(dict(error=type(error).__name__, detail=str(error), record_integrity_valid=False,
                              ready_to_drive=False, firmware_verified=False)))
        return 1
    finally:
        if opened is not None:
            opened.close()


if __name__ == '__main__':
    raise SystemExit(main())
