"""같은 UART의 IA 계약/IMU1 센서 프레임 분리. 프레임 본문을 다른 프로토콜로 재해석하지 않는다."""
from collections import Counter
import struct
import zlib

from arena_vehicle_interface.actuation_wire import MAX_PAYLOAD


class SensorLinkDecoder:
    def __init__(self):
        self.buffer = bytearray()
        self.counters = Counter()

    def feed(self, data):
        if len(data) > 65536:
            self.reset()
            self.counters['input_limit'] += 1
            return []
        self.buffer.extend(data)
        result = []
        while self.buffer:
            starts = [p for p in (self.buffer.find(b'IA'), self.buffer.find(b'IMU1')) if p >= 0]
            if not starts:
                # 분할된 magic 접두부만 보존한다.
                keep = next((n for n in (3, 2, 1) if any(m.startswith(self.buffer[-n:])
                             for m in (b'IA', b'IMU1')) and len(self.buffer) >= n), 0)
                self.counters['nonframe_bytes'] += len(self.buffer)-keep
                self.buffer[:] = self.buffer[-keep:] if keep else b''
                break
            start = min(starts)
            self.counters['nonframe_bytes'] += start
            del self.buffer[:start]
            imu = self.buffer.startswith(b'IMU1')
            header = 7 if imu else 4
            if len(self.buffer) < header:
                break
            size = struct.unpack_from('<H', self.buffer, 4 if imu else 2)[0]
            if (imu and (size != 68 or self.buffer[6] != 1)) or (not imu and not 1 <= size <= MAX_PAYLOAD):
                self.counters['invalid_header'] += 1
                del self.buffer[:2]
                continue
            total = 68 if imu else size+8
            if len(self.buffer) < total:
                break
            crc = struct.unpack_from('<I', self.buffer, total-4)[0]
            body = self.buffer[:total-4] if imu else self.buffer[2:total-4]
            if zlib.crc32(body) != crc:
                self.counters['crc_errors'] += 1
                del self.buffer[:2]
                continue
            packet = bytes(self.buffer[:total])
            del self.buffer[:total]
            kind = 'imu' if imu else 'control'
            self.counters[kind] += 1
            result.append((kind, packet))
        return result

    def reset(self):
        # 재연결/호스트 조립 만료에서 호출. 이전 조각을 새 boot에 이어 붙이지 않는다.
        self.buffer.clear()
