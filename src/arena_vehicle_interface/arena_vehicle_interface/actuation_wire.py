"""프로토타입 직렬 프레임: magic + 길이 + UTF-8 JSON + CRC32. 인증 기능 없음."""
import json
import struct
import zlib

MAGIC = b"IA"
MAX_PAYLOAD = 1536


def _pairs(items):
    result = {}
    for key, value in items:
        if key in result:
            raise ValueError("duplicate JSON key")
        result[key] = value
    return result


def _constant(value):
    raise ValueError(f"invalid JSON number {value}")


def encode(message):
    if type(message) is not dict:
        raise ValueError("object required")
    payload = json.dumps(message, allow_nan=False, separators=(",", ":"), ensure_ascii=True).encode("ascii")
    if not 1 <= len(payload) <= MAX_PAYLOAD:
        raise ValueError("frame length")
    body = struct.pack("<H", len(payload)) + payload
    return MAGIC + body + struct.pack("<I", zlib.crc32(body))


class Decoder:
    def __init__(self):
        self.buffer = bytearray()
        self.errors = 0

    def feed(self, data):
        """유한 크기 바이트 입력에서 분할/합쳐진 프레임을 복구한다. 호출자도 수신량을 제한한다."""
        if len(data) > 65536:
            self.buffer.clear()
            self.errors += 1
            return []
        self.buffer.extend(data)
        result = []
        while self.buffer:
            start = self.buffer.find(MAGIC)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == MAGIC[:1] else b""
                break
            if start:
                del self.buffer[:start]
                self.errors += 1
            if len(self.buffer) < 4:
                break
            size = struct.unpack_from("<H", self.buffer, 2)[0]
            if not 1 <= size <= MAX_PAYLOAD:
                del self.buffer[:2]
                self.errors += 1
                continue
            total = size + 8
            if len(self.buffer) < total:
                break
            crc = struct.unpack_from("<I", self.buffer, 4 + size)[0]
            if zlib.crc32(self.buffer[2:4 + size]) != crc:
                del self.buffer[:2]
                self.errors += 1
                continue
            payload = bytes(self.buffer[4:4 + size])
            del self.buffer[:total]
            try:
                message = json.loads(payload.decode("utf-8"), object_pairs_hook=_pairs, parse_constant=_constant)
                if type(message) is not dict:
                    raise ValueError("object required")
                result.append(message)
            except (ValueError, UnicodeError, RecursionError):
                self.errors += 1
        return result

    def reset(self):
        """재연결 때 이전 부분 프레임을 새 세션에 이어 붙이지 않는다."""
        self.buffer.clear()
