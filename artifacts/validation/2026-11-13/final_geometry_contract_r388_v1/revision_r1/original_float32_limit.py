"""r377 launch 송신 한계 원식. 진단 하네스 전용 보존 사본."""
import math
import struct


def _float32_limit_not_above(limit):
    """Ackermann float32 반올림으로 물리/MCU 상한을 넘지 않는 송신 상한."""
    if not math.isfinite(limit) or limit <= 0:
        raise ValueError('Command limit must be finite and positive')
    try:
        packed = struct.pack('<f', limit)
    except OverflowError as error:
        raise ValueError('Command limit cannot be represented as float32') from error
    rounded = struct.unpack('<f', packed)[0]
    if rounded > limit:
        bits = struct.unpack('<I', packed)[0]
        rounded = struct.unpack('<f', struct.pack('<I', bits-1))[0]
    if rounded <= 0:
        raise ValueError('Command limit is below positive float32 range')
    return rounded
