"""오프라인 기록 자원 한도와 기록기 조기 종료 감시. 차량 보호 한도가 아니다."""
import math

DEFAULT_MAX_RAW_GIB = 6.
DEFAULT_MAX_WALL_SECONDS = 1200.


def validate_limits(max_raw_gib, max_wall_seconds):
    if (not math.isfinite(max_raw_gib) or not .01 <= max_raw_gib <= 20.
            or not math.isfinite(max_wall_seconds) or not 1. <= max_wall_seconds <= 7200.):
        raise ValueError('recording limits out of range: .01..20 GiB, 1..7200 seconds')


def recorder_arguments(max_raw_gib, max_wall_seconds):
    validate_limits(max_raw_gib, max_wall_seconds)
    return ['--max-raw-gib', str(max_raw_gib), '--max-wall-seconds', str(max_wall_seconds)]


def require_recorder_running(process):
    """예정된 기록 종료 전에는 종료 코드 0도 입력 유실 경계로 취급한다."""
    if process is not None:
        code = process.poll()
        if code is not None:
            raise RuntimeError(f'A2 recorder exited before requested stop (code={code}); see recorder.log/manifest.json')
