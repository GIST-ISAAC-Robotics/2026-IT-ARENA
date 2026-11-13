"""오프라인 시험의 package search overlay. 공통 이관 대상이 아니다."""
from pathlib import Path
from pkgutil import extend_path

_artifact = Path(__file__).resolve().parents[2]
__path__ = [str(_artifact / 'staged/src/arena_autonomy/arena_autonomy'),
            *extend_path(__path__, __name__)]
