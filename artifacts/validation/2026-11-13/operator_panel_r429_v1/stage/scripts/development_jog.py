#!/usr/bin/env python3
"""기존 CLI 호환 진입점. 패널과 동일한 제한 jog/회수 구현을 사용한다."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]/'src'/'arena_autonomy'))
from arena_autonomy import development_jog as _implementation

if __name__ == '__main__':
    _implementation.main()
else:
    sys.modules[__name__] = _implementation
