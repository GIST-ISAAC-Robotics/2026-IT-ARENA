"""기존 독립 광선·회전/결측 경계 검사에 C++ 후보만 주입한다."""
from pathlib import Path
import sys
import pytest
from native_candidate import Kernel
from arena_autonomy import observation_support as support

root = Path(__file__).resolve().parents[4]
kernel = Kernel(sys.argv[1])
support.depth_support = kernel.function_variant()
tests = ['test_observation_depth_chunk.py','test_observation_depth_cone.py',
    'test_observation_depth_equivalence.py','test_observation_depth_window.py',
    'test_observation_support.py','test_native_measurement_motion.py','test_scan_topology.py']
code = pytest.main(['-q', *[str(root/'tests'/name) for name in tests]])
print(dict(native_calls=kernel.calls, native_rows=kernel.rows, numpy_fallback_rows=kernel.fallback_rows))
raise SystemExit(code)
