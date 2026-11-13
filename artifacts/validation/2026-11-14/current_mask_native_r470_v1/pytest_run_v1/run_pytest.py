"""실제 pytest를 실행하되 import 경로와 binary를 먼저 기록한다."""
import hashlib,json
from pathlib import Path
import pytest
from arena_autonomy import numeric_kernels,_numeric_kernels

ROOT=Path(__file__).resolve().parent
PACKAGE=Path('/home/q/it-arena-r470-a/stage/src/arena_autonomy/arena_autonomy')
assert Path(numeric_kernels.__file__).resolve()==PACKAGE/'numeric_kernels.py'
native=Path(_numeric_kernels.__file__).resolve()
assert native.parent==PACKAGE and hasattr(_numeric_kernels,'current_mask_occupied')
assert hashlib.sha256(native.read_bytes()).hexdigest()=='5654cd2f52af838b52384a084f159bb1319925db6690945b4403cb7ae33f5bff'
(ROOT/'imports.json').write_text(json.dumps(dict(pytest_version=pytest.__version__,
    numeric_wrapper=numeric_kernels.__file__,native_binary=str(native)),indent=2))
raise SystemExit(pytest.main(['-q','-p','no:cacheprovider','-c','/dev/null',
    '--junitxml='+str(ROOT/'pytest.xml'),str(ROOT/'test_current_mask_native.py')]))
