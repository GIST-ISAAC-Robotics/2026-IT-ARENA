"""별도 배정된 Linux 또는 Windows 회귀 실행용. r464 준비 단계에서는 미실행."""
from pathlib import Path
import os
import sys

HERE=Path(__file__).resolve().parent
sys.dont_write_bytecode=True
package_root=Path(os.environ.get('R464_PACKAGE_ROOT',str(HERE/'original')))
sys.path[:0]=[str(package_root/'src/arena_autonomy'),
             str(package_root/'src/arena_vehicle_interface')]
import arena_autonomy
arena_autonomy.__path__.insert(0,str(HERE/'stage/src/arena_autonomy/arena_autonomy'))
import pytest
raise SystemExit(pytest.main(['-q','-p','no:cacheprovider',str(HERE/'stage/tests/test_boundary_transient_r464.py'),*sys.argv[1:]]))
