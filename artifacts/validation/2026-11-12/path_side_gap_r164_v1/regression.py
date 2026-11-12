"""시험 후보를 실제 설치한 프로세스에서 기존 거절/경계 회귀를 수행한다."""
import importlib.util
from pathlib import Path
import pytest
import sys

A = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('installed_r164', A/'candidate.py')
c = importlib.util.module_from_spec(spec)
spec.loader.exec_module(c)
c.install()
try:
    code = pytest.main(['-q', *(sys.argv[1:] or [
        'tests/test_pass_release_trial.py', 'tests/test_g1_guard_partial_coupling.py',
        str(A/'test_candidate.py')])])
finally:
    c.uninstall()
raise SystemExit(code)
