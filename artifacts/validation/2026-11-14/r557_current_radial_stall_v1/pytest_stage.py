"""Run targeted policy tests with only the explicit three staged modules loaded."""
import importlib
from pathlib import Path
import pytest

A = Path(__file__).resolve().parent
for suffix in ('current_wall_reacquire', 'current_path_recheck._geometry',
               'race_runtime_policy.retained_recheck._geometry'):
    module = importlib.import_module('arena_autonomy.'+suffix)
    path = A/'stage/src/arena_autonomy/arena_autonomy'/Path(suffix.replace('.', '/')+'.py')
    module.__file__ = str(path)
    exec(compile(path.read_text(encoding='utf8'), str(path), 'exec'), module.__dict__)
raise SystemExit(pytest.main(['-q', 'tests/test_observed_crawl_recheck.py',
    '--junitxml='+str(A/'stage_tests.xml')]))
