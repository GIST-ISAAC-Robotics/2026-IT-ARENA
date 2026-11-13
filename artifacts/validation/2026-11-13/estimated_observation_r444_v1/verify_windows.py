"""공통 입력을 읽되 변경 모듈만 stage에서 import하는 Windows 순수 검사 실행기."""
from pathlib import Path
import importlib, json, sys, unittest

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
STAGE=BASE/'stage'
sys.dont_write_bytecode=True
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
package=importlib.import_module('arena_autonomy')
package.__path__.insert(0,str(STAGE/'src/arena_autonomy/arena_autonomy'))
policy=importlib.import_module('arena_autonomy.race_runtime_policy')
policy.__path__.insert(0,str(STAGE/'src/arena_autonomy/arena_autonomy/race_runtime_policy'))
suite=unittest.defaultTestLoader.discover(str(STAGE/'tests'),pattern='test_estimated_observation_scope.py')
result=unittest.TextTestRunner(verbosity=2).run(suite)
imports={name:module.__file__ for name,module in sys.modules.items()
    if name.startswith('arena_autonomy.') and getattr(module,'__file__',None)
    and str(STAGE) in module.__file__}
print(json.dumps({'tests':result.testsRun,'failures':len(result.failures),'errors':len(result.errors),
    'skips':len(result.skipped),'stage_imports':imports},ensure_ascii=False,indent=2))
raise SystemExit(0 if result.wasSuccessful() else 1)
