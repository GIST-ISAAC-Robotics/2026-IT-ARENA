import json
from pathlib import Path
import sys
B=Path(__file__).resolve().parent
R=B.parents[3]
S=B/'stage'
sys.path[:0]=[str(S/'src/arena_autonomy'),str(S/'src/arena_vehicle_interface'),str(R/'tests'),str(R/'scripts'),
    str(B.parent/'complete_plan_trace_r449_v1/test_dependencies')]
import pytest
out=B/sys.argv[1];out.mkdir(exist_ok=False)
names=sys.argv[2:] or ['tests/test_observed_guard_approach.py','tests/test_guard_curve_trial.py',
    'tests/test_guard_masked_speed.py','tests/test_runtime_guard_crawl.py',
    'tests/test_current_path_recheck.py','tests/test_guard_observation_rebase.py',
    str((S/'tests/test_bounded_guard_approach.py').resolve())]
paths=[str(R/p) if not Path(p).is_absolute() else p for p in names]
code=pytest.main(['-q','-p','no:cacheprovider','--basetemp',str(out/'tmp'),
    '--junitxml',str(out/'junit.xml'),*paths])
modules={n:str(Path(v.__file__).resolve()) for n,v in sys.modules.items()
    if n.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(v,'__file__',None)}
external={n:p for n,p in modules.items() if not Path(p).is_relative_to(S)}
(out/'receipt.json').write_text(json.dumps(dict(exit_code=int(code),python=sys.executable,imports=modules,external=external),indent=2)+'\n',encoding='utf-8')
raise SystemExit(int(code) or bool(external))
