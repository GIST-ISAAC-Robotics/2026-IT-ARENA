from pathlib import Path
import json,sys
B=Path(__file__).resolve().parent;stage=B/'stage'
sys.path[:0]=[str(stage/'src/arena_autonomy'),str(stage/'src/arena_vehicle_interface'),
    str(stage/'tests'),str(stage/'scripts'),str(B.parent/'operator_panel_r429_v1/pydeps')]
import pytest
out=B/sys.argv[1];out.mkdir(exist_ok=False)
outcomes=[]
class Capture:
    def pytest_runtest_logreport(self,report):
        if report.when=='call' or report.failed:outcomes.append(dict(test=report.nodeid,phase=report.when,outcome=report.outcome))
names=sys.argv[2:] or ['test_runtime_guard_crawl.py','test_runtime_guard_view_reuse.py',
    'test_guard_curve_trial.py','test_guard_masked_speed.py','test_pass_release_trial.py','test_guard_observation_policy.py']
code=pytest.main(['-q','-p','no:cacheprovider','--basetemp',str(out/'tmp'),*[str(stage/'tests'/n) for n in names]],plugins=[Capture()])
imports={n:str(Path(v.__file__).resolve()) for n,v in sys.modules.items()
    if n.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(v,'__file__',None)}
external={n:p for n,p in imports.items() if not Path(p).is_relative_to(stage)}
(out/'receipt.json').write_text(json.dumps(dict(exit_code=int(code),outcomes=outcomes,imports=imports,
    external_project_imports=external),indent=2)+'\n',encoding='utf8')
raise SystemExit(int(code) or bool(external))
