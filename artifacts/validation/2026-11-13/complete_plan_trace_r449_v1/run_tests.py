"""pytest와 실제 spawn의 종료 및 격리 import를 한 프로세스에서 회수한다."""
from pathlib import Path
import json,multiprocessing as mp,sys
import pytest

def main():
    root=Path(__file__).resolve().parent
    outcomes=[]
    class Plugin:
        def pytest_runtest_logreport(self,report):
            if report.when=='call' or report.failed:outcomes.append(dict(nodeid=report.nodeid,phase=report.when,outcome=report.outcome))
    code=pytest.main(['-q','-p','no:cacheprovider','--basetemp',str(root.parent/'tmp'),*sys.argv[1:]],plugins=[Plugin()])
    remaining=[dict(pid=p.pid,name=p.name,alive=p.is_alive()) for p in mp.active_children()]
    imports={n:str(Path(m.__file__).resolve()) for n,m in sys.modules.items()
        if n.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(m,'__file__',None)}
    external={n:p for n,p in imports.items() if not Path(p).is_relative_to(root/'src')}
    report=dict(pytest_exitcode=int(code),outcomes=outcomes,owned_remaining=remaining,imports=imports,
        external_imports=external,python=sys.version,pytest=pytest.__version__)
    (root/'test_receipt.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(dict(owned_remaining=remaining,external_imports=external)))
    for child in mp.active_children():child.terminate();child.join(3.)
    return int(code) or int(bool(remaining or external))
if __name__=='__main__':raise SystemExit(main())
