"""명시 선택 경계/기존 backend 회귀만. ROS init/spin/장치 없음."""
import json
from pathlib import Path
import pytest


def main():
    root=Path(__file__).resolve().parent
    results=[];count=0
    class Result:
        def pytest_collection_finish(self,session):
            nonlocal count
            count=len(session.items)
        def pytest_runtest_logreport(self,report):
            if report.when=='call' or report.failed or report.skipped:
                results.append(dict(nodeid=report.nodeid,when=report.when,outcome=report.outcome,
                    detail=str(report.longrepr) if report.longrepr else None))
    code=int(pytest.main(['-q','-p','no:cacheprovider','tests/test_current_corner_runtime.py',
                         'tests/test_runtime_node_backend.py'],plugins=[Result()]))
    passed=sum(r['when']=='call' and r['outcome']=='passed' for r in results)
    report=dict(passed=code==0 and passed==count,exit_code=code,collected=count,passed_calls=passed,
        skipped=sum(r['outcome']=='skipped' for r in results),failures=[r for r in results if r['outcome']!='passed'],
        ros_nodes_started=False,physical_devices_opened=False)
    (root/'tests_report.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
