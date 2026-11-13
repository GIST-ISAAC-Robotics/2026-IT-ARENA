"""격리된 동결 소스에서 지정 단위검사만 실행. ROS init/spin/장치 없음."""
import argparse
import hashlib
import importlib
import json
from pathlib import Path
import sys

import pytest


def digest(path): return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    output = parser.parse_args().output.resolve()
    root = Path(__file__).resolve().parent
    assert output.parent == root and not output.exists()
    output.mkdir()
    manifest = json.loads((root/'bundle_manifest.json').read_bytes())
    files = manifest['files']
    def changes():
        return [name for name,sha in files.items() if not (root/name).is_file() or digest(root/name)!=sha]
    assert not changes(), 'source changed before test'
    selected = {}
    for name in manifest['staged_modules']:
        module = importlib.import_module('arena_autonomy.'+name)
        expected = root/'src/arena_autonomy/arena_autonomy'/(name+'.py')
        assert Path(module.__file__).resolve() == expected.resolve(), ('wrong staged module', name)
        selected[name] = dict(path=expected.relative_to(root).as_posix(), sha256=digest(expected))
    outcomes = []; collected = 0
    class Results:
        def pytest_collection_finish(self, session):
            nonlocal collected
            collected = len(session.items)
        def pytest_runtest_logreport(self, report):
            if report.when == 'call' or report.outcome in ('failed','skipped'):
                outcomes.append(dict(nodeid=report.nodeid, when=report.when, outcome=report.outcome,
                    duration_s=report.duration, detail=str(report.longrepr) if report.longrepr else None))
    code = int(pytest.main(['-q', '-p', 'no:cacheprovider', *manifest['test_nodeids']], plugins=[Results()]))
    imported = {}; external_arena = []
    for name,module in sorted(sys.modules.items()):
        if not name.startswith(('arena_autonomy','arena_vehicle_interface')) or not getattr(module,'__file__',None): continue
        path = Path(module.__file__).resolve()
        if not path.is_relative_to(root):
            external_arena.append(dict(module=name,path=str(path))); continue
        rel = path.relative_to(root).as_posix()
        assert rel in files, ('unmanifested project import', name, rel)
        imported[name] = dict(path=rel,sha256=digest(path))
    altered = changes()
    passed = sum(r['when']=='call' and r['outcome']=='passed' for r in outcomes)
    skipped = sum(r['outcome']=='skipped' for r in outcomes)
    result = dict(passed=code==0 and passed==collected and skipped==0 and not altered and not external_arena,
        pytest_exit_code=code, collected=collected, passed_calls=passed, skipped=skipped,
        failures=[r for r in outcomes if r['outcome']!='passed'], selected_modules=selected,
        imported_sources=imported, source_changes=altered, external_arena_imports=external_arena,
        source_files=len(files), physical_device_opened=False, ros_nodes_started=False,
        simulation_started=False, physical_timing_verified=False, physical_mounting_verified=False)
    (output/'report.json').write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('imported_sources','selected_modules','failures')},ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__': raise SystemExit(main())
