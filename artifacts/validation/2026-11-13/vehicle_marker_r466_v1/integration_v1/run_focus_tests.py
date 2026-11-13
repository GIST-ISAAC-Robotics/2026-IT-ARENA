"""Common 경로로만 marker/vision 집중 검사; WSL/ROS/실물 실행 없음."""
import importlib
import json
import os
from pathlib import Path
import sys

OUT = Path(__file__).resolve().parent
REPO = OUT.parent.parents[3]
sys.dont_write_bytecode = True
os.environ['PYTHONDONTWRITEBYTECODE'] = '1'
sys.path[:0] = [str(REPO / 'artifacts/validation/2026-11-13/operator_panel_r429_v1/pydeps'),
               str(REPO / 'src/arena_autonomy'), str(REPO / 'src/arena_vehicle_interface'),
               str(REPO / 'tests')]

if __name__ == '__main__':
    modules = ['vehicle_marker_geometry', 'vehicle_marker_occupancy', 'vehicle_marker_channel',
               'vehicle_marker_calibration', 'vehicle_markers', 'vision_process', 'local_follow',
               'm3_execution_trial', 'race_runtime_policy.evaluator', 'runtime_node_backend']
    paths = {name: str(Path(importlib.import_module('arena_autonomy.' + name).__file__).resolve())
             for name in modules}
    assert all(Path(path).is_relative_to(REPO / 'src/arena_autonomy') for path in paths.values())
    (OUT / 'import_paths.json').write_text(json.dumps(paths, indent=2) + '\n', encoding='utf-8')
    import pytest
    tests = ['test_vehicle_marker_regions.py', 'test_vehicle_marker_connection.py',
             'test_vehicle_marker_duplicates.py', 'test_vehicle_marker_observer.py', 'test_vision_process.py']
    code = pytest.main(['-q', '-p', 'no:cacheprovider', '--basetemp=' + str(OUT / 'testtmp'),
                       '--junitxml=' + str(OUT / 'windows_focus.xml')]
                      + [str(REPO / 'tests' / name) for name in tests])
    (OUT / 'test_exit.json').write_text(json.dumps({'exit_code': int(code), 'tests': tests,
        'source': 'common repo only; no stage import', 'scope': 'Windows focused software checks'}) + '\n', encoding='utf-8')
    raise SystemExit(code)
