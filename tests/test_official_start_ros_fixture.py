"""B3 실제 ROS 시험기의 픽셀/사례 구성 회귀. ROS 실행 결과와는 별개다."""
from pathlib import Path
import sys
from types import SimpleNamespace
import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
import validate_pipeline_faults_ros as validation
from arena_autonomy.race_start import OfficialStartSignal


def test_runtime_manifest_covers_static_local_imports_transitively():
    # 모듈을 추가할 때 실행 경로에 import되지만 보존 목록에는 없는 파일을 방지한다.
    # 동적 import 문자열/외부 라이브러리 전체의 재현성 검사는 아니다.
    import ast
    manifest = {f'{package}.{name}' for package, names in validation.MODULES.items() for name in names}
    for module in manifest:
        package, name = module.split('.')
        tree = ast.parse((ROOT/'src'/package/package/f'{name}.py').read_text(encoding='utf-8'))
        imports = []
        for node in ast.walk(tree):
            if isinstance(node, ast.ImportFrom) and node.module:
                imports.append(node.module)
            elif isinstance(node, ast.Import):
                imports.extend(alias.name for alias in node.names)
        required = {name for name in imports if name.startswith(('arena_autonomy.', 'arena_vehicle_interface.'))}
        assert required <= manifest, (module, sorted(required-manifest))


def test_official_fixture_uses_pixel_sequence_not_side_channel():
    detector = OfficialStartSignal()
    stamp = 0.
    for color in ('green', 'off', 'red1', 'red2', 'red3', 'red', 'green'):
        rgb = np.frombuffer(validation.signal_rgb(color, 'official_4red_candidate'), np.uint8).reshape(480, 848, 3)
        for _ in range(3):
            stamp += .1
            detector.update(rgb, stamp)
        if stamp < 2.:
            assert not detector.started
    assert detector.started


def test_fixture_unknown_mode_fails():
    with pytest.raises(ValueError, match='mode'):
        validation.signal_rgb('red', 'unsupported')


def test_official_ros_sequence_retains_independent_stop_and_rearm():
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.signal_mode = 'official_4red_candidate'
    events = []
    validator.call = lambda operation: events.append(('service', operation))
    validator.children = SimpleNamespace(send=lambda *args, **kw: events.append(('exit', 'controller')))
    validator.stop_case = lambda name, action, expected, **kw: (events.append(('stop', name, kw)), action())
    validator.moving = lambda name: events.append(('move', name))
    validator.red_green = lambda: events.append(('pixels', 'red_green'))
    validator.restart_controller = lambda: events.append(('restart', 'no_preserved_start'))
    validator.final_stop = lambda: events.append(('final', 'stop_graph_cleanup'))
    validator.run_start_contract()
    names = [e[1] for e in events]
    assert names.index('stop') < names.index('clear_stop') < names.index('arm')
    assert names[-1] == 'stop_graph_cleanup'
    assert names.index('official_controller_exit_stops_before_restart') < names.index('no_preserved_start')
    assert 'official_green_extinguished_keeps_race_start' in names
    assert 'official_fresh_off_image_restores_existing_permit' in names
    assert all(e[2]['no_resume'] for e in events if e[0] == 'stop' and
               (e[1].endswith('_no_start') or e[1] == 'official_clear_alone_is_disarmed'))


def test_official_scenario_cannot_silently_use_old_detector():
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.signal_mode = 'temporary_3light'
    with pytest.raises(ValueError, match='official candidate'):
        validator.run_start_contract()


@pytest.mark.parametrize('failed', [False, True])
def test_failure_cleanup_stop_does_not_relabel_the_failed_run(failed):
    validator = validation.PipelineValidator.__new__(validation.PipelineValidator)
    validator.report = {'passed': False, 'error': 'original failure'}
    validator.monitor = SimpleNamespace(zero_watch=object(), sequence=7)
    def call(name):
        assert name == 'stop'
        if failed:
            raise RuntimeError('stop unreachable')
    validator.call = call
    validator.stopped = lambda seq: seq == 7
    validator.wait = lambda predicate, *a, **kw: predicate()
    validator.stop_after_failure()
    assert validator.report['passed'] is False
    assert validator.report['error'] == 'original failure'
    assert validator.report['failure_stop']['acknowledged'] is not failed
    assert validator.report['failure_stop']['fresh_zero_and_model_stopped'] is not failed
