"""교통·운영 평가 도구의 명시 track 선택과 옛 각본 차단을 확인한다.

기본 track=official의 입력·인자는 그대로이고, 새 코스는 같은 track으로 재등록한 설정에서만 열린다.
"""
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO/'scripts'))
import official_traffic_course as course  # noqa: E402
import race_ops_eval as ops  # noqa: E402
import summarize_motion_lap as summary  # noqa: E402
import validate_g1_traffic as harness  # noqa: E402
from validate_basic_autonomy import DEMO_TRACK, TRACK_WORLDS  # noqa: E402

NEW = 'official_v2026_11_06'
OLD_CONFIG = REPO/'config/tests/official_traffic_v1.json'
SENSOR = 'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'
ARGS = SimpleNamespace(mode='g1g2', handoff_reserve='paced', handoff_yield_progress=True,
                       handoff_rear_sweep=True, record_follow_decisions=True, seed=0)
LAUNCH = {'speed_profile': 'g1_open_1p5', 'grid_slot': 3}


def test_default_course_inputs_are_unchanged():
    assert course.entry_files() == course.ENTRY_FILES
    keys = list(course.inputs(ARGS, {'launch': LAUNCH}, {}))
    assert keys[:3] == ['src/arena_gazebo/worlds/it_arena_official/world.sdf',
                        'src/arena_gazebo/worlds/it_arena_official/scene.json',
                        'src/arena_gazebo/worlds/it_arena_official/centerline.csv']
    assert 'scripts/official_traffic_course.py' in keys


def test_new_track_requires_config_registered_for_that_track():
    world = TRACK_WORLDS[NEW]
    with pytest.raises(ValueError, match='재등록'):
        course.inputs(ARGS, {'launch': LAUNCH}, {}, world, NEW)
    with pytest.raises(ValueError, match='재등록'):
        course.inputs(ARGS, {'launch': LAUNCH, 'track': NEW}, {})
    keys = list(course.inputs(ARGS, {'launch': LAUNCH, 'track': NEW}, {}, world, NEW))
    assert keys[:3] == [f'src/arena_gazebo/worlds/it_arena_{NEW}/{name}'
                        for name in ('world.sdf', 'scene.json', 'centerline.csv')]


def test_old_traffic_config_is_not_registered_for_new_track():
    assert json.loads(OLD_CONFIG.read_text(encoding='utf-8')).get('track', DEMO_TRACK) == DEMO_TRACK


def run_harness(monkeypatch, tmp_path, *extra):
    argv = ['validate_g1_traffic.py', '--config', str(OLD_CONFIG), '--case', 'persistent_block', '--seed', '0',
            '--output', str(tmp_path/'unused'), *extra]
    monkeypatch.setattr(sys, 'argv', argv)
    harness.main()


@pytest.mark.parametrize('extra', [
    ('--mode', 'g1', '--track', NEW),
    ('--mode', 'g1', '--track', NEW, '--local-sensor-config', SENSOR),
    ('--mode', 'g1g2', '--handoff-reserve', 'paced', '--handoff-yield-progress', '--handoff-rear-sweep',
     '--record-follow-decisions', '--official-signal-course', '--track', NEW),
])
def test_traffic_harness_rejects_incomplete_new_track_selection(monkeypatch, tmp_path, extra):
    with pytest.raises(ValueError, match='--official-signal-course와 --local-sensor-config'):
        run_harness(monkeypatch, tmp_path, *extra)
    assert not (tmp_path/'unused').exists()


def test_traffic_harness_rejects_old_scenario_on_new_track(monkeypatch, tmp_path):
    with pytest.raises(ValueError, match='재등록'):
        run_harness(monkeypatch, tmp_path, '--mode', 'g1g2', '--handoff-reserve', 'paced', '--handoff-yield-progress',
                    '--handoff-rear-sweep', '--record-follow-decisions', '--official-signal-course',
                    '--track', NEW, '--local-sensor-config', SENSOR)
    assert not (tmp_path/'unused').exists()


def test_operation_scenarios_are_registered_only_for_default_track(monkeypatch, tmp_path):
    choices = ('representative', 'across_marker', 'parked_recovery', 'browser_stop')
    assert ops.REGISTERED_SCENARIOS == {DEMO_TRACK: choices}
    for scenario in choices:
        for extra in ((), ('--local-sensor-config', SENSOR)):
            monkeypatch.setattr(sys, 'argv', ['race_ops_eval.py', '--output', str(tmp_path/scenario),
                                              '--scenario', scenario, '--track', NEW, *extra])
            with pytest.raises(SystemExit):
                ops.main()
            assert not (tmp_path/scenario).exists()


def test_operation_course_reads_explicit_world():
    old, new = ops.Course(), ops.Course(TRACK_WORLDS[NEW])
    assert old.s[-1] != new.s[-1]
    assert ops.Course(TRACK_WORLDS[DEMO_TRACK]).s[-1] == old.s[-1]


def test_lap_summary_rejects_track_mismatch(monkeypatch, tmp_path):
    run = tmp_path/'artifacts/run'
    run.mkdir(parents=True)
    (run/'report.json').write_text(json.dumps({'passed': False, 'track': NEW}), encoding='utf-8')
    (run/'trace.json').write_text('[]', encoding='utf-8')
    monkeypatch.setattr(summary, 'REPO', tmp_path)
    monkeypatch.setattr(sys, 'argv', ['summarize_motion_lap.py', str(run)])
    with pytest.raises(ValueError, match='track='):
        summary.main()
    assert not (run/'lap_analysis.json').exists()
