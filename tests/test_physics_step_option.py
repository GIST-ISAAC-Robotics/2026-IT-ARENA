"""평가 전용 물리 step 옵션의 기본 불변·변경 범위·거부 조건과 분석 분류."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
import zipfile

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts'))
spec = importlib.util.spec_from_file_location('physics_step_launch', ROOT/'src/arena_bringup/launch/simulation.launch.py')
launch = importlib.util.module_from_spec(spec)
spec.loader.exec_module(launch)

import analyze_physics_step as analysis  # noqa: E402
import probe_braking_contact as probe  # noqa: E402
import validate_g1_traffic as g1  # noqa: E402

OFFICIAL = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
PREVIOUS_RUN = ROOT/'artifacts/validation/2026-10-08/zero_speed_v1/flat_passive_damping'
WORLD = ('<sdf version="1.8"><world name="w"><physics name="p" type="ode"><max_step_size>0.001</max_step_size>'
         '<real_time_factor>1.0</real_time_factor></physics><model name="m"><static>true</static></model></world></sdf>')


@pytest.fixture(scope='module')
def previous_launch(tmp_path_factory):
    """직전 1 ms 실행 당시 launch 사본. 기본 출력의 byte 동일성을 비교한다."""
    target = tmp_path_factory.mktemp('previous')/'simulation.launch.py'
    with zipfile.ZipFile(PREVIOUS_RUN/'source_snapshot.zip') as bundle:
        data = bundle.read('src/arena_bringup/launch/simulation.launch.py')
    assert hashlib.sha256(data).hexdigest() == 'b6c47b84bc758868fc171db2e3ddce4a80bcfda2108eb9e8a989e7d895e5334e'
    target.write_bytes(data)
    previous = importlib.util.spec_from_file_location('previous_launch', target)
    module = importlib.util.module_from_spec(previous)
    previous.loader.exec_module(module)
    return module


@pytest.mark.parametrize('options', [
    ('configured', True, True, False, None, True),
    ('configured', True, False, False, None, True),
    ('bullet', False, True, False, None, False),
])
def test_default_runtime_world_bytes_match_previous_launch(tmp_path, previous_launch, options):
    before = OFFICIAL.read_bytes()
    old, new = tmp_path/'old.sdf', tmp_path/'new.sdf'
    previous_launch._runtime_world_overrides(OFFICIAL, old, *options)
    launch._runtime_world_overrides(OFFICIAL, new, *options)
    launch._runtime_world_overrides(OFFICIAL, tmp_path/'explicit.sdf', *options, '')
    assert old.read_bytes() == new.read_bytes() == (tmp_path/'explicit.sdf').read_bytes()
    assert OFFICIAL.read_bytes() == before


@pytest.mark.parametrize('step', ['0.0005', '0.00025'])
def test_only_max_step_size_changes(tmp_path, step):
    before = OFFICIAL.read_bytes()
    base, changed = tmp_path/'base.sdf', tmp_path/'changed.sdf'
    launch._runtime_world_overrides(OFFICIAL, base, 'configured', True, False, False, None, True)
    launch._runtime_world_overrides(OFFICIAL, changed, 'configured', True, False, False, None, True, step)
    assert OFFICIAL.read_bytes() == before
    a, b = ET.parse(base).getroot(), ET.parse(changed).getroot()
    assert a.findtext('world/physics/max_step_size') == '0.001'
    assert b.findtext('world/physics/max_step_size') == step
    assert a.findtext('world/physics/real_time_factor') == b.findtext('world/physics/real_time_factor') == '1.0'
    b.find('world/physics/max_step_size').text = '0.001'
    assert ET.tostring(a) == ET.tostring(b)


@pytest.mark.parametrize('step', ['0.001', '0.0001', '0.002', 'abc', ' 0.0005'])
def test_unregistered_steps_rejected(step):
    with pytest.raises(ValueError):
        launch._check_evaluation_physics_step(step, '/tmp/evidence')
    with pytest.raises(ValueError):
        launch._evaluation_physics_step(ET.fromstring(WORLD).find('world'), step)


def test_evidence_path_required_and_default_noop():
    launch._check_evaluation_physics_step('', '')
    with pytest.raises(ValueError):
        launch._check_evaluation_physics_step('0.0005', '')
    world = ET.fromstring(WORLD).find('world')
    launch._evaluation_physics_step(world, '')
    assert ET.tostring(world) == ET.tostring(ET.fromstring(WORLD).find('world'))


@pytest.mark.parametrize('xml', [
    WORLD.replace('0.001</max', '0.002</max'),
    WORLD.replace('<max_step_size>0.001</max_step_size>', ''),
    WORLD.replace('</physics>', '<max_step_size>0.001</max_step_size></physics>'),
    WORLD.replace('</physics>', '</physics><physics name="q"><max_step_size>0.001</max_step_size></physics>'),
    WORLD.replace('<physics name="p" type="ode"><max_step_size>0.001</max_step_size><real_time_factor>1.0</real_time_factor></physics>', ''),
])
def test_world_without_single_baseline_step_rejected(xml):
    with pytest.raises(ValueError):
        launch._evaluation_physics_step(ET.fromstring(xml).find('world'), '0.0005')


def test_double_override_rejected():
    world = ET.fromstring(WORLD).find('world')
    launch._evaluation_physics_step(world, '0.00025')
    assert world.findtext('physics/max_step_size') == '0.00025'
    with pytest.raises(ValueError):
        launch._evaluation_physics_step(world, '0.0005')


def test_launch_argument_default_empty():
    description = launch.generate_launch_description()
    arguments = {a.name: a for a in description.entities if type(a).__name__ == 'DeclareLaunchArgument'}
    argument = arguments['evaluation_physics_step']
    assert argument.default_value[0].text == ''
    assert set(argument.choices) == {'', '0.0005', '0.00025'}
    assert arguments['evaluation_zero_speed_mode'].default_value[0].text == 'legacy'


def test_probe_default_arguments_match_previous_form():
    slot = {'x': 1.0, 'y': -2.0, 'yaw_rad': .5}
    assert probe.evaluation_extra() == []
    assert probe.evaluation_extra(None, '/tmp/w.sdf', slot) == [
        'world_override:=/tmp/w.sdf', 'test_spawn_x:=1.0', 'test_spawn_y:=-2.0', 'test_spawn_yaw:=0.5']
    assert probe.evaluation_extra('0.00025', None, slot) == ['evaluation_physics_step:=0.00025']
    assert probe.evaluation_extra('0.0005', '/tmp/w.sdf', slot)[0] == 'evaluation_physics_step:=0.0005'


def test_g1_default_launch_arguments_unchanged():
    assert g1.drivetrain_evaluation_arguments() == []
    assert g1.drivetrain_evaluation_arguments('legacy', None) == []
    assert g1.drivetrain_evaluation_arguments('passive_damping') == ['evaluation_zero_speed_mode:=passive_damping']
    assert g1.drivetrain_evaluation_arguments('legacy', '0.0005') == ['evaluation_physics_step:=0.0005']
    for bad in (('typo', None), ('legacy', '0.001'), ('legacy', '0.0001')):
        with pytest.raises(ValueError):
            g1.drivetrain_evaluation_arguments(*bad)


def test_contract_records_evaluation_script_changes():
    contract = json.loads((ROOT/'config/tests/physics_step_r0_sha256.json').read_text())
    scripts = contract['approved_evaluation_scripts']
    assert set(scripts) == {'scripts/probe_braking_contact.py', 'scripts/validate_g1_traffic.py'}
    # 직전 1 ms 실행이 기록한 실행 소스 해시가 변경 전 값이다.
    previous = json.loads((PREVIOUS_RUN/'report.json').read_text())['source_sha256']
    # 10/8 격자 발행 후속 계약이 probe의 다음 변경을 등록한다. 이 계약은 소급 수정하지 않는다.
    chain, parent = [], 'config/tests/physics_step_r0_sha256.json'
    for name in ('scheduled_command_r0_sha256.json', 'scheduled_command_r1_sha256.json', 'scheduled_command_r2_sha256.json',
                 'scheduled_command_r3_sha256.json', 'scheduled_command_r4_sha256.json', 'scheduled_command_r5_sha256.json',
                 'scheduled_command_r6_sha256.json', 'scheduled_command_r7_sha256.json',
                 'follow_trace_r0_sha256.json', 'fragmented_boundary_r0_sha256.json', 'dense_stop_evidence_r0_sha256.json',
                 'lidar_stress_eval_r0_sha256.json', 'per_ray_occupancy_r0_sha256.json', 'corner_anchor_r0_sha256.json', 'corner_distance_r0_sha256.json', 'corridor_fallback_r0_sha256.json', 'recent_observation_r0_sha256.json'):
        successor = json.loads((ROOT/'config/tests'/name).read_text())
        assert successor['parent_contract'] == parent
        if name in ('follow_trace_r0_sha256.json', 'fragmented_boundary_r0_sha256.json', 'dense_stop_evidence_r0_sha256.json',
                    'lidar_stress_eval_r0_sha256.json', 'per_ray_occupancy_r0_sha256.json', 'corner_anchor_r0_sha256.json', 'corner_distance_r0_sha256.json', 'corridor_fallback_r0_sha256.json', 'recent_observation_r0_sha256.json'):
            assert hashlib.sha256((ROOT/parent).read_bytes()).hexdigest() == successor['parent_contract_sha256']
        chain.append(successor)
        parent = 'config/tests/'+name
    for path, change in scripts.items():
        assert change['before'] == previous[path]
        latest = change
        for successor in chain:
            later = successor['approved_evaluation_scripts'].get(path)
            if later:
                assert later['before'] == latest['after']
                latest = later
        assert hashlib.sha256((ROOT/path).read_bytes()).hexdigest() == latest['after']


def test_source_difference_requires_registered_pairs():
    contract = {'approved_changes': {'a': {'before': '1', 'after': '2'}},
                'approved_evaluation_scripts': {'b': {'before': '3', 'after': '4'}}}
    assert analysis.source_difference({'a': '1', 'b': '3', 'c': '5'}, {'a': '2', 'b': '4', 'c': '5'}, contract)['approved']
    assert analysis.source_difference({'a': '1'}, {'a': '1'}, contract)['approved']
    assert not analysis.source_difference({'a': '1', 'c': '5'}, {'a': '2', 'c': '6'}, contract)['approved']
    assert not analysis.source_difference({'a': '0'}, {'a': '2'}, contract)['approved']
    assert not analysis.source_difference({'a': '1'}, {'a': '1', 'd': '7'}, contract)['approved']


def group(values):
    return {step: {'metrics': {name: value for name in analysis.PRIMARY_METRICS}} for step, value in values.items()}


@pytest.mark.parametrize('values,label', [
    ({.001: .40, .0005: .41, .00025: .42}, 'consistent_including_1ms'),
    ({.001: .60, .0005: .41, .00025: .42}, 'fine_consistent_1ms_distorted'),
    ({.001: .40, .0005: .50, .00025: .30}, 'not_converged'),
    ({.001: .40, .00025: .43}, 'consistent_including_1ms'),
    ({.001: .40, .00025: .60}, 'differs_at_1ms'),
])
def test_preregistered_classification(values, label):
    assert analysis.classify(group(values))['classification'] == label


def test_missing_metric_is_not_consistent():
    data = group({.001: .4, .0005: .4, .00025: .4})
    data[.0005]['metrics']['request_zero_to_stable_s'] = None
    assert analysis.classify(data)['classification'] == 'not_converged'


def test_tolerance_uses_larger_absolute_or_relative():
    assert analysis.tolerance('velocity', .2) == pytest.approx(.05)
    assert analysis.tolerance('velocity', 1.) == pytest.approx(.10)
    assert analysis.tolerance('distance', .1) == pytest.approx(.02)
    assert analysis.tolerance('time', .1) == pytest.approx(.05)


def test_step_match_uses_declared_requested_and_observed():
    run = {'declared_step_s': .0005, 'requested_override_s': '0.0005',
           'observed_step': {'median_s': .0005, 'min_s': .0005, 'max_s': .000502}}
    assert analysis.step_matches(run)
    assert not analysis.step_matches({**run, 'observed_step': {'median_s': .001, 'min_s': .001, 'max_s': .001}})
    assert not analysis.step_matches({**run, 'requested_override_s': None})
    assert analysis.step_matches({'declared_step_s': .001, 'requested_override_s': None,
                                  'observed_step': {'median_s': .001, 'min_s': .001, 'max_s': .001}})
    assert not analysis.step_matches({**run, 'observed_step': None})


@pytest.mark.parametrize('steps', [(.001,), (.0005, .00025), (.001, .0001), (.002, .001)])
def test_classification_rejects_missing_baseline_or_unregistered_steps(steps):
    with pytest.raises(ValueError):
        analysis.classify(group({s: .4 for s in steps}))


def test_duplicate_surface_mode_step_rejected(tmp_path, monkeypatch):
    contract = tmp_path/'contract.json'
    contract.write_text(json.dumps({'approved_changes': {}, 'approved_evaluation_scripts': {}}))
    fake = {'a': .001, 'b': .00025, 'c': .00025}
    monkeypatch.setattr(analysis, 'run_metrics', lambda p: ({'path': p, 'surface': 'official', 'mode': 'passive_damping',
                                                              'declared_step_s': fake[p]}, {}, [], '', ''))
    with pytest.raises(ValueError, match='duplicate'):
        analysis.analyze(['a', 'b', 'c'], contract)


def test_mismatched_samples_counts_grid_differences():
    # 10 ms 간격 유지값. 전환 시각만 20 ms 다르면 그 사이의 격자 한 점만 불일치다.
    stream = lambda switch: [{'t': i*.01+.005, 'v': float(i*.01+.005 >= switch)} for i in range(100)]
    result = analysis.mismatched_samples(stream(.49), stream(.51), 'v')
    assert result['mismatched'] == 1 and result['times_s'] == [.505]
    assert analysis.mismatched_samples(stream(.49), stream(.49), 'v')['mismatched'] == 0
    assert analysis.mismatched_samples([], stream(.49), 'v')['mismatched'] is None
