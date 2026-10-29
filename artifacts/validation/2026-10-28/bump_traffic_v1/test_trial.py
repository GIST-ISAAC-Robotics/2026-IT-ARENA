"""방지턱ON 전달·각본/판정 보존·원본과 실행 충돌 대조 검사."""
import importlib.util
import inspect
import json
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import pytest

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path[:0] = [str(ROOT/'scripts'), str(ROOT/'src/arena_autonomy'), str(ROOT/'src/arena_vehicle_interface')]
spec = importlib.util.spec_from_file_location('bump_traffic_trial_test', BASE/'run.py')
run = importlib.util.module_from_spec(spec)
spec.loader.exec_module(run)


def effective_launch(output, settings):
    from validate_basic_autonomy import start_demo_process
    values = inspect.signature(start_demo_process).bind_partial(None)
    values.apply_defaults()
    data = values.arguments
    data.update(speed_profile='g1_compare_1mps', lidar_rate_hz=10,
        lidar_acquisition='sequential', lidar_compensation='both', control_rate_hz=50,
        grid_slot=3, autonomy_mode='local_pursuit', actuation_mode='virtual_mcu',
        speed_bump=True, collision_detector='configured', g1_follow=True,
        r0_plus=False, lidar_test_profile='', motion_test_profile='',
        record_inputs=False, runtime_artifacts=str(output))
    keys = ('partial_observation_policy', 'g1_parallel_hold_trial', 'g1_pass_release_trial',
        'g1_guard_curve_trial', 'g1_traffic_recovery_trial', 'g1_traffic_path_recovery_trial',
        'g1_boundary_evidence_mode', 'g1_trace_path', 'g1_occupancy_timing_mode',
        'g1_corridor_fallback_enabled', 'g1_recent_observation_enabled', 'g1_short_boundary_prefix_enabled')
    data['extra_arguments'] = ['evaluation_actor_sdf:='+str(output/'evaluation_actor.sdf')]
    for key in keys:
        value = settings[key]
        data['extra_arguments'].append(key+':='+(str(value).lower() if isinstance(value, bool) else str(value)))
    return data


def test_launch_accepts_only_explicit_bump_on_and_existing_complete_contract():
    output = BASE/'not_executed'
    text, settings, arguments = run.launch_source(output)
    assert "'speed_bump': 'true'" in text
    assert "'speed_bump': 'false'" not in text
    assert settings['g1_traffic_recovery_trial'] and settings['g1_traffic_path_recovery_trial']
    assert settings['start_signal_mode'] == 'official_4red_local_candidate'
    valid = effective_launch(output, settings)
    assert run.validate_launch_call(valid, settings, arguments, output)
    for key, value in [('speed_bump', False), ('speed_profile','brisk'),
                       ('lidar_rate_hz', 15), ('actuation_mode','legacy'), ('g1_follow', False)]:
        with pytest.raises(ValueError):
            run.validate_launch_call({**valid,key:value}, settings, arguments, output)
    with pytest.raises(ValueError):
        run.validate_launch_call(valid, settings, {**arguments,'speed_bump':'false'}, output)


def test_config_keeps_schedule_sensor_drivetrain_and_existing_verdicts():
    current = json.loads((BASE/'traffic_config.json').read_text(encoding='utf-8'))
    old = json.loads((ROOT/'config/tests/g1_traffic_full_course_v1.json').read_text(encoding='utf-8'))
    for key in ('cases', 'acceptance', 'actor', 'timeouts', 'vehicle_and_sensor_unchanged'):
        assert current[key] == old[key]
    changed = {key for key in current['launch'] if current['launch'][key] != old['launch'][key]}
    assert changed == {'speed_bump','temporary_traffic_light_trigger'}


def test_capture_only_adds_pose_observation_and_file_no_extra_control():
    base = run.original_effective()
    changed = run.effective_sources()
    assert base[0] == changed[0]
    expected = base[1][1].replace('    ego, actor, contacts, commands = [], [], [], []',
        '    ego, actor, contacts, commands = [], [], [], []\n    bump_poses = []')
    anchor = "                    ego.append((when, float(p.get('x', 0.)), float(p.get('y', 0.)), yaw_of(pose['orientation'])))"
    expected = expected.replace(anchor,anchor+"\n                    bump_poses.append(dict(time=when, position=dict(p), orientation=dict(pose['orientation'])))")
    anchor = "        (output/'actor_commands.json').write_text(json.dumps(commands, indent=1), encoding='utf-8')"
    expected = expected.replace(anchor,anchor+"\n        (output/'bump_pose_trace.json').write_text(json.dumps(bump_poses), encoding='utf-8')")
    assert expected == changed[1][1]
    compile(changed[1][1], 'trial_capture', 'exec')


@pytest.mark.parametrize('change', ['remove','shape','position'])
def test_runtime_bump_contract_detects_missing_or_changed_collision(tmp_path, change):
    original = run.parent.FIXTURE/'world.sdf'
    reference = run.bump_collisions(original)
    assert reference and sum(len(row['collisions']) for row in reference) >= 20
    tree = ET.parse(original)
    for model in tree.getroot().iter('model'):
        link = next((link for link in model.findall('link')
                     if link.get('name','').startswith('safety_bump_')), None)
        if link is None:
            continue
        if change == 'remove':
            model.remove(link)
        elif change == 'shape':
            link.find('collision/geometry/box/size').text = '1 1 1'
        else:
            link.find('pose').text = '0 0 0 0 0 0'
        break
    out = tmp_path/'changed.sdf'
    tree.write(out, encoding='unicode')
    assert run.bump_collisions(out) != reference
