"""기존 주행의 기구·제어·평가를 유지한 복합 오차 시험인지 검사한다. 주행 증거가 아니다."""
import hashlib
import json
from pathlib import Path
import xml.etree.ElementTree as ET

from sensor_stress import apply_stress, BIAS, SIGMA, YAW, sensor

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
OLD = BASE.parent/'g2_center_gazebo_v1'


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    original = ET.parse(OLD/'run1/runtime_vehicle.sdf').getroot()
    changed = ET.fromstring(apply_stress(ET.tostring(original, encoding='unicode')))
    lidar = sensor(changed, 'rplidar_c1')
    pose = list(map(float, lidar.findtext('pose').split()))
    lidar.remove(lidar.find('pose'))
    imu = sensor(changed, 'lsm6dsox_imu')
    native_ok = True
    for axis in ('x', 'y', 'z'):
        noise = imu.find('imu/angular_velocity/'+axis+'/noise')
        native_ok &= float(noise.findtext('stddev')) == SIGMA
        native_ok &= float(noise.findtext('mean')) == (BIAS if axis == 'z' else 0.)
        noise.find('mean').text = '0'
        noise.find('stddev').text = '0.001'
    # XML formatting can differ, so normalize text-only numeric representation before comparing.
    for axis in ('x', 'y', 'z'):
        noise = sensor(original, 'lsm6dsox_imu').find('imu/angular_velocity/'+axis+'/noise')
        noise.find('mean').text = '0'
        noise.find('stddev').text = '0.001'
    model_only = ET.tostring(original) == ET.tostring(changed)
    manifest = json.loads((OLD/'run1_prepared.json').read_text())['source_sha256']
    relevant = {p:h for p,h in manifest.items() if p.startswith(('src/', 'scripts/', 'config/'))}
    altered = [p for p,h in relevant.items() if not (ROOT/p).is_file() or sha(ROOT/p) != h]
    children=[json.loads((ROOT/name).read_text()) for name in (
        'config/tests/g2_signed_hold_r1_sha256.json','config/tests/g2_handoff_corridor_r1_sha256.json')]
    approved={};dependencies=True
    for child in children:
        approved.update(child['approved_changes'])
        dependencies &= all(sha(ROOT/p)==h for p,h in child['new_dependencies'].items())
    registered=set(altered)==set(approved) and all(manifest[p]==row['before'] and sha(ROOT/p)==row['after']
        for p,row in approved.items())
    geometry = json.loads((OLD/'preflight.json').read_text())
    cfg = 'g2_center_config_v1.json'
    actual_config = json.loads((BASE/cfg).read_text())
    previous_config=json.loads((OLD/cfg).read_text())
    expected=actual_config['acceptance']['g2']['trace_header_expected']
    registered_handoff=(expected['schema']==5 and expected['controller_mode']=='g1_g2_handoff_trial'
        and expected['execution_config']['entry_policy']=='corridor_local_v1')
    allowed=('trace_header_expected','trace_header_basis','command_cap_mps','command_cap_scope','decision_replay')
    for key in allowed:
        actual_config['acceptance']['g2'].pop(key,None)
        previous_config['acceptance']['g2'].pop(key,None)
    installed=ROOT/'install/arena_autonomy/lib/python3.12/site-packages/arena_autonomy'
    names=('bypass_handoff.py','g1_g2_node.py','bypass_handoff_corridor_trial.py','bypass_signed_hold_trial.py')
    install_current=all((installed/name).is_file() and sha(installed/name)==sha(ROOT/'src/arena_autonomy/arena_autonomy'/name) for name in names)
    checks = dict(baseline_geometry_passed=geometry['passed'],
        same_config_except_explicit_policy=actual_config == previous_config and registered_handoff, installed_sources_current=install_current,
        same_evaluation=all(sha(BASE/p) == sha(OLD/p) for p in ('g2_eval_core.py', 'g2_eval_v2.py')),
        only_registered_controller_change=registered, registered_new_dependencies=dependencies, only_registered_model_errors=model_only,
        gyro_model=native_ok, lidar_pose=pose == [0., 0., 0., 0., 0., YAW])
    result = dict(passed=all(checks.values()), checks=checks, changed=altered,
        inherited_geometry_file=str(OLD/'preflight.json'), inherited_geometry_sha256=sha(OLD/'preflight.json'),
        inherited_straight_ahead_m=geometry['straight_ahead_m'],
        scope='same geometry/physics/evaluation, registered G1/G2 corridor successor and prior sensor stress; not a driving result')
    with (BASE/'preflight.json').open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
    print(json.dumps(result, ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
