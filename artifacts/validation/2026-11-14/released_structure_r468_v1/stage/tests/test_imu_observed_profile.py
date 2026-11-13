import copy
import json
from pathlib import Path

import pytest
import yaml

from prepare_imu_observation_trial import profiles, audit_recording

ROOT = Path(__file__).resolve().parents[1]


def test_observed_profile_only_changes_imu_trial_inputs():
    base = yaml.safe_load((ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml').read_text())
    original = copy.deepcopy(base)
    summary = json.loads((ROOT/'artifacts/validation/2026-11-11/esp32_imu_input_v1/no_gc_summary.json').read_text())
    changed, fault = profiles(summary, base)
    assert base == original
    assert changed['camera'] == base['camera'] and changed['lidar'] == base['lidar']
    assert changed['imu']['rate_hz'] == pytest.approx(199.06850941348748)
    assert changed['imu']['nominal_rate_hz'] == 208.
    assert changed['imu']['gyro_full_scale_dps'] == 500 and changed['imu']['accel_full_scale_g'] == 8
    assert fault['imu'] == {'bias': pytest.approx(-.015615217079382257)}
    assert 'delay_s' not in fault['imu'] and 'stamp_offset_s' not in fault['imu']


def test_raw_conversion_and_generation_boundary(tmp_path):
    import math
    data = [dict(phase='measure', type=1, sensor_tag=1, boot=10, read_begin_us=1000+i*5000,
                 raw_xyz=[0, 1, -50], gyro_rad_s=[0., .0175*math.pi/180, -50*.0175*math.pi/180])
            for i in range(3)]
    path = tmp_path/'recording.jsonl'
    path.write_text(''.join(json.dumps(r)+'\n' for r in data))
    result = audit_recording(path)
    assert result['accepted'] == 3 and result['rejected'] == 0
    assert not result['timing_verified'] and not result['calibration_adopted']
    data[-1]['boot'] += 1
    path.write_text(''.join(json.dumps(r)+'\n' for r in data))
    with pytest.raises(ValueError, match='multiple MCU boots'):
        audit_recording(path)


def test_raw_si_mismatch_rejected(tmp_path):
    path = tmp_path/'recording.jsonl'
    path.write_text(json.dumps(dict(phase='measure', type=1, sensor_tag=1, boot=10,
        read_begin_us=1000, raw_xyz=[0, 0, 50], gyro_rad_s=[0., 0., 50.])))
    with pytest.raises(ValueError, match='SI conversion'):
        audit_recording(path)


def test_current_profile_reaches_generated_imu_without_adopting_measured_bias():
    import xml.etree.ElementTree as ET
    import xacro
    from test_low_speed_arc_reacquire import launch_module
    profile = yaml.safe_load((ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml').read_text())
    xml = xacro.process_file(str(ROOT/'src/arena_description/models/arena_car/model.sdf.xacro'),
                            mappings={'d435i_depth_enabled': 'false', 'tof_enabled': 'false'}).toxml()
    launch = launch_module()
    rendered = launch._b0183_model(xml, profile)
    trial = yaml.safe_load((ROOT/'artifacts/validation/2026-11-11/imu_observed_sensitivity_v1/sensor.yaml').read_text())
    assert rendered == launch._b0183_model(xml, trial)
    model = ET.fromstring(rendered)
    sensor = model.find(".//sensor[@name='lsm6dsox_imu']")
    assert float(sensor.findtext('update_rate')) == profile['imu']['rate_hz']
    for kind, sigma in (('angular_velocity', .0011), ('linear_acceleration', .0103)):
        for axis in 'xyz':
            noise = sensor.find(f'imu/{kind}/{axis}/noise')
            assert noise.attrib['type'] == 'gaussian'
            assert float(noise.findtext('mean')) == 0.
            assert float(noise.findtext('stddev')) == sigma
