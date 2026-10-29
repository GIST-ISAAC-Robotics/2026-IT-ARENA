import json
from pathlib import Path
import xml.etree.ElementTree as ET
import pytest
from sensor_stress import apply_stress,sensor,noise_nodes,verify_runtime_stress,SIGMA,BIAS,LIDAR_YAW

BASE=Path(__file__).resolve().parent
REFERENCE=BASE.parent/'official_traffic_v1/run2_course/runtime_vehicle.sdf'


def test_only_expected_sensor_fields_change():
    raw=REFERENCE.read_text(encoding='utf-8')
    original=ET.fromstring(raw);changed=ET.fromstring(apply_stress(raw))
    lidar=sensor(changed,'rplidar_c1'); pose=lidar.find('pose')
    assert list(map(float,pose.text.split()))==[0,0,0,0,0,LIDAR_YAW]
    lidar.remove(pose)
    for axis,node in noise_nodes(changed).items():
        assert float(node.findtext('stddev'))==SIGMA
        assert float(node.findtext('mean'))==(BIAS if axis=='z' else 0.)
        node.find('stddev').text=noise_nodes(original)[axis].findtext('stddev')
        node.find('mean').text=noise_nodes(original)[axis].findtext('mean')
    assert ET.tostring(original)==ET.tostring(changed)


def test_duplicate_or_missing_sensor_rejected():
    with pytest.raises(ValueError):apply_stress(apply_stress(REFERENCE.read_text(encoding='utf-8')))
    with pytest.raises(ValueError):apply_stress('<sdf/>')


def test_runtime_model_and_uncorrected_parameters_required(tmp_path):
    (tmp_path/'runtime_vehicle.sdf').write_text(apply_stress(REFERENCE.read_text(encoding='utf-8')),encoding='utf-8')
    for gyro,yaw,expected in ((0.,0.,True),(BIAS,0.,False),(0.,LIDAR_YAW,False)):
        (tmp_path/'follow_decisions.jsonl').write_text(json.dumps(dict(data=dict(motion_config=dict(gyro_bias_rad_s=gyro,lidar_yaw_rad=yaw))))+'\n',encoding='utf-8')
        assert verify_runtime_stress(tmp_path)['passed']==expected
    (tmp_path/'runtime_vehicle.sdf').write_bytes(REFERENCE.read_bytes())
    assert not verify_runtime_stress(tmp_path)['passed']
