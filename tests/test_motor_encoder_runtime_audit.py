"""실행 당시 설정과 대조하며 현행 감속비로 과거 결과를 다시 해석하지 않는다."""
import hashlib
import json
import zipfile

import pytest
import yaml

from audit_motor_encoder_runtime import audit


@pytest.mark.parametrize('ratio', [15., 2.5 * 61 / 17])
def test_runtime_matches_its_own_snapshot_and_rejects_wrong_model(tmp_path, ratio):
    path = 'src/arena_description/config/vehicle.yaml'
    config = {'vehicle': {'drivetrain': {'motor': {'gear_ratio': ratio}, 'wheel_radius_m': .0325},
        'sensors': {'drive_feedback': {'mode': 'drive_motor_shaft',
            'drive_motor_encoder': {'counts_per_motor_revolution': 28}}}}}
    raw = yaml.safe_dump(config).encode()
    with zipfile.ZipFile(tmp_path/'source_snapshot.zip', 'w') as bundle:
        bundle.writestr(path, raw)
    report = {'source_sha256': {path: hashlib.sha256(raw).hexdigest()},
              'autonomy_subscriptions': ['/drive_motor/encoder'],
              'safety_subscriptions': ['/drive_motor/encoder']}
    (tmp_path/'report.json').write_text(json.dumps(report))
    (tmp_path/'actuation_trace.json').write_text(json.dumps([
        {'feedback_mode': 'drive_motor_shaft', 'motor_counts': 123}]))
    def model(value):
        (tmp_path/'runtime_vehicle.sdf').write_text(
            '<sdf><model><plugin name="arena::SingleMotorDrive">'
            f'<gear_ratio>{value}</gear_ratio><wheel_radius>0.0325</wheel_radius>'
            '</plugin></model></sdf>')
    model(ratio)
    assert audit(tmp_path)['passed']
    assert audit(tmp_path)['gear_ratio'] == ratio
    # 단독 평가가 완주 전 종료되면 구독 확인도 수행되지 않는다. 구동비와
    # 모터 카운트 근거만 확인하고 누락된 구독 증거를 통과로 만들지 않는다.
    solo={'input_sha256': report['source_sha256']}
    (tmp_path/'report.json').write_text(json.dumps(solo))
    limited=audit(tmp_path)
    assert limited['checks']['snapshot_config_hash_matches'] and limited['checks']['motor_counts_present']
    assert not limited['passed'] and not limited['subscription_evidence_present']
    (tmp_path/'report.json').write_text(json.dumps(report))
    model(ratio + 1.)
    assert not audit(tmp_path)['passed']
