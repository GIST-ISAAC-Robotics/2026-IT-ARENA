"""기존 IMU/단일 모터축 오류 중계의 G1 평가 연결. 제어 입력/보호 기준은 유지한다."""
import hashlib
import json
from pathlib import Path
import yaml

from arena_vehicle_interface.drive_feedback import MODE_MOTOR, MOTOR_TOPIC
from arena_vehicle_interface.motion_impairment import MotionFaultConfig
from motion_fault_audit import audit


def load_motion_profile(path, repo):
    if path is None:
        return None, None
    resolved=Path(path).resolve()
    resolved.relative_to(Path(repo).resolve())
    return resolved, MotionFaultConfig.from_dict(json.loads(resolved.read_text(encoding='utf-8'))).as_dict()


def motor_count_resolution(repo):
    vehicle = yaml.safe_load((Path(repo)/'src/arena_description/config/vehicle.yaml').read_text(encoding='utf-8'))['vehicle']
    return vehicle['sensors']['drive_feedback']['drive_motor_encoder']['counts_per_motor_revolution']


def require_motion_evidence(report, output, requested, runtime_sha256):
    """판정을 더 엄격하게만 결합한다. 주입이 없거나 잘못되면 시험 유효성도 거절한다."""
    output=Path(output)
    try:
        evidence=audit(output/'motion_injection.jsonl',requested,feedback_mode=MODE_MOTOR,
                       counts_per_motor_revolution=report['motion_test_profile']['counts_per_motor_revolution'])
        evidence['runtime_profile_unchanged']=(hashlib.sha256((output/'motion_test_profile.json').read_bytes()).hexdigest()==runtime_sha256)
        evidence['log_sha256']=hashlib.sha256((output/'motion_injection.jsonl').read_bytes()).hexdigest()
        evidence['verified']=evidence['verified'] and evidence['runtime_profile_unchanged']
    except (OSError, ValueError, KeyError, TypeError) as error:
        evidence={'verified':False,'error':f'{type(error).__name__}: {error}'}
    expected={'/imu/data',MOTOR_TOPIC}
    publishers=report.get('motion_relay_publishers',{})
    exclusive=set(publishers)==expected and all(v==['motion_fault_relay'] for v in publishers.values())
    graph=(expected.issubset(report.get('autonomy_subscriptions',[]))
           and expected.issubset(report.get('safety_subscriptions',[])))
    verified=evidence.get('verified') is True and exclusive and graph
    report['motion_stress_evidence']={**evidence,'exclusive_publishers':exclusive,'both_consumers':graph,'verified':verified}
    report['integrity']['motion_stress_verified']=verified
    report['harness_valid']=report['harness_valid'] and verified
    report['run_valid']=report['run_valid'] and verified
    report['passed']=report['passed'] and verified
    return report
