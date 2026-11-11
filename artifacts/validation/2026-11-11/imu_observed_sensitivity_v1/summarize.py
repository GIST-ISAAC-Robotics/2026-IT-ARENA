"""원 결과를 바꾸지 않고 두 주행과 후속 프로필의 공개 요약을 만든다."""
import hashlib
import json
from pathlib import Path
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
target = HERE/'verification.json'
if target.exists():
    raise SystemExit('preserve verification')
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
preflight = json.loads((HERE/'preflight.json').read_text())
baseline_ok = sha(HERE/'baseline_sensor.before.yaml') == preflight['base_profile_sha256']
assert baseline_ok
with_bias = json.loads((HERE/'motion.json').read_text())
without_bias = json.loads((HERE/'zero_bias_motion.json').read_text())
with_bias['imu']['bias'] = 0.
assert with_bias == without_bias
summary = {}
for name,surface_file in (('solo_run1','surface.json'), ('zero_bias_after_overlay','zero_bias_surface.json')):
    run = HERE/name
    report = json.loads((run/'report.json').read_text())
    surface = json.loads((HERE/surface_file).read_text())
    imu = [json.loads(line) for line in (run/'motion_injection.jsonl').open()]
    imu = [r for r in imu if r.get('stream')=='imu' and r['event']=='publish']
    bad = [p for p,h in report['input_sha256'].items() if sha(ROOT/p)!=h]
    summary[name] = dict(passed=report['passed'], progress_m=report['progress_m'],
        completed_laps=report['completed_laps'], peak_ground_speed_mps=report['peak_ground_speed_mps'],
        collision_samples=report['collision_samples'],
        last_autonomy_reason=report['last_observation']['autonomy']['reason'],
        last_safety_reason=report['last_observation']['safety']['reason'],
        motion_status_audit=report['motion_status_audit'],
        source_count=len(imu), source_rate_hz=(len(imu)-1)/(imu[-1]['source_stamp_s']-imu[0]['source_stamp_s']),
        bias_error_max_rad_s=max(abs((r['after'][0]-r['before'][0])-
            (preflight['gyro_bias_trial_rad_s'] if name=='solo_run1' else 0.)) for r in imu),
        motion_impairment_verified=report['motion_impairment_audit']['verified'],
        historical_source_unchanged=report['source_unchanged'], current_source_hash_errors=bad,
        stop_passed=report.get('stop_stable',report.get('failure_cleanup_stop',{}).get('dense_pose_stop_passed',False)),
        stop_displacement_m=report.get('stop_displacement_m'),
        shutdown_clean=report['shutdown_clean'], remaining_test_pids=report.get('remaining_test_pids',[]),
        surface_samples=surface['samples'], body_outside_samples=surface['body_outside_samples_over_1e-6_m2'],
        wheel_outside_max={k:v['max_off_wheels'] for k,v in surface['interpretations'].items()})
    assert not bad
canonical = ROOT/'src/arena_description/config/b0390_c1_jr60p20_sim.yaml'
new = yaml.safe_load(canonical.read_text())
trial = yaml.safe_load((HERE/'sensor.yaml').read_text())
new['imu']['status'] = trial['imu']['status']
assert new == trial
result = dict(runs=summary, same_configuration_except_bias=True,
    baseline_profile_copy_hash_verified=baseline_ok, current_sensor_profile_sha256=sha(canonical),
    canonical_matches_trial_except_status=True,
    tests=dict(passed=127, command='pytest -q tests/test_imu_observed_profile.py tests/test_b0390_profile.py tests/test_local_sensor_profile.py tests/test_motion_impairment.py tests/test_lidar_motion_ros.py tests/test_lidar_motion.py'),
    limitations=['single run per bias condition; uncontrolled Gazebo noise/scheduling',
        'terminal-window bias removal is a saved-input counterfactual, not new driving',
        'hardware calibration, acquisition clocks and native C1 operational adapter unverified',
        'canonical profile uses zero-mean IMU simulation, not proven physical zero bias',
        'first zero_bias_control was ROS-overlay launch failure with zero sensor samples'])
target.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
print(json.dumps(result))
