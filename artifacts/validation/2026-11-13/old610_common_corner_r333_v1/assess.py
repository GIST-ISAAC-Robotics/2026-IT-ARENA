"""원 보고를 덮지 않고 주행/정지/입력 수집 판정을 분리한다."""
import hashlib
import json
from pathlib import Path

A = Path(__file__).resolve().parent
p = A/'run1/report.json'
original = p.read_bytes()
r = json.loads(original)
acq = json.loads((A/'run1/acquisition.json').read_text())
counts = [x['discarded_source_gaps'] for x in acq]
stamps = [x['first_ray_stamp_s'] for x in acq]
checks = dict(one_lap=r['completed_laps'] >= 1, target_progress=r['progress_m'] >= r['target_progress_m'],
    no_evaluated_collision=r['collision_samples'] == r['stop_collision_samples'] == 0,
    stop_stable=r['stop_stable'], dense_stop=r['dense_pose_stop_evidence']['passed'],
    actuation=r['actuation_passed'], route=r['route_complete'], signal=r['official_signal_evidence']['passed'],
    controller_sensor_only=r['sensor_only_inputs'], guard_sensor_only=r['safety_sensor_only_inputs'],
    nominal_lidar=r['lidar_nominal_interface_passed'], native_lidar=r['native_acquisition_verified'],
    sources=r['source_unchanged'], shutdown=r['shutdown_clean'], no_residue=not r['remaining_test_pids'])
result = dict(original_report_sha256=hashlib.sha256(original).hexdigest(), original_passed=r['passed'],
    functional_checks=checks, functional_checks_passed=all(checks.values()),
    acquisition=dict(samples=len(acq), first_counter=counts[0], last_counter=counts[-1],
        first_published_s=acq[0]['source_stamp_at_publish_s'], run_started_s=r['start_time_sim_s'],
        first_counter_predates_start=acq[0]['source_stamp_at_publish_s'] < r['start_time_sim_s'],
        later_counter_increments=sum(b-a for a,b in zip(counts,counts[1:])),
        max_completed_source_interval_s=max(b-a for a,b in zip(stamps,stamps[1:])),
        max_capture_error_s=max(x['max_capture_time_error_s'] for x in acq)),
    limitations=['original whole-run acquisition-loss-free condition failed; report unchanged',
                 '0 contact messages do not independently certify no physical contact',
                 '0.25 requested real-time factor; not Jetson real-time or multi-car proof'])
assert p.read_bytes() == original
(A/'assessment.json').write_text(json.dumps(result, indent=2)+'\n')
print(json.dumps(result, indent=2))
