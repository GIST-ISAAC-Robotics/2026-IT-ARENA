"""평가 정답과 모터축 추정의 분리. 제어기에 입력하지 않는 사후 진단."""
import hashlib
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
cases = {
    'r11_baseline': ROOT / 'artifacts/validation/2026-11-07/junction_handoff_v1/run1/trace.json',
    'straight_first': ROOT / 'artifacts/validation/2026-11-07/solo_straight_newcourse_v1/run1/trace.json',
    'r14_oblique': HERE / 'run1/trace.json',
}
result = {}
for name, path in cases.items():
    rows = json.loads(path.read_text())
    selected = []
    for progress in (33., 33.8, 34.1, 34.5, 34.8, 35.1):
        row = min(rows, key=lambda r: abs(r['progress_m'] - progress))
        d, a = row['dynamics'], row['autonomy']
        selected.append({
            'time_s': row['time'], 'progress_m': row['progress_m'],
            'truth_speed_mps': d['truth_longitudinal_mps'],
            'wheel_mean_mps': d['wheel_surface_speed_mps'],
            'measured_motor_speed_mps': a.get('measured_speed_mps'),
            'left_wheel_rad_s': d['left_speed_rad_s'], 'right_wheel_rad_s': d['right_speed_rad_s'],
            'safe_target_mps': row['safe_drive_speed_mps'], 'motor_torque_nm': d['motor_torque_nm'],
            'truth_z_m': d['truth_z_m'], 'truth_roll_rad': d['truth_roll_rad'],
            'truth_pitch_rad': d['truth_pitch_rad'], 'reason': a.get('reason'),
            'centerline_error_m': row['centerline_error_m'],
        })
    result[name] = {'trace_sha256': hashlib.sha256(path.read_bytes()).hexdigest(), 'samples': selected}
result['interpretation'] = (
    'r14 has one rear wheel slowing/reversing while the other rolls and the body remains near 2m/s; '
    'the motor-shaft mean underestimates body travel before path loss. This establishes wheel/body '
    'disagreement, not its contact-force cause. No direct ground-truth feedback is proposed. '
    'This run failed before the previously targeted unknown-ray gap and does not test its escape.'
)
(HERE / 'departure_diagnosis.json').write_text(json.dumps(result, ensure_ascii=False, indent=2) + '\n', encoding='utf-8')
print('saved departure_diagnosis.json: three traces, six samples each')
