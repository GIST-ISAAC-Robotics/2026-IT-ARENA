"""보고서 수치 검산: 월드 차이, 바퀴 힘 규모 추정. 전 구간 재생 표는 압축 보관한다."""
import difflib
import gzip
import json
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
R, MASS, I_WHEEL = .0325, 2.0, .04 * .0325 ** 2 / 2


def world_diff(a, b):
    la = (ROOT / a / 'runtime_world.sdf').read_text(encoding='utf-8').splitlines()
    lb = (ROOT / b / 'runtime_world.sdf').read_text(encoding='utf-8').splitlines()
    changed = [(x, y) for x, y in zip(la, lb) if x != y]
    return dict(lines_a=len(la), lines_b=len(lb), changed_lines=len(changed),
                samples=[(x.strip()[:150], y.strip()[:150]) for x, y in changed[:3]])


with gzip.open(HERE / 'extract_combined_failed.json.gz', 'rt', encoding='utf-8') as f:
    rows = json.load(f)['trajectory']
window = [r for r in rows if 45.085 <= r['time'] <= 45.591]
torque = [abs(r['right_torque_nm']) for r in window]
mean_t = sum(torque) / len(torque)
dt = window[-1]['time'] - window[0]['time']
vr0, vr1 = window[0]['right_speed_rad_s'], window[-1]['right_speed_rad_s']
vl = [r['left_speed_rad_s'] * R for r in window]
body = [r['truth_longitudinal_mps'] for r in window]
# 45.039(하강 직전) ~ 45.499(최저)의 오른바퀴 각감속도로 무부하 관성 토크를 계산한다.
a = next(r for r in rows if abs(r['time'] - 45.039) < .002)
b = next(r for r in rows if abs(r['time'] - 45.499) < .002)
alpha = (a['right_speed_rad_s'] - b['right_speed_rad_s']) / (b['time'] - a['time'])
DAMPING = 5e-6      # 실행 runtime_vehicle.sdf의 바퀴 관절 점성값
mean_omega = sum(abs(r['right_speed_rad_s']) for r in window)/len(window)
inertia = I_WHEEL*abs(alpha)
viscous = DAMPING*mean_omega
scale = mean_t+inertia+viscous
force = dict(window_s=[45.085, 45.591], samples=len(window), mean_abs_wheel_torque_nm=round(mean_t, 4),
             right_wheel_alpha_rad_s2=round(alpha, 1), wheel_inertia_torque_nm=round(inertia, 4),
             joint_viscous_torque_nm=round(viscous, 6), torque_terms_sum_nm=round(scale, 4),
             wheel_force_scale_n=round(scale/R, 3), nominal_coulomb_n=round(.9*4.905, 2),
             scale_to_nominal=round(scale/R/(.9*4.905), 3), body_net_force_n=round(MASS*(body[0]-body[-1])/dt, 3),
             left_minus_body_max_mps=round(max(abs(x-y) for x, y in zip(vl, body)), 3),
             scope='약 46 ms 표본의 토크 균형 규모 추정이다. 순간 상한이 아니며 접촉·해석기 기여는 기록이 없어 분리하지 못했다.')
result = dict(world=dict(rise_vs_combined=world_diff('race_acceleration_v1/run1', 'qualifying_profile_v1/run1'),
                         turn_vs_combined=world_diff('turn_preview_v1/run4', 'qualifying_profile_v1/run1')),
              free_wheel_force=force)
with (HERE / 'evidence_checks.json').open('w', encoding='utf-8', newline='\n') as f:
    json.dump(result, f, ensure_ascii=False, indent=1)
print(json.dumps(result, ensure_ascii=False, indent=0)[:1500])
for name in ('rise_only_baseline', 'rise_only_truth_history', 'turn_only_baseline', 'turn_only_truth_history',
             'combinedfull_baseline', 'combinedfull_truth_history'):
    p = HERE / 'replay' / f'{name}.json'
    if p.exists():
        with gzip.open(p.with_suffix('.json.gz'), 'wb') as f:
            f.write(p.read_bytes())
        p.unlink()
