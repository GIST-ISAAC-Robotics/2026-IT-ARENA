"""좌우 바퀴 환산 속도·기울기·높이를 같은 표로 본다. 평가 전용."""
import gzip
import json
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
R = .0325


def load(name):
    with gzip.open(HERE / f'extract_{name}.json.gz', 'rt', encoding='utf-8') as f:
        return json.load(f)


def dump(name, lo, hi):
    rows = [r for r in load(name)['trajectory'] if lo <= r['time'] <= hi]
    print(f'# {name} {lo}-{hi}')
    print(' time   prog   body  vlat  vL    vR    mean  appl   torq  tL      tR     z_mm   pitch_d roll_d yaw_r  latA')
    prev = None
    for r in rows:
        vl, vr = r['left_speed_rad_s'] * R, r['right_speed_rad_s'] * R
        print(f"{r['time']:6.3f} {r['progress_m']:6.3f} {r['truth_longitudinal_mps']:5.2f} {r['truth_lateral_mps']:+5.2f} "
              f"{vl:5.2f} {vr:5.2f} {(vl+vr)/2:5.2f} {r['applied_target_mps']:5.2f} {r['motor_torque_nm']:+6.3f} "
              f"{r['left_torque_nm']:+6.3f} {r['right_torque_nm']:+6.3f} {r['truth_z_m']*1000:6.2f} "
              f"{r['truth_pitch_rad']*57.3:+6.2f} {r['truth_roll_rad']*57.3:+6.2f} {r['truth_yaw_rate_rad_s']:+5.2f} "
              f"{r['truth_yaw_rate_rad_s']*r['truth_longitudinal_mps']:+5.2f}")


if __name__ == '__main__':
    args = sys.argv[1:]
    for i in range(0, len(args), 3):
        dump(args[i], float(args[i + 1]), float(args[i + 2]))
