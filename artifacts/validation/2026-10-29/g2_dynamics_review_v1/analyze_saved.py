"""저장된 좌우 G2 주행의 정지 명령/바퀴 헛돎을 분리한다. 평가 전용."""
import hashlib
import json
from pathlib import Path
import statistics

BASE = Path(__file__).resolve().parent
PARENT = BASE.parent


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def summary(folder):
    path = PARENT / folder / 'run1' / 'trace.json'
    rows = json.loads(path.read_text(encoding='utf-8'))
    moving = [r for r in rows if r.get('elapsed_since_green_s') is not None]
    late = [r for r in moving if 6 <= r['elapsed_since_green_s'] <= 11.9]
    def values(field, diagnostic=False):
        return [(r['braking_diagnostics'] if diagnostic else r)[field] for r in late]
    def stats(field, diagnostic=False):
        v = values(field, diagnostic)
        return dict(min=min(v), median=statistics.median(v), max=max(v))
    calls_path = path.parent / 'g2_decisions.jsonl'
    resets = []
    with calls_path.open(encoding='utf-8') as stream:
        for line in stream:
            entry = json.loads(line)
            if entry['kind'] != 'call':
                continue
            data = entry['data']
            if data['output'][2].get('reason') == 'entry_guide_input_reset':
                resets.append(dict(reference_s=data['input']['reference_s'],
                                   speed_mps=data['input']['speed_mps']))
    return dict(source=str(path.relative_to(PARENT)), sha256=sha(path),
                decision_sha256=sha(calls_path), window_since_green_s=[6, 11.9],
                samples=len(late), drive=stats('drive_speed_mps'), safe=stats('safe_speed_mps'),
                truth_speed=stats('ground_speed_mps'),
                left_rear_rad_s=stats('left_speed_rad_s', True),
                right_rear_rad_s=stats('right_speed_rad_s', True),
                motor_torque_nm=stats('motor_torque_nm', True),
                safety_reasons=sorted(set(r['safety_reason'] for r in late)),
                longitudinal_progress_m=late[-1]['progress_m']-late[0]['progress_m'],
                guide_input_resets=resets,
                interpretation='Saved evaluation truth only; no controller input or contact-force inference')


if __name__ == '__main__':
    report = dict(right_target=summary('g2_entry_gazebo_v1'),
                  left_target=summary('g2_entry_left_v1'))
    (BASE/'saved_dynamics.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(report, indent=2))
