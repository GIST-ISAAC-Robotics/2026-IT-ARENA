"""완료된 명목·회전 오차·짧은 안쪽 지지 보완 주행의 진행과 정체를 비교한다."""
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE = Path(__file__).resolve().parent
RUNS = {
    'Nominal sensor model': BASE.parent / 'official_traffic_v1/run2_course',
    'Rotation error, original': BASE / 'run1_course',
    'Rotation error, observed prefix': BASE / 'run2_course',
}


def main():
    image, metadata = BASE / 'rotation_comparison.png', BASE / 'rotation_comparison.json'
    if image.exists() or metadata.exists():
        raise FileExistsError('preserved comparison already exists')
    results = {}
    fig, axes = plt.subplots(3, 1, figsize=(11, 8), sharex=True, layout='constrained')
    for label, folder in RUNS.items():
        report = json.loads((folder / 'report.json').read_text(encoding='utf-8'))
        if not report.get('shutdown_clean'):
            raise ValueError('requires completed run and shutdown: ' + label)
        path = folder / 'trajectory.jsonl'
        raw = path.read_bytes()
        start, end = (report['case_window'][key] for key in ('green_sim_s', 'end_sim_s'))
        rows = [row for line in raw.decode('utf-8').splitlines()
                if start <= (row := json.loads(line))['t'] <= end]
        if not rows:
            raise ValueError('empty driving window')
        times = [row['t'] - start for row in rows]
        for axis, field in zip(axes, ('progress_m', 'drive_speed_mps', 'ground_speed_mps')):
            axis.plot(times, [row[field] for row in rows], label=label, alpha=.85, lw=1.3)
        results[label] = dict(trajectory_sha256=hashlib.sha256(raw).hexdigest(),
            passed=report['passed'], run_valid=report['run_valid'],
            failed_gates=[key for key, value in report['gates'].items() if not value],
            samples=len(rows), progress_m=rows[-1]['progress_m'])
        assert path.read_bytes() == raw
    axes[0].axhline(19.45737, color='grey', ls=':', lw=1, label='Original stress-run stall')
    for axis, label in zip(axes, ('Progress (m)', 'Planner command (m/s)', 'Measured body speed (m/s)')):
        axis.set_ylabel(label)
        axis.grid(alpha=.25)
        axis.legend(loc='upper left', fontsize=8)
    axes[-1].set_xlabel('Simulation seconds after applied green')
    fig.suptitle('Official signal + scripted traffic | rotation-error comparison\n'
                 'One run per condition; bump OFF; assumed errors, not hardware validation')
    fig.savefig(image, dpi=150)
    metadata.write_text(json.dumps(results, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(results))


if __name__ == '__main__':
    main()
