"""완료 입력에서 수신 gyro와 평가용 정지 구간을 대조한다. 제어기에 정답을 넣지 않는다."""
import argparse
import hashlib
import json
from pathlib import Path
import statistics


def sha(path):
    with path.open('rb') as stream:
        return hashlib.file_digest(stream, 'sha256').hexdigest()


def audit(folder):
    paths = [folder / name for name in ('trajectory.jsonl', 'follow_decisions.jsonl')]
    before = {p.name: sha(p) for p in paths}
    spans, current = [], []
    with paths[0].open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            braking = row.get('braking_diagnostics') or {}
            stamp = braking.get('sim_time_s')
            yaw = braking.get('truth_yaw_rate_rad_s')
            longitudinal = braking.get('truth_longitudinal_mps')
            lateral = braking.get('truth_lateral_mps')
            valid = (all(isinstance(x, (float, int)) for x in (stamp, yaw, longitudinal, lateral))
                and abs(yaw) < .002 and abs(longitudinal) < .005 and abs(lateral) < .005)
            if current and (not valid or stamp - current[-1] > .15 or stamp <= current[-1]):
                if current[-1] - current[0] >= .5:
                    spans.append([current[0] + .1, current[-1] - .1])
                current = []
            if valid:
                current.append(stamp)
    if current and current[-1] - current[0] >= .5:
        spans.append([current[0] + .1, current[-1] - .1])
    samples = {}
    all_stamps = set()
    header = None
    with paths[1].open(encoding='utf-8') as stream:
        for line in stream:
            row = json.loads(line)
            if row['kind'] == 'header':
                header = row['data']['motion_config']
            if row['kind'] != 'call':
                continue
            gyro = row['data']['input']['history']['gyro']
            for stamp, value in zip(gyro['times'], gyro['values'], strict=True):
                all_stamps.add(stamp)
                if not any(a <= stamp <= b for a, b in spans):
                    continue
                if stamp in samples and samples[stamp] != value:
                    raise ValueError('same acquisition has different gyro values')
                samples[stamp] = value
    values = list(samples.values())
    unchanged = all(sha(p) == before[p.name] for p in paths)
    if not unchanged:
        raise RuntimeError('input changed during collection')
    return dict(scope='accepted gyro samples in evaluation-selected near-stationary intervals',
        input_sha256=before, input_unchanged=unchanged,
        stationary_criteria=dict(max_yaw_rate_rad_s=.002, max_planar_component_mps=.005,
            maximum_pose_gap_s=.15, minimum_interval_s=.5, edge_trim_s=.1),
        stationary_intervals_s=spans, unique_gyro_samples=len(all_stamps),
        stationary_gyro_samples=len(values),
        stationary_gyro_mean_rad_s=statistics.fmean(values) if values else None,
        stationary_gyro_std_rad_s=statistics.stdev(values) if len(values) > 1 else None,
        configured_gyro_bias_rad_s=header.get('gyro_bias_rad_s') if header else None,
        limitations=['truth selects evaluation intervals only; never enters driving control',
            'accepted z samples are after the actual callback guard, not raw three-axis measurements',
            'sampled near-stationary intervals do not prove zero motion at every instant'])


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('folder', type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    result = audit(args.folder)
    with args.output.open('x', encoding='utf-8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
    print(json.dumps({k: v for k, v in result.items() if k not in ('input_sha256', 'stationary_intervals_s')}, ensure_ascii=False))
