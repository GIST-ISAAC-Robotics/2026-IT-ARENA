#!/usr/bin/env python3
"""저장 C1 원시 기록의 제한 반복성 진단. 정확도 교정/시뮬 잡음 설정용이 아님.

공개 출력에는 방위별 거리나 실내 점군을 넣지 않는다. 원천 각도/유효값을
유지한 최근접 선택만 사용하며 보간·공백 채우기·노이즈 제거는 하지 않는다.
"""
import argparse
import csv
import hashlib
import json
from pathlib import Path

import numpy as np


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    a = a[np.isfinite(a)]
    if not a.size:
        return dict(n=0)
    return dict(n=int(a.size), p50=float(np.percentile(a, 50)),
                p95=float(np.percentile(a, 95)), p99=float(np.percentile(a, 99)),
                maximum=float(np.max(a)))


def analyze(scans_path, points_path):
    hashes = dict(scans=digest(scans_path), points=digest(points_path))
    with scans_path.open(newline='') as stream:
        counts = {int(r['scan']): int(r['count']) for r in csv.DictReader(stream)
                  if r['measured'] == '1' and int(r['result']) == 0 and int(r['count']) > 0}
    rows = {sid: [] for sid in counts}
    with points_path.open(newline='') as stream:
        for row in csv.DictReader(stream):
            sid = int(row['scan'])
            if sid in rows:
                rows[sid].append((int(row['index']), int(row['angle_q14']),
                                  int(row['dist_mm_q2']), int(row['quality'])))
    ordered = sorted(counts)
    if not ordered:
        raise ValueError('no measured successful scans')
    scans = []
    for sid in ordered:
        raw = rows[sid]
        if len(raw) != counts[sid] or [r[0] for r in raw] != list(range(counts[sid])):
            raise ValueError('source count/order mismatch')
        scans.append(np.array(raw, dtype=float))
    targets = np.arange(360, dtype=float)
    outputs = []
    for tolerance_deg in (.1, .2, .3):
        values = np.full((len(scans), 360), np.nan)
        actual_angles = values.copy()
        for i, raw in enumerate(scans):
            angles = raw[:, 1] * 360. / 65536.
            # Select nearest actual ray BEFORE filtering its range; an invalid
            # ray must not be replaced by a more distant valid neighbouring ray.
            delta = np.abs((angles[:, None] - targets + 180.) % 360. - 180.)
            nearest = np.argmin(delta, axis=0)
            ranges = raw[nearest, 2] / 4000.
            ok = ((delta[nearest, np.arange(360)] <= tolerance_deg)
                  & (raw[nearest, 3] > 0) & (ranges >= .15) & (ranges <= 3.))
            values[i, ok] = ranges[ok]
            actual_angles[i, ok] = angles[nearest[ok]]
        populated = np.sum(np.isfinite(values), axis=0)
        medians = np.array([np.median(values[np.isfinite(values[:, j]), j])
                            if populated[j] else np.nan for j in range(360)])
        coverage = populated >= max(10, int(np.ceil(.4 * len(scans))))
        # Use the cross-scan median geometry, not each sample's residual, to
        # identify gentle regions; retain BOTH filtered and unfiltered outputs.
        gentle = (coverage & np.roll(coverage, 1) & np.roll(coverage, -1)
                  & (np.abs(medians - np.roll(medians, 1)) <= .020)
                  & (np.abs(medians - np.roll(medians, -1)) <= .020))
        adjacent = np.diff(np.array(ordered)) == 1
        angular_match = np.abs((actual_angles[1:] - actual_angles[:-1] + 180.) % 360. - 180.) <= .08
        pair_mask = adjacent[:, None] & angular_match
        groups = {}
        for name, mask in [('coverage_only', coverage), ('gentle_median_geometry', gentle)]:
            residual = np.abs(values[:, mask] - medians[mask]) * 1000.
            mad_per_bin = [float(np.nanmedian(np.abs(values[:, j] - medians[j]))) * 1000.
                           for j in np.flatnonzero(mask)]
            differences = (values[1:] - values[:-1]) * 1000.
            matched = differences[pair_mask & mask[None, :]]
            groups[name] = dict(angular_bins=int(mask.sum()),
                absolute_residual_from_bin_median_mm=stats(residual.ravel()),
                per_bin_mad_mm=stats(mad_per_bin),
                consecutive_absolute_difference_mm=stats(np.abs(matched)))
        outputs.append(dict(nearest_ray_tolerance_deg=tolerance_deg, groups=groups))
    unchanged = hashes == dict(scans=digest(scans_path), points=digest(points_path))
    if not unchanged:
        raise ValueError('source changed during audit')
    return dict(scan_count=len(scans), measurement_count=sum(counts.values()),
                inputs_sha256=hashes, inputs_unchanged=unchanged, comparisons=outputs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scans', type=Path, required=True)
    parser.add_argument('--points', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('output already exists; preserve previous evidence')
    result = analyze(args.scans, args.points)
    result.update(schema=1, script_sha256=digest(Path(__file__)),
        method=dict(target_step_deg=1., range_selection_m=[.15, 3.],
            coverage_fraction_min=.4, count_min=10,
            median_neighbour_delta_max_m=.020, consecutive_actual_angle_delta_max_deg=.08),
        limitations=['short tilted obstructed indoor recording without distance truth',
            'unknown scene or sensor motion; direction-dependent selection and angular mismatch remain',
            'not accuracy, calibration, all-surface performance or an adopted simulation noise model',
            'gentle-geometry selection excludes edges; inspect coverage-only group too'],
        independent_uniform_30mm_theoretical=dict(stddev_mm=30./np.sqrt(3.),
            consecutive_absolute_difference_p95_mm=60.*(1.-np.sqrt(.05))))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, ensure_ascii=False)+'\n', encoding='utf8')
    print(json.dumps(dict(output=str(args.output), scans=result['scan_count'],
                         inputs_unchanged=result['inputs_unchanged'])))


if __name__ == '__main__':
    main()
