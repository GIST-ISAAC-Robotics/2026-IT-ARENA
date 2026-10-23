"""보존된 C1 합성 입력에서 시간 오프셋과 형상 잔차의 식별 한계를 진단한다.

점 ID/벽 ID/정답은 평가 전용이다. 자동 교정/운영 보정 상수 추정기가 아니다.
"""
import hashlib
import json
from pathlib import Path
import shutil

import numpy as np

from audit_c1_motion_pipeline import NOW, motion, reference_points
from arena_autonomy.lidar_point_review import project_acquisition_points

ROOT = Path(__file__).resolve().parents[4]
OUT = Path(__file__).resolve().parent
BASE = OUT.parent/'c1_motion_pipeline_v1'


def rigid_residual(points, reference):
    """같은 원본의 대응점에 최적 SE(2) 정렬. 대응 자체는 평가기가 제공한다."""
    center_p, center_r = points.mean(axis=0), reference.mean(axis=0)
    p, r = points-center_p, reference-center_r
    dot = np.sum(p*r)
    cross = np.sum(p[:, 0]*r[:, 1]-p[:, 1]*r[:, 0])
    angle = np.arctan2(cross, dot)
    c, s = np.cos(angle), np.sin(angle)
    rotation = np.array([[c, -s], [s, c]])
    translation = center_r-center_p@rotation.T
    after = points@rotation.T+translation-reference
    before = points-reference
    return dict(before_max_m=float(np.linalg.norm(before, axis=1).max()),
                aligned_max_m=float(np.linalg.norm(after, axis=1).max()),
                aligned_rms_m=float(np.sqrt(np.mean(np.sum(after*after, axis=1)))),
                best_rotation_rad=float(angle), best_translation_m=translation.tolist())


def wall_fit(points, wall_ids):
    """독립 벽 ID로 TLS 직선을 맞춘다. 실제 자동 벽 연관을 구현한 것이 아니다."""
    result = []
    for wall_id in np.unique(wall_ids):
        group = points[wall_ids == wall_id]
        if len(group) < 3:
            raise ValueError('insufficient wall samples')
        centered = group-group.mean(axis=0)
        _, _, axes = np.linalg.svd(centered, full_matrices=False)
        residual = centered@axes[-1]
        result.append(dict(wall_id=int(wall_id), count=len(group),
            rms_m=float(np.sqrt(np.mean(residual**2))), max_m=float(abs(residual).max())))
    return result


def main():
    # Known rigid rotation/translation must be removed, while nonrigid deformation
    # must survive. These checks are distinct from the C1 outcomes below.
    test = np.array([[0., 0.], [1., 0.], [1., 1.], [-1., .3]])
    angle = .3
    rot = np.array([[np.cos(angle), -np.sin(angle)], [np.sin(angle), np.cos(angle)]])
    shifted = test@rot.T+[.2, -.1]
    assert rigid_residual(shifted, test)['aligned_max_m'] < 1e-12
    shifted[0] += [.1, .1]
    assert rigid_residual(shifted, test)['aligned_max_m'] > .05

    (OUT/'source').mkdir(exist_ok=False)
    sources = ['scripts/audit_c1_motion_pipeline.py',
               'src/arena_autonomy/arena_autonomy/lidar_motion.py',
               'src/arena_autonomy/arena_autonomy/lidar_point_review.py']
    before = {}
    for name in sources:
        source = ROOT/name
        before[name] = hashlib.sha256(source.read_bytes()).hexdigest()
        dest = OUT/'source'/name
        dest.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(source, dest)
    rows, input_hashes = [], {}
    for name, v, w in [('straight', 1., 0.), ('left', 1., 1.), ('right', 1., -1.)]:
        saved = BASE/(name+'_phase0_100ms.npz')
        input_hashes[str(saved.relative_to(ROOT))] = hashlib.sha256(saved.read_bytes()).hexdigest()
        with np.load(saved, allow_pickle=False) as data_file:
            data = dict(data_file)
        wall_ids = 2*data['wall_axes']+(data['hits'][np.arange(500), data['wall_axes']] > 0)
        history = motion(v, w)
        raw = data['range_q2']/4000.
        baseline, _, _ = project_acquisition_points(history, raw, data['angles'], data['times'],
                                                   NOW, .05, 12., timing_verified=True)
        for offset in (0., -.01, .01):
            points, indices, _ = project_acquisition_points(history, raw, data['angles'],
                data['times']+offset, NOW, .05, 12., timing_verified=True)
            aligned = rigid_residual(points, baseline)
            fitting = wall_fit(points, wall_ids)
            baseline_fit = wall_fit(baseline, wall_ids)
            fit_diff = max(abs(a['rms_m']-b['rms_m']) for a, b in zip(fitting, baseline_fit))
            exact = reference_points(data, indices, v, w)
            row = dict(motion=name, offset_s=offset, relative_to_zero_offset=aligned,
                       actual_endpoint_error_max_m=float(np.linalg.norm(points-exact, axis=1).max()),
                       per_wall_fits=fitting, wall_rms_change_max_m=fit_diff)
            # Constant-twist time shift is globally rigid: shape cannot identify it.
            row['checks'] = dict(rigid_shape_unchanged=aligned['aligned_max_m'] < 1e-11,
                                 wall_fit_unchanged=fit_diff < 1e-11,
                                 nonzero_offset_moves_points=offset == 0 or aligned['before_max_m'] > .009)
            rows.append(row)
    unchanged = all(hashlib.sha256((ROOT/name).read_bytes()).hexdigest() == sha
                    for name, sha in {**before, **input_hashes}.items())
    passed = unchanged and all(all(row['checks'].values()) for row in rows)
    report = dict(scope='saved_synthetic_constant_twist_point_correspondence_wall_ids_are_evaluation_truth',
                  cases=rows, source_hashes=before, input_hashes=input_hashes,
                  source_and_inputs_unchanged=unchanged, checks_passed=passed,
                  hardware_verified=False, calibration_estimated=False, ready_to_drive=False)
    (OUT/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
    (OUT/'exit.txt').write_text(('0' if passed else '1')+'\n')
    print(json.dumps(dict(cases=len(rows), checks_passed=passed, unchanged=unchanged)))
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
