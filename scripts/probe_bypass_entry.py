"""평가용 상자를 500개 광선으로 변환한 뒤 G2 진입 후보에 센서 장면만 전달한다."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import zipfile

from arena_autonomy.bypass_entry_trial import compare_entries
from arena_autonomy.local_scene import scene_from_scan
from g0_geometry import Footprint, TruthBox, ray_box_distance

ROOT = Path(__file__).resolve().parents[1]


def sensor_scene(targets=(), *, wall_half_width=.425, missing=(), now=.5, first=.4,
                 target_height=.12, rear_half=30., epoch=0):
    """자차가 정지한 초기 관측. 표적은 평가에만 있고 제어기 입력에서는 label 제거."""
    boxes = [TruthBox(2., sign*(wall_half_width+.025), .4, Footprint(8., .05), label='wall')
             for sign in (-1, 1)]
    boxes += [TruthBox(x, 0., .4, Footprint(.05, 2.), label='wall') for x in (-2.025, 6.025)]
    boxes += [TruthBox(x, y, target_height) for x, y in targets]
    ranges, labels = [], []
    for i in range(500):
        angle = -math.pi+i*math.tau/500
        hits = [(distance, box.label) for box in boxes if (distance := ray_box_distance(
            (.06, 0., .095), (math.cos(angle), math.sin(angle), 0.), box, 0., 12.)) is not None]
        distance, label = min(hits, default=(math.inf, None))
        ranges.append(distance if distance >= .05 else math.nan)
        labels.append(label)
    for index in missing:
        ranges[index] = math.inf
    stamps = [first+i*.0002 for i in range(500)]
    scene = scene_from_scan(ranges, angle_min=-math.pi, angle_increment=math.tau/500,
        range_min=.05, range_max=12., acquired_at_s=stamps, t_ref_s=now,
        source_epoch=epoch, rear_half_angle_deg=rear_half)
    return scene, dict(target_returns=sum(label == 'target' and ray.status == 'return'
        for label, ray in zip(labels, scene.rays)), synthetic_static_sensor=True,
        truth_used_only_for_ray_cast=True, low_obstacle_detection_verified=False)


def case_specs():
    cases = [('empty', dict())]
    for x in (.5, .8, 1.2):
        for y in (0., -.15, .15):
            cases.append((f'target_x{x}_y{y}', dict(targets=((x, y),))))
    cases += [
        ('close_front', dict(targets=((.22, 0.),))),
        ('two_targets', dict(targets=((.8, -.15), (.8, .15)))),
        ('narrow_front_missing', dict(missing=(250,))),
        ('wide_front_missing', dict(missing=tuple(range(230, 271)))),
        ('side_missing', dict(missing=(375,))),
        ('narrow_walls', dict(wall_half_width=.20)),
        ('low_target_unseen', dict(targets=((.8, 0.),), target_height=.06)),
        ('rear_unmasked_counterfactual', dict(targets=((.8, .15),), rear_half=0.)),
        ('old_scan', dict(now=.71)),
    ]
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    if not output.is_relative_to(ROOT/'artifacts/validation'):
        parser.error('validation output required')
    output.mkdir(parents=True, exist_ok=False)
    sources = list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')) + [
        Path(__file__), ROOT/'scripts/g0_geometry.py', ROOT/'tests/test_bypass_entry_trial.py']
    raw = {str(p.relative_to(ROOT)):p.read_bytes() for p in sources}
    with zipfile.ZipFile(output/'sources.zip', 'x', zipfile.ZIP_DEFLATED) as archive:
        for name, data in raw.items(): archive.writestr(name, data)
    rows = []
    for name, kwargs in case_specs():
        scene, truth_evaluation = sensor_scene(**kwargs)
        result = compare_entries(scene, scene.t_ref_s, 0., 0., .02)
        rows.append(dict(case=name, evaluation=truth_evaluation, result=result))
    hashes = {name:hashlib.sha256(data).hexdigest() for name, data in raw.items()}
    unchanged = all((ROOT/name).read_bytes() == data for name, data in raw.items())
    report = dict(scope='synthetic_initial_sensor_entry_comparison_not_driving',
        source_sha256=hashes, source_unchanged=unchanged, control_authorized=False, cases=rows)
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n', encoding='utf-8')
    print(json.dumps([dict(case=row['case'], reason=row['result']['reason'],
        selected=row['result']['selected'], target_returns=row['evaluation']['target_returns']) for row in rows]))
    if not unchanged: raise RuntimeError('source changed during comparison')


if __name__ == '__main__': main()
