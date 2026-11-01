"""과거 궤적과 고정 옆차 각본의 만남만 예상한다. 새 폐루프 결과가 아니다."""
from hashlib import sha256
import json
import math
import numpy as np
import prepare as plan
from prelap_signal_eval import FIXTURE
from validate_g1_traffic import actor_start_s, actor_pose_at, load_centerline
from g1_traffic_eval import rectangle
from review import longest


def preview():
    config, overlay = plan.expected()
    spec = overlay['additional_actors'][0]
    line, scene = load_centerline(FIXTURE)
    start = actor_start_s(config, scene, 0, dict(actor_start_center_gap_m=spec['start_center_gap_m']))
    rows = [row for row in json.loads(plan.REFERENCE.read_bytes())
            if row.get('elapsed_since_green_s') is not None and row['elapsed_since_green_s'] >= 0]
    stamps = np.array([row['elapsed_since_green_s'] for row in rows])
    times = np.arange(max(.1, stamps[0]), min(17., stamps[-1])-.02, .02)
    own = np.column_stack([np.interp(times, stamps, [row[k] for row in rows]) for k in ('x', 'y')]
                           + [np.interp(times, stamps, np.unwrap([row['yaw'] for row in rows]))])
    other = np.array([actor_pose_at(line, start, spec['schedule'], float(t), plan.LATERAL) for t in times])
    previous = np.array([actor_pose_at(line, start, spec['schedule'], float(t-.1), plan.LATERAL) for t in times])
    other_speed = np.linalg.norm(other[:, :2]-previous[:, :2], axis=1)/.1
    own_speed = np.interp(times, stamps, [row['ground_speed_mps'] for row in rows])
    delta = other[:, :2]-own[:, :2]
    c, s = np.cos(own[:, 2]), np.sin(own[:, 2])
    longitudinal, lateral = c*delta[:, 0]+s*delta[:, 1], -s*delta[:, 0]+c*delta[:, 1]
    angle = lambda a: np.abs(np.arctan2(np.sin(a), np.cos(a)))*180/math.pi
    turning = angle(own[:, 2]-rows[0]['yaw'])
    beside = (np.abs(longitudinal) <= .30) & (np.abs(lateral) >= .15) & (np.abs(lateral) <= .45)
    moving = (own_speed >= .10) & (other_speed >= .10)
    parallel = beside & moving & (angle(other[:, 2]-own[:, 2]) <= 45.)
    corner = parallel & (turning >= 30.) & (turning <= 150.)
    gaps, overlaps = [], 0
    for a, b in zip(own, other):
        first, second = rectangle(*a, .20, .15), rectangle(*b, .20, .15)
        gaps.append(first.distance(second))
        overlaps += int(first.intersection(second).area > 1e-9)
    return dict(scope='과거 앞차 추종 궤적 위의 가상 배치. 상대 추가 후 실제 자차 궤적은 달라진다.',
                reference_sha256=sha256(plan.REFERENCE.read_bytes()).hexdigest(),
                script_sha256=sha256(plan.HERE.joinpath('placement_preview.py').read_bytes()).hexdigest(),
                moving_beside=longest(times, parallel), corner_beside=longest(times, corner),
                sampled_minimum_body_gap_m=min(gaps), sampled_overlap_count=overlaps,
                samples=len(times), time_step_s=.02)


if __name__ == '__main__':
    result = preview()
    plan.write(plan.HERE/'placement_preview.json', result)
    print(json.dumps(result, ensure_ascii=False))
