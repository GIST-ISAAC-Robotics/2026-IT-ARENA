"""오프라인 개입 후보 생성: 런타임 observed_measured_v1 평행 후보에서 스윕 여유 계산만 바꾼 사본을 만든다.

바꾸는 것: 현재 차체 옆(거리 0에서 |x| <= 길이/2+팽창)이면서 옆면 밖 고정 측정 여유(0.025+measurement_padding)보다
멀고, 광선 나이 x 물체 이동 상한 팽창 때문에만 차체와 겹치는 점은 고정 여유로만 스윕한다. 그 밖의 점(앞쪽,
고정 여유 안쪽)은 기존 광선별 팽창 그대로다. 직선 평행 후보(횡변위 1 cm 이하)에서만 쓰이며 운영 코드에 연결하지 않는다.
이유: 정지해도 옆 물체의 가정된 측방 접근을 줄이지 못하는 거리 0 겹침이 재개 확인을 반복 초기화하는지 개입 비교한다.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
SRC = ROOT/'src/arena_autonomy/arena_autonomy'

SPEED_PAIRS = [
    ('"""선택형 시험, 기본 미사용: rear_lane_keep_policy=observed_measured_v1 전용 평행 경로 평가.',
     '"""오프라인 개입 후보(운영 미연결): observed_measured_v1 평행 경로 평가의 옆면 나이 팽창 분리 사본.'),
    ("""    padding = per_ray_padding(scene, .025+config.measurement_padding_m, config.object_motion_bound_mps)
    clearance = min(swept_limit(points, angle, config.wheelbase_m, horizon=2.5, margin=padding,
                               length=config.length_m, width=config.width_m)
                    for angle in (steering, previous))
""", """    fixed = .025+config.measurement_padding_m
    padding = per_ray_padding(scene, fixed, config.object_motion_bound_mps)
    # 오프라인 후보: 현재 차체 옆에서 나이 팽창으로만 겹치는 점은 고정 여유로 스윕한다(정지로 줄지 않는 겹침).
    beside = ((np.abs(points[:, 0]) <= config.length_m/2+padding)
              & (np.abs(points[:, 1]) > config.width_m/2+fixed)
              & (np.abs(points[:, 1]) <= config.width_m/2+padding))
    clearance = min(min(swept_limit(points[~beside], angle, config.wheelbase_m, horizon=2.5, margin=padding[~beside],
                                    length=config.length_m, width=config.width_m),
                        swept_limit(points[beside], angle, config.wheelbase_m, horizon=2.5, margin=fixed,
                                    length=config.length_m, width=config.width_m))
                    for angle in (steering, previous))
    result['beside_age_only_points'] = int(np.sum(beside))
"""),
]
STATE_PAIRS = [
    ('"""선택형 시험, 기본 미사용: observed_measured_v1의 평행 후보.',
     '"""오프라인 개입 후보(운영 미연결): observed_measured_v1 평행 후보의 시간/재개 처리 사본.'),
    ('from arena_autonomy.measured_parallel_speed_trial import proposal', 'from candidate_speed import proposal'),
]


def build(source, pairs, target):
    text = source.read_text(encoding='utf-8')
    for before, after in pairs:
        assert text.count(before) == 1, before[:60]
        text = text.replace(before, after)
    target.write_bytes(text.encode('utf-8'))
    return hashlib.sha256(text.encode('utf-8')).hexdigest()


def main():
    manifest = dict(
        scope='offline intervention candidate only; not connected to runtime or ROS',
        origin={p: hashlib.sha256((SRC/p).read_bytes()).hexdigest()
                for p in ('measured_parallel_speed_trial.py', 'measured_parallel_trial.py')},
        candidate={'candidate_speed.py': build(SRC/'measured_parallel_speed_trial.py', SPEED_PAIRS, BASE/'candidate_speed.py'),
                   'candidate_state.py': build(SRC/'measured_parallel_trial.py', STATE_PAIRS, BASE/'candidate_state.py')})
    (BASE/'candidate_manifest.json').write_bytes((json.dumps(manifest, ensure_ascii=False, indent=2)+'\n').encode())
    print(json.dumps(manifest))


if __name__ == '__main__':
    main()
