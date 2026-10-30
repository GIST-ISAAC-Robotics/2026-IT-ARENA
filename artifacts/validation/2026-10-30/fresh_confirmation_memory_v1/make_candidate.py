"""오프라인 진단용 평행 후보 사본 생성: 런타임 proposal에 '팽창 기준 시각' 인자 하나만 더한다.

인자를 주지 않으면 런타임과 같은 계산이다. 인자를 주면 같은 raw 광선의 원래 취득 시각은 그대로 두고, 광선별 팽창 나이만
지정 시각(이전 fresh 확인 시각) 기준으로 계산한 반사실 결과를 낸다. 실제 명령 판단에는 쓰지 않고 거절 원인 분류에만 쓴다.
"""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
SOURCE = ROOT/'src/arena_autonomy/arena_autonomy/measured_parallel_speed_trial.py'
PAIRS = [
    ('"""선택형 시험, 기본 미사용: rear_lane_keep_policy=observed_measured_v1 전용 평행 경로 평가.',
     '"""오프라인 후보 진단 사본(운영 미연결): observed_measured_v1 평행 경로 평가 + 팽창 기준 시각 반사실 인자.'),
    ('import math\n', 'import math\nfrom types import SimpleNamespace\n'),
    ('def proposal(scene, baseline, speed, previous, dt, config, target_cap=.35):',
     'def proposal(scene, baseline, speed, previous, dt, config, target_cap=.35, padding_reference_s=None):'),
    ('    padding = per_ray_padding(scene, .025+config.measurement_padding_m, config.object_motion_bound_mps)\n',
     '    # 진단 반사실 전용: 같은 raw 광선·원래 취득 시각에서 팽창 나이만 지정 시각 기준으로 계산한다.\n'
     '    padding_scene = scene if padding_reference_s is None else SimpleNamespace(t_ref_s=padding_reference_s, rays=scene.rays)\n'
     '    padding = per_ray_padding(padding_scene, .025+config.measurement_padding_m, config.object_motion_bound_mps)\n'),
]


def main():
    text = SOURCE.read_text(encoding='utf-8')
    for before, after in PAIRS:
        assert text.count(before) == 1, before[:60]
        text = text.replace(before, after)
    (BASE/'candidate_speed.py').write_bytes(text.encode('utf-8'))
    manifest = dict(scope='offline diagnosis copy; not connected to runtime or ROS',
                    origin={SOURCE.relative_to(ROOT).as_posix(): hashlib.sha256(SOURCE.read_bytes()).hexdigest()},
                    candidate={'candidate_speed.py': hashlib.sha256(text.encode('utf-8')).hexdigest(),
                               'candidate_state.py': hashlib.sha256((BASE/'candidate_state.py').read_bytes()).hexdigest()})
    (BASE/'candidate_manifest.json').write_bytes((json.dumps(manifest, ensure_ascii=False, indent=2)+'\n').encode())
    print(json.dumps(manifest))


if __name__ == '__main__':
    main()
