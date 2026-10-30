"""검사 직전의 연결 관련 소스 바이트를 새 ZIP과 해시 JSON으로 남긴다(기존 파일 덮어쓰기 금지)."""
import hashlib
import json
from pathlib import Path
import sys
import zipfile

ROOT = Path(__file__).resolve().parents[4]
FILES = [
    'src/arena_autonomy/arena_autonomy/fresh_parallel_trial.py',
    'src/arena_autonomy/arena_autonomy/fresh_parallel_speed_trial.py',
    'config/tests/fresh_confirmation_r2_sha256.json',
    'tests/test_fresh_parallel_trial.py', 'tests/test_fresh_confirmation_provenance.py',
    'src/arena_autonomy/arena_autonomy/local_follow.py',
    'src/arena_autonomy/arena_autonomy/local_pursuit.py',
    'src/arena_autonomy/arena_autonomy/measured_lane_preference_trial.py',
    'src/arena_autonomy/arena_autonomy/measured_parallel_speed_trial.py',
    'src/arena_autonomy/arena_autonomy/measured_parallel_trial.py',
    'src/arena_autonomy/arena_autonomy/observed_lane_preference_trial.py',
    'src/arena_autonomy/arena_autonomy/observed_parallel_speed_trial.py',
    'src/arena_autonomy/arena_autonomy/parallel_lane_trial.py',
    'src/arena_autonomy/arena_autonomy/rear_lane_keep_trial.py',
    'scripts/prepare_jetson_bundle.py', 'scripts/profile_follow_trace.py',
    'scripts/validate_g1_traffic.py', 'scripts/validate_pipeline_faults_ros.py',
    'config/tests/measured_lane_preference_r1_sha256.json',
    'tests/test_measured_lane_preference_trial.py', 'tests/test_measured_lane_preference_provenance.py',
    'tests/test_observed_lane_preference_provenance.py', 'tests/source_contract_helpers.py',
    'tests/test_corner_follow.py', 'tests/test_g1_dense_stop.py', 'tests/test_physics_step_option.py',
    'tests/test_source_provenance_r0_plus_b3.py',
]


def main(target):
    target = Path(target)
    hashes = {}
    with zipfile.ZipFile(target, 'x', zipfile.ZIP_DEFLATED) as archive:
        for path in FILES:
            data = (ROOT/path).read_bytes()
            hashes[path] = hashlib.sha256(data).hexdigest()
            archive.writestr(path, data)
    with target.with_suffix('.json').open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(dict(files=hashes, zip_sha256=hashlib.sha256(target.read_bytes()).hexdigest()),
                  stream, ensure_ascii=False, indent=2)
        stream.write('\n')


if __name__ == '__main__':
    main(sys.argv[1])
