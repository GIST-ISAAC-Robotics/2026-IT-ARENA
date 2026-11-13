"""검사 전용 경로·동결 확인. 실행 프로필을 등록하거나 장치를 열지 않는다."""
import hashlib
import json
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
PREPARED = HERE / 'prepared_v2'
R399 = HERE.parent / 'runtime_profile_r399_v1/generated_v1'
sys.path[:0] = [str(ROOT / 'src/arena_autonomy'), str(ROOT / 'src/arena_vehicle_interface')]


def load_json(path):
    return json.loads(path.read_bytes())


def profile():
    import yaml
    return yaml.safe_load((PREPARED / 'runtime_profile.yaml').read_bytes())


def verify_manifest():
    manifest = load_json(HERE / 'manifest.json')
    actual = {name: hashlib.sha256((ROOT / name).read_bytes()).hexdigest() for name in manifest['files']}
    assert actual == manifest['files'], 'r401 frozen source/input/profile differs'
    return actual
