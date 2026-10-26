"""동결 실행 전후의 소스·시험·입력 SHA-256. 실행 중 대상 파일이 바뀌었는지 확인하는 용도다."""
import hashlib
import json
from pathlib import Path
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
SOURCES = sorted((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
TESTS = [ROOT/'tests'/name for name in (
    'test_m3_execution_trial.py', 'test_m3_observed_follow.py', 'test_observed_lateral_trial.py',
    'test_lateral_intent.py', 'test_lateral_sweep_review.py', 'test_observation_support.py',
    'test_local_follow.py', 'test_surface_contact_eval.py')]
TOOLS = [ROOT/'artifacts/validation/2026-10-24/m3_transition_v2'/name for name in (
    'replay_execution.py', 'analyze_run17.py', 'hash_manifest.py')] + [
    ROOT/'artifacts/validation/2026-10-24/m3_surface_lateral_v1/screen_lateral.py']
RECORDS = ['artifacts/validation/2026-10-24/e3_onroad_v1/g1', 'artifacts/validation/2026-10-24/e3_onroad_v1/r0plus',
           'artifacts/validation/2026-10-24/e3_stop_restart_v1/run', 'artifacts/validation/2026-10-16/g1_corridor_solo_v1/run']
INPUTS = [ROOT/r/'follow_decisions.jsonl' for r in RECORDS] + [
    ROOT/r/name for r in RECORDS[:3] for name in ('ego_poses.jsonl', 'actor_poses.jsonl', 'additional_actors.json')]


def digest(path):
    h = hashlib.sha256()
    with open(path, 'rb') as stream:
        for block in iter(lambda: stream.read(1 << 20), b''):
            h.update(block)
    return h.hexdigest()


def main():
    files = SOURCES+TESTS+TOOLS+INPUTS
    result = {str(p.relative_to(ROOT)).replace('\\', '/'): digest(p) for p in files}
    Path(sys.argv[1]).write_text(json.dumps(result, indent=1, sort_keys=True), encoding='utf-8', newline='\n')
    print(len(result), 'files hashed')


if __name__ == '__main__':
    main()
