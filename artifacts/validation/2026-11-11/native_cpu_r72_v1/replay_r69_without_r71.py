"""최신 CPU 구현과 r69 정책을 분리해 재생한다. 파일/운영 설정을 되돌리지 않는다."""
import hashlib
import json
from pathlib import Path
import sys
import types
import zipfile

import arena_autonomy.follow_trace as trace

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
INPUT = HERE.parent/'forward_metric_r69_v1/solo_run1/follow_decisions.jsonl'
OUTPUT = HERE/'r69_cpu_only_replay.json'
if OUTPUT.exists():
    raise SystemExit('preserve previous comparison')
paths = [ROOT/'src/arena_autonomy/arena_autonomy'/name for name in (
    'local_follow.py', 'native_geometry.py', 'scene_motion.py', 'scan_input.py')]
sha = lambda p: hashlib.sha256(p.read_bytes()).hexdigest()
before = {p.relative_to(ROOT).as_posix(): sha(p) for p in paths}
input_hash = sha(INPUT)
with zipfile.ZipFile(HERE.parent/'imu_corner_r71_v1/before_r71.zip') as z:
    old_source = z.read('src/arena_autonomy/arena_autonomy/local_follow.py')
module = types.ModuleType('arena_autonomy.local_follow_before_r71')
sys.modules[module.__name__] = module
exec(compile(old_source, '<before-r71>', 'exec'), module.__dict__)
old_follow, old_config = trace.LocalFollow, trace.FollowConfig
trace.LocalFollow, trace.FollowConfig = module.LocalFollow, module.FollowConfig
try:
    with INPUT.open(encoding='utf-8') as stream:
        result = trace.replay_events(json.loads(line) for line in stream)
finally:
    trace.LocalFollow, trace.FollowConfig = old_follow, old_config
result.pop('outcomes', None)
result.update(scope='r69_policy_restored_only_in_process_with_r72_native_cpu',
    policy_sha256=hashlib.sha256(old_source).hexdigest(), input_sha256=input_hash,
    input_unchanged=sha(INPUT)==input_hash, source_sha256=before,
    source_unchanged=all(sha(ROOT/name)==digest for name,digest in before.items()))
OUTPUT.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k not in ('mismatches', 'source_sha256')}, ensure_ascii=False))
