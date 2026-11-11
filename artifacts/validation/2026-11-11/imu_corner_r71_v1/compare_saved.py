"""r71 변경 전/후 저장 입력 비교. 실행 source는 고정하고 결과를 덮어쓰지 않는다."""
import hashlib
import json
from pathlib import Path
import sys
import types
import zipfile

from arena_autonomy.local_follow import LocalFollow, FollowConfig
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.follow_trace import replay_call, freeze, comparable_decision, equal_output

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
TRACE = ROOT/'artifacts/validation/2026-11-11/imu_observed_sensitivity_v1/solo_run1/follow_decisions.jsonl'
OUT = HERE/'saved_compare.json'
if OUT.exists():
    raise SystemExit('preserve existing comparison')
source_path = 'src/arena_autonomy/arena_autonomy/local_follow.py'
with zipfile.ZipFile(HERE/'before_r71.zip') as z:
    old_source = z.read(source_path)
old_module = types.ModuleType('arena_autonomy.local_follow_before_r71')
sys.modules[old_module.__name__] = old_module
exec(compile(old_source, '<before-r71>', 'exec'), old_module.__dict__)
rows = []
baseline_mismatches = 0
trace_hash = hashlib.sha256(TRACE.read_bytes()).hexdigest()
source_hash = hashlib.sha256((ROOT/source_path).read_bytes()).hexdigest()
for line in TRACE.open(encoding='utf-8'):
    event = json.loads(line)
    data = event['data']
    if event['kind'] == 'header':
        motion = MotionConfig(**data['motion_config'])
        baseline = old_module.LocalFollow(old_module.FollowConfig(**data['follow_config']))
        candidate = LocalFollow(FollowConfig(**data['follow_config']))
    elif event['kind'] == 'reset':
        baseline.reset(); candidate.reset()
    elif event['kind'] == 'call':
        before, after = [replay_call(c, data['input'], motion) for c in (baseline, candidate)]
        baseline_mismatches += not equal_output(comparable_decision(freeze(before)), comparable_decision(data['output']))
        if not equal_output(comparable_decision(freeze(before)), comparable_decision(freeze(after))):
            rows.append(dict(seq=event['seq'], time_s=data['input']['reference_s'],
                before_speed=before[0], after_speed=after[0], before_reason=before[2]['reason'],
                after_reason=after[2]['reason'], arc=after[2].get('boundary_history_agreement')))
result = dict(scope='saved_input_counterfactual_not_closed_loop', baseline_mismatches=baseline_mismatches,
    changed_calls=len(rows), zero_to_positive=sum(r['before_speed']==0 and r['after_speed']>0 for r in rows),
    positive_to_zero=sum(r['before_speed']>0 and r['after_speed']==0 for r in rows),
    trace_sha256=trace_hash, source_sha256=source_hash,
    unchanged=trace_hash==hashlib.sha256(TRACE.read_bytes()).hexdigest() and source_hash==hashlib.sha256((ROOT/source_path).read_bytes()).hexdigest(),
    rows=rows)
OUT.write_text(json.dumps(result, ensure_ascii=False, indent=2)+'\n', encoding='utf-8')
print(json.dumps({k:v for k,v in result.items() if k!='rows'}, ensure_ascii=False))
