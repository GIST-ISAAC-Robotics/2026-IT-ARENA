"""기존 재생 실패와 최종 정지의 진단만 보충한다. 원 판정/소스는 변경하지 않는다."""
import copy
import hashlib
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
ROOT = BASE.parents[3]
sys.path.insert(0, str(ROOT/'src/arena_autonomy'))
sys.path.insert(0, str(ROOT/'src/arena_vehicle_interface'))
from arena_autonomy.follow_trace import equal_output

replay_path = BASE/'run1_course/follow_trace_replay.json'
before = hashlib.sha256(replay_path.read_bytes()).hexdigest()
report = json.loads(replay_path.read_text())
details = []
for mismatch in report['mismatches']:
    expected, actual = copy.deepcopy(mismatch['expected']), copy.deepcopy(mismatch['actual'])
    before_fit = expected[2]['partial_observation']['anchor_diagnostic']['fit']
    after_fit = actual[2]['partial_observation']['anchor_diagnostic']['fit']
    first, second = before_fit.pop('processing_ms'), after_fit.pop('processing_ms')
    details.append(dict(seq=mismatch['seq'], expected_ms=first, actual_ms=second,
        only_difference_is_anchor_fit_processing_ms=equal_output(expected, actual),
        command_identical=equal_output(expected[:2], actual[:2]),
        reason_identical=expected[2].get('reason') == actual[2].get('reason')))

late = []
targets = [129.6, 130.02, 131.3, 131.52, 132., 133., 154.]
with (BASE/'run1_course/follow_decisions.jsonl').open() as stream:
    for line in stream:
        row = json.loads(line)
        if row['kind'] != 'call' or not targets:
            continue
        data = row['data']
        stamp = data['input']['reference_s']
        if stamp < targets[0]-1e-8:
            continue
        requested = targets.pop(0)
        meta = data['output'][2]
        partial = meta.get('partial_observation') or {}
        anchor = partial.get('anchor_diagnostic') or {}
        late.append(dict(requested_s=requested, reference_s=stamp,
            command=data['output'][:2], reason=meta.get('reason'), baseline_reason=meta.get('baseline_reason'),
            selector_reason=meta.get('selector_reason'),
            partial_reason=partial.get('reason'), latch_reason=partial.get('latch_reason'),
            latch_time_s=partial.get('latch_time_s'),
            anchor_reason=anchor.get('reason'), anchor_released=anchor.get('released'),
            corridor=meta.get('corridor_candidate'),
            restored_history=meta.get('restored_passage_history')))

only_timing = (report['matched'] + len(details) == report['calls'] and
               all(d['only_difference_is_anchor_fit_processing_ms'] for d in details))
result = dict(scope='post-run mismatch explanation; original replay remains failed',
    original_passed=report['passed'], original_calls=report['calls'], original_matched=report['matched'],
    explained_mismatches=details, all_remaining_fields_equal=only_timing,
    replay_sha256=before, replay_unchanged=hashlib.sha256(replay_path.read_bytes()).hexdigest()==before,
    late_samples=late,
    next_step='First add narrowly scoped timing-field normalization and tests; then inspect the new latch in saved inputs. No runtime changes or new driving in this audit.')
with (BASE/'closeout_audit.json').open('x', encoding='utf-8') as stream:
    json.dump(result, stream, ensure_ascii=False, indent=2, allow_nan=False)
print(json.dumps(result, ensure_ascii=False))
