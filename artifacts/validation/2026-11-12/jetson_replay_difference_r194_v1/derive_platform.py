"""원 비교 실패를 보존하고 고정 Jetson 결과의 재현/CPU 비용용 입력을 별도로 만든다."""
import hashlib
import json
from pathlib import Path
from arena_autonomy.follow_trace import equal_output

BASE=Path(__file__).resolve().parent
WORK=BASE/'worktree'
source=WORK/'artifacts/validation/2026-11-12/selected_profile_r192_v1/input/follow_decisions.jsonl'
summary=json.loads((BASE/'difference_r194/summary.json').read_text())
assert summary['input_sha256']==hashlib.sha256(source.read_bytes()).hexdigest()
assert summary['input_unchanged'] and not summary['command_changes']
assert summary['calls']==439 and summary['mismatch_calls']==82
allowed={
 'output[2].normal.selector_guard_candidate.region.queries',
 'output[2].normal.selector_guard_candidate.region.masked_right',
 'output[2].normal.selector_guard_candidate.region.masked_left',
 'output[2].normal.selector_guard_candidate.current_approach.sides.left.samples',
 'output[2].normal.selector_region.queries',
 'output[2].normal.selector_region.masked_left',
 'output[2].normal.selector_region.masked_right',
 'output[2].normal.selector_region.unobserved',
 'output[2].normal.selector_rear_fringe.queries',
 'output[2].normal.same_path_gap.queries',
 'output[2].entry_space.candidates[0].motion_region.queries',
 'output[2].entry_space.candidates[0].motion_region.masked_left'}
assert set(summary['leaf_counts'])==allowed
assert all(type(leaf['expected']) is int and type(leaf['actual']) is int
    for row in summary['details'] for leaf in row['differences'])
actual={row['seq']:row for row in map(json.loads,(BASE/'difference_r194/mismatches.jsonl').open())}
events=[json.loads(line) for line in source.open()]
output=WORK/'artifacts/validation/2026-11-12/jetson_platform_r194/input.jsonl'
output.parent.mkdir(parents=True,exist_ok=False)
inputs=hashlib.sha256();replaced=0
with output.open('x') as stream:
    for event in events:
        if event['kind']=='call':
            inputs.update(json.dumps(event['data']['input'],sort_keys=True,separators=(',',':')).encode())
        if event['seq'] in actual:
            row=actual[event['seq']]
            assert event['data']['output']==row['expected']
            assert equal_output(row['expected'][:2],row['actual'][:2])
            event['data']['output']=row['actual'];replaced+=1
        stream.write(json.dumps(event,allow_nan=False)+'\n')
check=hashlib.sha256()
for line in output.open():
    event=json.loads(line)
    if event['kind']=='call':
        check.update(json.dumps(event['data']['input'],sort_keys=True,separators=(',',':')).encode())
assert inputs.digest()==check.digest() and replaced==82
record=dict(original=source.relative_to(WORK).as_posix(),original_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    derived=output.relative_to(WORK).as_posix(),derived_sha256=hashlib.sha256(output.read_bytes()).hexdigest(),
    input_digest=inputs.hexdigest(),inputs_equal=True,commands_equal=True,replaced_diagnostics=82,
    original_platform_replay_passed=False,
    scope='Jetson_same_source_fixed_input_repeatability_and_cpu_cost_only_not_cross_platform_pass')
(BASE/'platform_reference_r194.json').write_text(json.dumps(record,indent=2)+'\n')
print(json.dumps(record))
