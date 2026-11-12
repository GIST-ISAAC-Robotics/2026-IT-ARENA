"""완료한 ON 산출물의 표면 정합 벽시계 차이를 원 비교 규칙으로 분리한다."""
from pathlib import Path
import gzip,json
from arena_autonomy.follow_trace import equal_output,comparable_decision
A=Path(__file__).resolve().parent
reports=[]
for tag in ('r220','r217'):
    original=json.loads((A.parent/f'traffic_solo_gap_r223_v1/candidate_{tag}.json').read_text())
    rows={r['seq']:r for r in original['rows']}
    changes={r['seq']:r for r in original['summary']['changes']}
    outputs=json.load(gzip.open(A/f'on_{tag}_outputs.json.gz','rt'))
    raw_errors=[];errors=[]
    def partial(value):return comparable_decision([0.,0.,dict(partial_observation=value)])[2]['partial_observation']
    for seq,answer in outputs:
        row=rows[seq];normal=answer[2].get('normal')or{}
        observed=normal.get('partial_observation')
        if not equal_output(observed,row['partial']):raw_errors.append(seq)
        matches=(equal_output(answer[0],row['speed']) and normal.get('reason')==row['reason']
                 and normal.get('selector_mode')==row['mode'] and equal_output(partial(observed),partial(row['partial'])))
        if seq in changes:matches=matches and equal_output(answer[:2],changes[seq]['after'])
        if not matches:errors.append(seq)
    reports.append(dict(trace=tag,calls=len(outputs),raw_observation_differences=raw_errors,
        normalized_observation_or_command_mismatches=errors,
        scope='only existing surface_correspondence.processing_ms normalization; no new tolerance'))
with (A/'on_audit.json').open('x') as out:json.dump(reports,out,indent=2);out.write('\n')
print(json.dumps(reports));assert not any(r['normalized_observation_or_command_mismatches'] for r in reports)
