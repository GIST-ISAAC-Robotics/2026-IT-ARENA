"""원 결과를 보존하며 r464 405행 차이를 Windows에서 집계한다."""
from pathlib import Path
import json

HERE=Path(__file__).resolve().parent
ROOT=HERE/'execution_v1'
base=json.loads((ROOT/'baseline_result/summary.json').read_text())
new=json.loads((ROOT/'candidate_result/summary.json').read_text())
checks=json.loads((ROOT/'candidate_result/checks.json').read_text())
def group(rows):
    return {'count':len(rows),'exact':sum(r['numeric_equal'] for r in rows),
        'command_equal':sum(r['command_equal'] for r in rows),
        'zero_to_positive':[r['sequence'] for r in rows if r['original_command'][0]==0 and r['actual_command'][0]>0],
        'positive_to_zero':[r['sequence'] for r in rows if r['original_command'][0]>0 and r['actual_command'][0]==0],
        'path_changes':[r['sequence'] for r in rows if not r['path_equal']],
        'wall_changes':[r['sequence'] for r in rows if not r['path_wall_equal']],
        'side_changes':[r['sequence'] for r in rows if not r['selector_sides_equal']],
        'selector_mode_changes':[r['sequence'] for r in rows if not r['selector_mode_equal']],
        'handoff_state_changes':[r['sequence'] for r in rows if not r['handoff_state_equal']]}
result={'baseline':{k:v for k,v in base.items() if k!='modules'},
        'candidate':{k:v for k,v in new.items() if k!='modules'},
        'groups':{'all':group(checks),'before641':group([r for r in checks if r['sequence']<641]),
                  '641_through645':group([r for r in checks if 641<=r['sequence']<=645]),
                  'after645':group([r for r in checks if r['sequence']>645])},
        'focus':[r for r in checks if r['sequence'] in (641,643,644,645)],
        'first_changed':next((r for r in checks if not r['numeric_equal']),None),
        'notes':['saved measured speed/history remain from the original drive; no counterfactual vehicle motion',
                 'current recheck and independent guard were not rerun; positive plan is not publication proof']}
(ROOT/'comparison_summary.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps({'baseline_exact':base['numeric_mismatches']==0,'groups':result['groups'],
    'focus':[dict(sequence=r['sequence'],old=r['original_command'],new=r['actual_command'],
                  old_reason=r['original_reason'],new_reason=r['actual_reason'],agreement=r['actual_agreement'])
             for r in result['focus']]},indent=2))
