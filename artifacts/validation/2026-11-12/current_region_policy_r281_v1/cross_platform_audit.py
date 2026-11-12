"""진단 개수 차이를 숨기지 않고 PC/Jetson 출력·명령을 별도 대조."""
import collections,json,sys
from pathlib import Path
A=Path(__file__).resolve().parent;sys.path.insert(0,str(A.parents[3]/'src/arena_autonomy'))
from arena_autonomy.follow_trace import equal_output


def walk(a,b,path=''):
    if equal_output(a,b):return []
    if isinstance(a,dict) and isinstance(b,dict) and a.keys()==b.keys():
        return [v for k in a for v in walk(a[k],b[k],path+'.'+k)]
    if isinstance(a,list) and isinstance(b,list) and len(a)==len(b):
        return [v for i,(x,y) in enumerate(zip(a,b)) for v in walk(x,y,path+f'[{i}]')]
    return [dict(path=path,pc=a,jetson=b)]


operational=('speed_mps','steering_rad','reason','reference_s','original_reference_s','source_first_s','path_source_s','independent_guard_required','runtime_authorized')
left=json.loads((A/'pc_reuse/answers.json').read_bytes());right=json.loads((A/'jetson_reuse/answers.json').read_bytes())
assert len(left)==len(right)==518
differences=[];command=[]
for i,(a,b) in enumerate(zip(left,right)):
    if not equal_output(a,b):differences.append(dict(index=i,differences=walk(a,b)))
    if not equal_output({k:a.get(k) for k in operational},{k:b.get(k) for k in operational}):command.append(i)
original=json.loads((A/'jetson/original_full_differences.json').read_bytes())
original_paths=collections.Counter(v['path'] for x in original for v in walk(x['expected'],x['actual']))
result=dict(calls=len(left),full_equal=len(left)-len(differences),commands_equal=len(left)-len(command),command_difference_indices=command,
    differing_paths=dict(collections.Counter(v['path'] for x in differences for v in x['differences'])),differences=differences,
    original_differing_paths=dict(original_paths),scope='same 518 frozen inputs; numerical tolerance 1e-10 as existing comparator, integer diagnostics retained')
(A/'cross_platform_audit.json').write_text(json.dumps(result,indent=2)+'\n')
print(json.dumps({k:v for k,v in result.items() if k!='differences'}))
assert not command
