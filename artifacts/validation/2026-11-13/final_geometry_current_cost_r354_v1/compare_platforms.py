"""원 비교 실패를 보존하고 어느 출력 항목이 달랐는지만 따로 기록한다."""
from pathlib import Path
import hashlib,json,math

A=Path(__file__).resolve().parent

def differences(x,y,path=''):
    if isinstance(x,dict) and isinstance(y,dict):
        if x.keys()!=y.keys():return [dict(path=path,kind='keys')]
        return [v for k in x for v in differences(x[k],y[k],path+'.'+k)]
    if isinstance(x,list) and isinstance(y,list):
        if len(x)!=len(y):return [dict(path=path,kind='length')]
        return [v for i,(a,b) in enumerate(zip(x,y)) for v in differences(a,b,f'{path}[{i}]')]
    if isinstance(x,(int,float)) and isinstance(y,(int,float)) and not isinstance(x,bool) and not isinstance(y,bool):
        if math.isclose(x,y,rel_tol=1e-10,abs_tol=1e-10):return []
    elif type(x) is type(y) and x==y:return []
    return [dict(path=path,pc=x,jetson=y)]

def main():
    inputs=['pc/answers.json','jetson_initial/jetson/answers.json']
    a,b=[json.loads((A/p).read_bytes()) for p in inputs]
    rows=[dict(selected_index=i,differences=differences(x,y)) for i,(x,y) in enumerate(zip(a,b)) if differences(x,y)]
    allowed={'.current_route_proof.region.queries','.current_route_proof.region.masked_left',
             '.current_route_proof.region.masked_right','.region.queries','.region.masked_right'}
    result=dict(calls=len(a),full_equal=len(a)-len(rows),differences=rows,
        diagnostic_counts_only=all(d['path'] in allowed for r in rows for d in r['differences']),
        comparison_tolerance=1e-10,original_failure_preserved=True,
        inputs_sha256={p:hashlib.sha256((A/p).read_bytes()).hexdigest() for p in inputs})
    (A/'platform_comparison.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k not in ('differences','inputs_sha256')}))
    assert len(a)==len(b)==26 and len(rows)==7 and result['diagnostic_counts_only']

if __name__=='__main__':main()
