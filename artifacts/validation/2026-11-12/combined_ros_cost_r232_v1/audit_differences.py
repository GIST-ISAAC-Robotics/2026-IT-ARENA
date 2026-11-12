from pathlib import Path
from collections import Counter
import hashlib,json,math
A=Path(__file__).resolve().parent
source=A/'jetson_full/differences.json'
changes=[]
def compare(a,b,path,seq,stage):
    if isinstance(a,(int,float)) and not isinstance(a,bool) and isinstance(b,(int,float)) and not isinstance(b,bool):
        same=math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-10)
    elif isinstance(a,dict) and isinstance(b,dict):
        if a.keys()!=b.keys():changes.append(dict(seq=seq,stage=stage,path=path+'.keys',actual=sorted(a),expected=sorted(b)))
        for k in a.keys()&b.keys():compare(a[k],b[k],path+'.'+k,seq,stage)
        return
    elif isinstance(a,list) and isinstance(b,list):
        if len(a)!=len(b):changes.append(dict(seq=seq,stage=stage,path=path+'.length',actual=len(a),expected=len(b)))
        for i,(x,y) in enumerate(zip(a,b)):compare(x,y,path+f'[{i}]',seq,stage)
        return
    else:same=type(a) is type(b) and a==b
    if not same:changes.append(dict(seq=seq,stage=stage,path=path,actual=a,expected=b))
rows=json.loads(source.read_bytes())
for row in rows:compare(row['actual'],row['expected'],'root',row['seq'],row['stage'])
counts=Counter((r['stage'],r['path']) for r in changes)
report=dict(original_difference_cases=len(rows),leaf_differences=len(changes),
    paths=[dict(stage=k[0],path=k[1],count=v) for k,v in counts.most_common()],rows=changes,
    original_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
    scope='existing 1e-10 replay comparison, no tolerance or control change')
with (A/'difference_audit.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in report.items() if k!='rows'},ensure_ascii=False))
