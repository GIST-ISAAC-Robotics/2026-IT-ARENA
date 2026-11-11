"""이미 회수한 PC/Jetson 출력의 차이만 집계한다. 판단을 다시 실행하지 않는다."""
from collections import Counter
import gzip
import hashlib
from itertools import zip_longest
import json
import math
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
paths=[ROOT/'build/r88_jetson_decisions.jsonl.gz', ROOT/'build/r89_jetson_decisions.jsonl.gz']
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before=[sha(p) for p in paths]
counts=Counter(); examples=[]; mismatches=[]; commands=[]; calls=0

def differences(a,b,path=''):
    if isinstance(a,(float,int)) and not isinstance(a,bool) and isinstance(b,(float,int)) and not isinstance(b,bool):
        if not math.isclose(a,b,rel_tol=1e-10,abs_tol=1e-10):yield path,a,b
    elif isinstance(a,dict) and isinstance(b,dict):
        if a.keys()!=b.keys():yield path+'.keys',sorted(a),sorted(b)
        for key in a.keys()&b.keys():yield from differences(a[key],b[key],path+'.'+key)
    elif isinstance(a,list) and isinstance(b,list):
        if len(a)!=len(b):yield path+'.length',len(a),len(b)
        else:
            for i,(x,y) in enumerate(zip(a,b)):yield from differences(x,y,path+f'[{i}]')
    elif type(a) is not type(b) or a!=b:yield path,a,b

with gzip.open(paths[0],'rt',encoding='utf8') as pc,gzip.open(paths[1],'rt',encoding='utf8') as jetson:
    for x,y in zip_longest(pc,jetson):
        assert x is not None and y is not None
        a,b=json.loads(x),json.loads(y)
        assert a['seq']==b['seq']
        calls+=1
        diff=list(differences(a['output'],b['output']))
        if diff:
            mismatches.append(a['seq'])
            counts.update(path for path,_,_ in diff)
            if len(examples)<8:examples.append(dict(seq=a['seq'],differences=diff[:30]))
        if list(differences(a['output'][:2],b['output'][:2])):commands.append(a['seq'])
report=dict(calls=calls,full_mismatch_count=len(mismatches),full_mismatch_seqs=mismatches,
    command_difference_seqs=commands,field_counts=counts,examples=examples,
    input_sha256=before,input_unchanged=before==[sha(p) for p in paths],
    scope='comparison only; original tolerance unchanged; not a new control replay')
(A/'implementation_comparison.json').write_text(json.dumps(report,indent=2)+'\n',encoding='utf8')
print(json.dumps({k:v for k,v in report.items() if k not in ('examples','full_mismatch_seqs')}))
