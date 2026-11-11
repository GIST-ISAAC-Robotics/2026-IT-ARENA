"""Compare the two Jetson streams using the unchanged follow_trace tolerance."""
import ast
import gzip
import hashlib
import itertools
import json
import math
from numbers import Real
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
A=Path(__file__).resolve().parent
source=(ROOT/'src/arena_autonomy/arena_autonomy/follow_trace.py').read_text(encoding='utf8')
node=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='equal_output')
exec(compile(ast.Module(body=[node],type_ignores=[]),'equal_output_from_current_source','exec'))
paths=[ROOT/'build'/f'r85_stream_{name}.jsonl.gz' for name in ('before','after')]
hashes=[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths]
summaries=[json.loads((A/f'jetson_stream_{name}.json').read_bytes()) for name in ('before','after')]
assert all(h==d['decisions_sha256'] for h,d in zip(hashes,summaries))
count=0;differences=[]
with gzip.open(paths[0],'rt',encoding='utf8') as left,gzip.open(paths[1],'rt',encoding='utf8') as right:
    for a,b in itertools.zip_longest(left,right):
        assert a is not None and b is not None
        a,b=json.loads(a),json.loads(b);assert a['seq']==b['seq'];count+=1
        if not equal_output(a['output'],b['output']):differences.append(a['seq'])
assert count==3072
result=dict(scope='pairwise complete comparable Jetson outputs, same input and machine',
            calls=count,matched=count-len(differences),difference_sequences=differences,
            tolerance=1e-10,inputs_sha256=hashes,
            inputs_unchanged=hashes==[hashlib.sha256(p.read_bytes()).hexdigest() for p in paths])
out=A/'jetson_stream_pairwise.json';assert not out.exists()
out.write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
print(json.dumps(result))
