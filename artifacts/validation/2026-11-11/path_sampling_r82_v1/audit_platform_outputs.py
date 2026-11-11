"""고정 소스/입력의 전체 플랫폼 출력 차이를 분류한다. 비용/ROS 마감 검증은 아니다."""
import argparse,collections,hashlib,json,platform,time
from pathlib import Path
import numpy as np
from arena_autonomy import follow_trace as ft

parser=argparse.ArgumentParser();parser.add_argument('trace',type=Path);parser.add_argument('out',type=Path)
parser.add_argument('--manifest',type=Path,required=True)
a=parser.parse_args();assert not a.out.exists()
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
manifest=json.loads(a.manifest.read_bytes())
root=Path(ft.__file__).resolve().parents[3]
assert all(sha(root/name)==row['sha256'] for name,row in manifest['files'].items())
before=sha(a.trace);current=None;calls=0;changes=[];counts=collections.Counter();examples={};began=time.monotonic()
original=ft.replay_call

def leaves(actual,expected,path='$'):
    if ft.equal_output(actual,expected):return
    if isinstance(actual,dict) and isinstance(expected,dict):
        for key in sorted(actual.keys()|expected.keys()):
            if key not in actual or key not in expected:
                yield path+'.'+key,actual.get(key,'<missing>'),expected.get(key,'<missing>')
            else:yield from leaves(actual[key],expected[key],path+'.'+key)
    elif isinstance(actual,list) and isinstance(expected,list) and len(actual)==len(expected):
        for i,(x,y) in enumerate(zip(actual,expected)):yield from leaves(x,y,path+f'[{i}]')
    else:yield path,actual,expected

def capture(*args,**kwargs):
    global calls
    value=original(*args,**kwargs);calls+=1
    actual=ft.comparable_decision(ft.freeze(value));expected=ft.comparable_decision(current['data']['output'])
    for path,x,y in leaves(actual,expected):
        counts[path]+=1
        if path not in examples:examples[path]=dict(seq=current['seq'],actual=x,expected=y)
    if not ft.equal_output(actual[:2],expected[:2]):
        changes.append(dict(seq=current['seq'],time_s=current['data']['input']['reference_s'],expected=expected[:2],actual=actual[:2]))
    return value

def events():
    global current
    for line in a.trace.open(encoding='utf-8'):
        current=json.loads(line);yield current

ft.replay_call=capture
try:result=ft.replay_events(events())
finally:ft.replay_call=original
report=dict(scope='full frozen r82 offline output audit; no timing costs or new driving',calls=calls,
    all_outputs_matched=result['passed'],full_matched=result['matched'],commands_matched=not changes,
    command_differences=changes,leaf_difference_counts=dict(counts),first_example_by_path=examples,
    input_sha256=before,input_unchanged=before==sha(a.trace),elapsed_s=time.monotonic()-began,
    source_unchanged=all(sha(root/name)==row['sha256'] for name,row in manifest['files'].items()),
    python=platform.python_version(),numpy=np.__version__,machine=platform.machine(),replay_module=str(Path(ft.__file__).resolve()))
a.out.write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
print(json.dumps({k:report[k] for k in ('calls','all_outputs_matched','full_matched','commands_matched','leaf_difference_counts','input_unchanged','source_unchanged','elapsed_s')}))
