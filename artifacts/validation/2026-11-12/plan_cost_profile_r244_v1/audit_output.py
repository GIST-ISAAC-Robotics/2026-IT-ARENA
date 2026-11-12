"""r244 전체 진단 차이의 위치를 기록한다. 허용 오차/기준 출력은 변경하지 않는다."""
from collections import Counter
import json
from profile_plan import A,ROOT,make_controller,replay_call,replay_reset,freeze,equal_output,comparable_decision


def differences(a,b,path=''):
    if equal_output(a,b):return
    if isinstance(a,dict) and isinstance(b,dict) and a.keys()==b.keys():
        for k in a:yield from differences(a[k],b[k],path+'/'+k)
    elif isinstance(a,list) and isinstance(b,list) and len(a)==len(b):
        for i,(x,y) in enumerate(zip(a,b)):yield from differences(x,y,path+'/'+str(i))
    else:yield path,a,b


def main():
    manifest=json.loads((A/'bundle_manifest.json').read_bytes())
    import hashlib
    sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    assert sha(ROOT/manifest['trace'])==manifest['trace_sha256']
    assert all(sha(ROOT/n)==r['sha256'] for n,r in manifest['files'].items())
    counts=Counter();examples={};commands=[];seqs=[];calls=0
    with (ROOT/manifest['trace']).open(encoding='utf8') as trace:
        follow,motion=make_controller(json.loads(next(trace))['data'])
        for line in trace:
            e=json.loads(line);d=e['data']
            if e['kind']=='reset':replay_reset(follow,d)
            elif e['kind']=='call':
                calls+=1;out=freeze(replay_call(follow,d['input'],motion))
                if not equal_output(out[:2],d['output'][:2]):commands.append(dict(seq=e['seq'],expected=d['output'][:2],actual=out[:2]))
                diff=list(differences(comparable_decision(d['output']),comparable_decision(out)))
                if diff:seqs.append(e['seq'])
                for path,expected,actual in diff:
                    counts[path]+=1
                    if path not in examples:examples[path]=dict(seq=e['seq'],expected=expected,actual=actual)
    result=dict(calls=calls,differing_sequences=seqs,command_differences=commands,
        field_counts=dict(counts),first_examples=examples,scope='same Jetson source versus historical PC reference; strict mismatch retained')
    (A/'result/output_audit.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf8')
    print(json.dumps(dict(calls=calls,differing_calls=len(seqs),command_differences=len(commands),fields=dict(counts))))
    return 0 if not commands else 1


if __name__=='__main__':raise SystemExit(main())
