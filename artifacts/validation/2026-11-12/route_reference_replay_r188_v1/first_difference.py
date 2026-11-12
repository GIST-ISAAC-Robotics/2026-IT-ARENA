import json
from compare import A, TRACE, load, make_controller, replay_call, freeze, comparable_decision, equal_output

load('r188_difference_parent', A.parent/'guard_approach_cap_r181_v1/candidate.py').install()
candidate=load('r188_difference_route',A.parent/'expiry_route_reference_r186_v1/candidate_v2.py')
candidate.install()
events=[json.loads(line) for line in TRACE.open()]
owner,motion=make_controller(events[0]['data'])
for event in events[1:]:
    d=event['data']
    if event['kind']=='reset':
        if d.get('reason')=='decision_source_expired':candidate.reset_reference(owner)
        else:owner.reset()
    if event['kind']!='call':continue
    actual=freeze(replay_call(owner,d['input'],motion))
    if equal_output(comparable_decision(actual),comparable_decision(d['output'])):continue
    def differences(a,b,path=''):
        if isinstance(a,dict) and isinstance(b,dict):
            for key in sorted(set(a)|set(b)):
                if key not in a or key not in b:yield dict(path=path+'/'+key,replay=a.get(key),recorded=b.get(key))
                else:yield from differences(a[key],b[key],path+'/'+key)
        elif isinstance(a,list) and isinstance(b,list) and len(a)==len(b):
            for index,(x,y) in enumerate(zip(a,b)):yield from differences(x,y,path+'/'+str(index))
        elif not equal_output(a,b):yield dict(path=path,replay=a,recorded=b)
    result=dict(seq=event['seq'],t=d['input']['reference_s'],compute=d['compute'],
                differences=list(differences(comparable_decision(actual),comparable_decision(d['output']))))
    with (A/'first_difference.json').open('x') as out:json.dump(result,out,indent=2);out.write('\n')
    print(json.dumps(result))
    break
