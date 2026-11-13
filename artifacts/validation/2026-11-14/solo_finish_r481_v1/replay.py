"""r477 frozen540 원 계산 순서 및 기록 current의 독립 baseline/후보 비교."""
from pathlib import Path
import argparse,copy,hashlib,json,math,sys,time

A=Path(__file__).resolve().parent
S=A/'stage'
TRACE=A.parent/'solo_variations_r477_v1/cases/start_left20_yaw_plus3/run1/follow_decisions.jsonl'
parser=argparse.ArgumentParser()
parser.add_argument('mode',choices=['baseline','candidate'])
parser.add_argument('result')
parser.add_argument('--baseline-result')
args=parser.parse_args()
OUT=A/args.result/args.mode
OUT.mkdir(parents=True,exist_ok=False)
sys.dont_write_bytecode=True
sys.path[:0]=[str(S/'src/arena_autonomy'),str(S/'src/arena_vehicle_interface')]
if args.mode=='baseline':
    # 원 2개 모듈만 동결 원본에서 읽고 나머지 229개는 동일 stage를 쓴다.
    import arena_autonomy
    arena_autonomy.__path__.insert(0,str(A/'originals/src/arena_autonomy/arena_autonomy'))
    import arena_autonomy.race_runtime_policy
    arena_autonomy.race_runtime_policy.__path__.insert(0,str(A/'originals/src/arena_autonomy/arena_autonomy/race_runtime_policy'))
from arena_autonomy.race_runtime_policy.evaluator import Evaluator,supported
from arena_autonomy.runtime_dual_state import current_request


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as f:
        for block in iter(lambda:f.read(1024*1024),b''):h.update(block)
    return h.hexdigest()


def write(name,value):
    (OUT/(name+'.json')).write_text(json.dumps(value,indent=2,allow_nan=False)+'\n',encoding='utf-8')


def differences(a,b,path='',limit=12):
    """원 수치 기준: float 1e-12, 정수/불리언/문자/키 exact. 필드 제외 없음."""
    found=[]
    def visit(x,y,p):
        if len(found)>=limit:return
        if type(x) is not type(y):found.append(dict(path=p,old=x,new=y));return
        if isinstance(x,dict):
            if x.keys()!=y.keys():found.append(dict(path=p,old_keys=list(x),new_keys=list(y)))
            for k in x.keys()&y.keys():visit(x[k],y[k],p+'.'+k)
        elif isinstance(x,list):
            if len(x)!=len(y):found.append(dict(path=p,old_length=len(x),new_length=len(y)))
            for i,(v,w) in enumerate(zip(x,y)):visit(v,w,p+f'[{i}]')
        elif isinstance(x,float):
            if not math.isclose(x,y,rel_tol=0,abs_tol=1e-12):found.append(dict(path=p,old=x,new=y))
        elif x!=y:found.append(dict(path=p,old=x,new=y))
    visit(a,b,path)
    return found


def child_output(output):
    result=copy.deepcopy(output)
    # 기록은 부모의 감속 cap 적용 뒤 값이다. child 원 계산과 부모 cap을
    # 구분하여 비교하며, 아래 원시 기록·cap은 버리지 않는다.
    cap=result.pop('new_plan_deceleration_cap',None)
    if cap is not None:
        assert cap['steering_changed'] is False
        assert cap['current_check_reused_without_timestamp_renewal'] is True
        assert result['speed_mps']==min(cap['checked_target_mps'],cap['speed_mps'])
        result['speed_mps']=cap['checked_target_mps']
    return result,cap


def apply_recorded_cap(output,cap):
    if cap is None:return output
    result=copy.deepcopy(output)
    result['speed_mps']=min(result['speed_mps'],cap['speed_mps'])
    return result


def main():
    seal=json.loads((A/'prepared.json').read_bytes())
    assert digest(TRACE)==seal['trace_sha256']
    for name,sha in seal['stage_files'].items():assert digest(S/name)==sha,name
    for name,sha in seal['original_files'].items():assert digest(A/'originals'/name)==sha,name
    import arena_autonomy.local_follow as local
    from arena_autonomy.race_runtime_policy import current_route_priority as policy
    from arena_autonomy import _numeric_kernels as native
    write('imports',dict(local_follow=local.__file__,policy=policy.__file__,native=native.__file__,native_sha256=digest(Path(native.__file__))))
    assert digest(Path(native.__file__))==seal['native_sha256']
    submitted=[];completed={};calls=[];header=None
    for line,text in enumerate(TRACE.open(encoding='utf-8'),1):
        row=json.loads(text);d=row['data'];kind=row['kind']
        if kind=='runtime_evaluator_header':header=d['header']
        elif kind=='runtime_plan_submitted':submitted.append(d)
        elif kind=='runtime_plan_completed':
            key=(d['generation'],d['sequence']);assert key not in completed;completed[key]=d
        elif kind=='call' and d.get('current_rebase'):calls.append(dict(line=line,seq=row['seq'],data=d['current_rebase']))
    assert len(submitted)==len(completed)==540
    assert header==json.loads((A/'header.json').read_bytes())
    assert header['numeric_execution']=='native_v1'
    candidate=args.mode=='candidate'
    if candidate:
        baseline_dir=A/(args.baseline_result or args.result)/'baseline'
        baseline=json.loads((baseline_dir/'summary.json').read_bytes())
        assert baseline['baseline_verified'] is True
        assert baseline['native_sha256']==seal['native_sha256']
    e=Evaluator('plan',header);generation=None;outputs={};rows=[]
    try:
        with (OUT/'plan_outputs.jsonl').open('x',encoding='utf-8') as stream:
            for item in submitted:
                key=item['generation'],item['sequence']
                assert key not in outputs
                if generation!=key[0]:e.reset(item['reset_reason']);generation=key[0]
                encoded=json.dumps(item['input'],sort_keys=True)
                start=time.perf_counter();actual=e(item['input']);ms=(time.perf_counter()-start)*1000
                assert encoded==json.dumps(item['input'],sort_keys=True)
                expected=completed[key]['output'];diff=differences(expected,actual)
                old=expected[2].get('normal') or {};new=actual[2].get('normal') or {}
                info=dict(key=key,reference_s=item['input']['reference_s'],ms=ms,exact=actual==expected,numeric_match=not diff,
                    old_command=expected[:2],command=actual[:2],old_reason=old.get('reason'),reason=new.get('reason'),
                    old_selector_reason=old.get('selector_reason'),selector_reason=new.get('selector_reason'),
                    old_mode=old.get('selector_mode'),mode=new.get('selector_mode'),
                    sides_same=old.get('selector_sides')==new.get('selector_sides'),
                    handoff_same=expected[2].get('handoff_state')==actual[2].get('handoff_state'),
                    path_same=old.get('path_points')==new.get('path_points'),
                    selected_wall_same=old.get('path_wall')==new.get('path_wall'),
                    clearance_same=old.get('swept_clearance_m')==new.get('swept_clearance_m'),
                    old_partial_reason=(old.get('partial_observation') or {}).get('reason'),
                    partial_reason=(new.get('partial_observation') or {}).get('reason'))
                rows.append(info);outputs[key]=actual
                stream.write(json.dumps(dict(key=key,output=actual),allow_nan=False)+'\n')
                if diff and not candidate:
                    write('baseline_failure',dict(row=info,differences=diff,recorded=expected,actual=actual))
                    raise AssertionError(f'baseline plan mismatch {key}')
    finally:e.close()
    write('plan_rows',rows)
    # 같은 원 계산의 재전달은 한 번 계산한다. 중복 input/계산시각/원 child
    # 결과가 하나라도 달라지면 중단하며 임의 상태/시각을 채우지 않는다.
    e=Evaluator('current',header);cache={};current_rows=[]
    try:
        with (OUT/'current_outputs.jsonl').open('x',encoding='utf-8') as stream:
            for call in calls:
                c=call['data'];meta=c['compute'];oldplan=meta['plan'];key=oldplan['generation'],oldplan['sequence']
                expected,cap=child_output(c['output'])
                assert oldplan['output']==completed[key]['output']
                original_sig=json.dumps(dict(input=meta['input'],plan=oldplan,expected=expected,
                    began=meta['began_wall_s'],completed=meta['completed_wall_s']),sort_keys=True)
                ck=meta['generation'],meta['sequence']
                plan=outputs[key]
                if ck in cache:
                    saved_sig,actual,ms=cache[ck];assert original_sig==saved_sig,('current duplicate differs',ck)
                    reused=True
                else:
                    assert supported(plan),(key,'current plan unsupported')
                    request=current_request(plan,oldplan['input'],meta['input'],retained=meta['route_check']['retained'],crop_current_selection=True)
                    encoded=json.dumps(request,sort_keys=True)
                    start=time.perf_counter();actual=e(request);ms=(time.perf_counter()-start)*1000
                    assert encoded==json.dumps(request,sort_keys=True)
                    cache[ck]=(original_sig,actual,ms);reused=False
                    stream.write(json.dumps(dict(key=ck,plan_key=key,output=actual),allow_nan=False)+'\n')
                diff=differences(expected,actual)
                delivered=apply_recorded_cap(actual,cap)
                info=dict(line=call['line'],trace_seq=call['seq'],key=ck,plan_key=key,reused=reused,
                    reference_s=meta['input']['reference_s'],ms=ms,exact=actual==expected,numeric_match=not diff,
                    old_speed=c['output']['speed_mps'],speed=delivered['speed_mps'],
                    old_reason=expected['reason'],reason=actual['reason'],recorded_parent_cap=cap,
                    path_same=expected.get('path_points')==actual.get('path_points'),
                    clearance_same=expected.get('clearance_m')==actual.get('clearance_m'))
                current_rows.append(info)
                if diff and not candidate:
                    write('baseline_failure',dict(row=info,differences=diff,recorded_raw=c['output'],recorded_child=expected,actual=actual))
                    raise AssertionError(f'baseline current mismatch {ck}')
    finally:e.close()
    write('current_rows',current_rows)
    # 정체 입력은 실제 후속 current가 존재하지 않는다. 최초/중간/마지막을
    # 같은 센서 시점으로 진단하여 연결만 확인하고 실제 발행/진행과 구분한다.
    examples=[]
    if candidate:
        restored=[item for item in submitted if completed[(item['generation'],item['sequence'])]['output'][0]==0
                  and outputs[(item['generation'],item['sequence'])][0]>0]
        if restored:
            e=Evaluator('current',header)
            try:
                for item in (restored[0],restored[len(restored)//2],restored[-1]):
                    key=item['generation'],item['sequence'];plan=outputs[key]
                    if not supported(plan):
                        examples.append(dict(key=key,supported=False));continue
                    request=current_request(plan,item['input'],item['input'],retained=False,crop_current_selection=True)
                    examples.append(dict(key=key,reference_s=item['input']['reference_s'],plan_speed=plan[0],
                        output=e(request),scope='same original plan input; no actual later current/publication'))
            finally:e.close()
    write('recovered_same_input_current',examples)
    summary=dict(mode=args.mode,plans=len(rows),plan_exact=sum(r['exact'] for r in rows),
        plan_numeric_match=sum(r['numeric_match'] for r in rows),
        zero_to_positive=sum(r['old_command'][0]==0<r['command'][0] for r in rows),
        positive_to_zero=sum(r['old_command'][0]>0 and r['command'][0]<=0 for r in rows),
        positive_increased=sum(0<r['old_command'][0]<r['command'][0] for r in rows),
        positive_decreased=sum(0<r['command'][0]<r['old_command'][0] for r in rows),
        sides_changed=sum(not r['sides_same'] for r in rows),handoff_changed=sum(not r['handoff_same'] for r in rows),
        path_changed=sum(not r['path_same'] for r in rows),selected_wall_changed=sum(not r['selected_wall_same'] for r in rows),
        clearance_changed=sum(not r['clearance_same'] for r in rows),
        current_records=len(current_rows),current_calculations=len(cache),current_reused=len(current_rows)-len(cache),
        current_exact=sum(r['exact'] for r in current_rows),current_numeric_match=sum(r['numeric_match'] for r in current_rows),
        current_positive_to_zero=sum(r['old_speed']>0 and r['speed']<=0 for r in current_rows),
        current_zero_to_positive=sum(r['old_speed']==0<r['speed'] for r in current_rows),
        baseline_verified=(not candidate and all(r['numeric_match'] for r in rows+current_rows)),
        source_unchanged=all(digest(S/p)==h for p,h in seal['stage_files'].items()),
        originals_unchanged=all(digest(A/'originals'/p)==h for p,h in seal['original_files'].items()),
        trace_unchanged=digest(TRACE)==seal['trace_sha256'],
        input_epoch_unchanged=True,native_sha256=digest(Path(native.__file__)),
        scope='saved input evaluation; no new drive, pose feedback, child delivery schedule or hardware proof')
    write('summary',summary)
    write('focus',dict(plans=[r for r in rows if r['reference_s']>=49],
                       currents=[r for r in current_rows if r['reference_s']>=49]))
    print(json.dumps(summary),flush=True)
    assert summary['source_unchanged'] and summary['trace_unchanged'] and summary['originals_unchanged']


if __name__=='__main__':main()
