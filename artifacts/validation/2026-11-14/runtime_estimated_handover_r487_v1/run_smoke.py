"""기존 r444 명시 estimated 계약 변환의 4대표 plan/current 각1회. 실물 입력 아님."""
from pathlib import Path
import copy,hashlib,json,sys,traceback
A=Path(__file__).resolve().parent
sys.path[:0]=[str(A/'src/arena_autonomy'),str(A/'src/arena_vehicle_interface')]
from contract_reference import estimated_header,estimated_record,request
from arena_autonomy.race_runtime_policy.evaluator import Evaluator
from arena_autonomy import current_lane_preference
from arena_autonomy.race_runtime_policy import current_route_priority
sha=lambda v:hashlib.sha256(json.dumps(v,sort_keys=True,allow_nan=False).encode()).hexdigest()
data=json.loads((A/'inputs.json').read_bytes());out=A/'results';out.mkdir()
rows=[];sources={current_lane_preference.__file__,current_route_priority.__file__}
for original in data['records']:
    key=f"{original['dataset']}:{original['trace_seq']}";original_before=sha(original)
    converted=estimated_record(original);header=estimated_header(data['headers'][original['dataset']])
    assert sha(original)==original_before
    for stage in ('plan','current'):
        worker=Evaluator(stage,header);inp=converted['plan_input'] if stage=='plan' else request(converted)
        before=sha(inp);lines={};error=None;value=None
        def trace(frame,event,arg):
            if frame.f_code.co_filename not in sources:return None
            if event=='line':lines.setdefault(frame.f_code.co_filename,set()).add(frame.f_lineno)
            return trace
        try:
            sys.settrace(trace)
            value=worker(inp)
        except BaseException as exc:error=dict(type=type(exc).__name__,message=str(exc))
        finally:sys.settrace(None);worker.close()
        snapshots=[converted['plan_input'],converted['current']]
        timing_ok=all(x.get('timing_verified') is False and x.get('timing_policy')=='host_estimated_v1'
            and x['scan']['measurements']['time_basis']=='unverified_estimate' for x in snapshots)
        if stage=='plan' and value is not None:
            normal=value[2].get('normal') or {}
            concise=dict(speed_mps=value[0],steering_rad=value[1],reason=normal.get('reason'),
                handoff_state=value[2].get('handoff_state'),selector_reason=normal.get('selector_reason'),
                observation_policy=(normal.get('partial_observation') or {}).get('execution_policy'))
        elif value is not None:
            concise={k:value.get(k) for k in ('speed_mps','steering_rad','reason','current_observation_policy',
                'independent_guard_required','runtime_authorized')}
        else:concise=None
        rows.append(dict(key=key,stage=stage,error=error,output=value,concise=concise,input_unchanged=sha(inp)==before,
            original_unchanged=sha(original)==original_before,estimated_timing_unchanged=timing_ok,
            executed_lines={str(Path(p).relative_to(A)):sorted(v) for p,v in lines.items()},
            input_sha256=before,physical_measurement=False))
        print(json.dumps({k:rows[-1][k] for k in ('key','stage','error','concise','input_unchanged','estimated_timing_unchanged')}),flush=True)
result=dict(completed=len(rows)==8,rows=rows,calls=len(rows),physical_measurement=False,
    synthetic_contract_copy=True,source='unchanged r447 selected simulation records with original r444 conversion',
    fresh_plan_state=True,current_uses_original_saved_plan_output=True,
    timing_benchmark=False,sensors_opened=False,ros_started=False,
    passed=len(rows)==8 and all(r['error'] is None and r['input_unchanged'] and r['original_unchanged'] and r['estimated_timing_unchanged'] for r in rows))
(out/'report.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
raise SystemExit(0 if result['passed'] else 1)
