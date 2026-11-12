"""새 공통 만료 참조를 일반 기록 재생기가 해석하는지 저장 입력으로 비교."""
from pathlib import Path
import argparse,ast,hashlib,json
import arena_autonomy.follow_trace as runtime
A=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('mode',choices=['baseline','candidate']);args=parser.parse_args()
trace=A.parent/'traffic_solo_gap_runtime_r225_v1/two_actor_run1/run1/follow_decisions.jsonl'
path=Path(runtime.__file__)
source=path.read_text()
if args.mode=='candidate':
    old="elif kind in ('decision_submission_policy', 'current_path_recheck_policy', 'delivery_delay_profile'):"
    new="elif kind in ('decision_submission_policy', 'current_path_recheck_policy', 'delivery_delay_profile', 'expiry_reference_policy'):"
    assert source.count(old)==1;source=source.replace(old,new)
    old="""        elif kind == 'reset':
            follow.reset()"""
    new="""        elif kind == 'reset':
            if (follow.config.expiry_route_reference_trial
                    and data.get('reason') == 'decision_source_expired'):
                from arena_autonomy.expiry_reference import reset_after_source_expiry
                reset_after_source_expiry(follow)
            else:
                follow.reset()"""
    assert source.count(old)==1;source=source.replace(old,new)
node=next(n for n in ast.parse(source).body if isinstance(n,ast.FunctionDef) and n.name=='replay_events')
namespace=vars(runtime).copy()
exec(compile(ast.Module(body=[node],type_ignores=[]),'r231_selected_replay','exec'),namespace)
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
before=sha(trace)
try:
    with trace.open() as f:report=namespace['replay_events'](json.loads(line) for line in f)
except (ValueError,KeyError,TypeError) as e:
    report=dict(passed=False,capture_valid=False,error=str(e))
report.update(mode=args.mode,input_sha256=before,input_unchanged=sha(trace)==before,
    original_replay_source_sha256=sha(path),scope='saved controller stage only, excludes actual transport/guard/physics')
with (A/f'{args.mode}.json').open('x') as f:json.dump(report,f,indent=2);f.write('\n')
print(json.dumps({k:v for k,v in report.items() if k not in ('outcomes','mismatches','audit_metadata')}),flush=True)
raise SystemExit(0 if report['passed'] and report['input_unchanged'] else 1)
