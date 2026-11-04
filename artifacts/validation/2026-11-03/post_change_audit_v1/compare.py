"""기둥 대응 변경 전/후의 격리 저장입력 대조. 실제 대안 주행의 증거가 아니다."""
from pathlib import Path
import argparse, hashlib, json, subprocess, sys, time, zipfile
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
BASE=ROOT/'build/post_change_audit_20261103'
SPECS={
 'arena_autonomy/local_follow.py':'solo_alternative_r1',
 'arena_autonomy/bypass_handoff_corridor_trial.py':'entry_obstruction_r1',
 'arena_autonomy/pass_release_trial.py':'observed_gap_recovery_r1',
}
TRACES={
 'normal_bypass':ROOT/'artifacts/validation/2026-11-02/official_traffic_v1/p1/follow_decisions.jsonl',
 'restart_merge':ROOT/'artifacts/validation/2026-11-02/official_traffic_v1/r1/follow_decisions.jsonl',
 'selected_solo':ROOT/'artifacts/validation/2026-11-02/solo_alternative_speed_v1/run1/follow_decisions.jsonl',
 'post_free_front':ROOT/'artifacts/validation/2026-11-03/post_free_facility_v1/run1/follow_decisions.jsonl',
}
def sha(data):return hashlib.sha256(data).hexdigest()
def prepare():
 BASE.mkdir(parents=True,exist_ok=False)
 manifest={}
 for mode in ('current','before'):
  dest=BASE/mode;files={}
  for package in ('arena_autonomy','arena_vehicle_interface'):
   source=ROOT/'src'/package/package
   for p in source.rglob('*.py'):
    rel=p.relative_to(source.parent);target=dest/rel;target.parent.mkdir(parents=True,exist_ok=True)
    data=p.read_bytes();target.write_bytes(data);files[rel.as_posix()]=sha(data)
  if mode=='before':
   for rel,name in SPECS.items():
    contract=json.loads((ROOT/f'config/tests/{name}_sha256.json').read_text())
    source='src/arena_autonomy/'+rel
    with zipfile.ZipFile(ROOT/contract['before_snapshot']) as z:data=z.read(source)
    assert sha(data)==contract['approved_changes'][source]['before']
    (dest/rel).write_bytes(data);files[rel]=sha(data)
  manifest[mode]=files
 (HERE/'snapshot_manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
 with zipfile.ZipFile(HERE/'source_snapshots.zip','x',zipfile.ZIP_DEFLATED) as z:
  for p in BASE.rglob('*.py'):z.write(p,p.relative_to(BASE).as_posix())
def child(mode,label,round_id):
 sys.path.insert(0,str(BASE/mode))
 import numpy as np
 from arena_autonomy.follow_trace import replay_call,freeze,equal_output
 from arena_autonomy.local_follow import FollowConfig,LocalFollow
 from arena_autonomy.lidar_motion import MotionConfig
 from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
 import arena_autonomy.bypass_handoff_corridor_trial as corridor
 gate_cost=[]
 if hasattr(corridor,'entry_path_conflict'):
  original_gate=corridor.entry_path_conflict
  def timed_gate(*args,**kwargs):
   begin=time.perf_counter_ns()
   try:return original_gate(*args,**kwargs)
   finally:gate_cost.append((time.perf_counter_ns()-begin)/1e6)
  corridor.entry_path_conflict=timed_gate
 calls=0;costs=[];first=None;cmd_hash=hashlib.sha256(); extra_calls=0
 for line in TRACES[label].open():
  event=json.loads(line);data=event['data']
  if event['kind']=='header':
   cfg=FollowConfig(**data['follow_config']);motion=MotionConfig(**data['motion_config'])
   control=(BypassHandoffFollow(cfg,HandoffExecutionConfig.from_mapping(data['execution_config']),allow_simulation_trial=True)
            if 'execution_config' in data else LocalFollow(cfg))
  elif event['kind']=='reset':control.reset()
  elif event['kind']=='call':
   begin=time.perf_counter_ns();actual=replay_call(control,data['input'],motion);costs.append((time.perf_counter_ns()-begin)/1e6)
   actual=freeze(actual);expected=data['output'];calls+=1
   vector=[actual[0],actual[1],actual[2].get('handoff_state')]
   cmd_hash.update(json.dumps(vector,sort_keys=True).encode())
   extra_calls+=int('entry_path_conflict' in str(actual[2].get('entry_admission')))
   if not equal_output(actual[:2],expected[:2]) or actual[2].get('handoff_state')!=expected[2].get('handoff_state'):
    first=dict(call=calls,reference_s=data['input']['reference_s'],expected=expected[:2],actual=actual[:2],
               expected_mode=expected[2].get('handoff_state'),actual_mode=actual[2].get('handoff_state'))
    break
 modules={k:v.__file__ for k,v in sys.modules.items() if k.startswith(('arena_autonomy','arena_vehicle_interface')) and getattr(v,'__file__',None)}
 assert modules and all(Path(v).resolve().is_relative_to((BASE/mode).resolve()) for v in modules.values())
 result=dict(mode=mode,trace=label,round=round_id,calls=calls,first_control_difference=first,controls_sha256=cmd_hash.hexdigest(),
             new_entry_diagnostic_calls=extra_calls,entry_gate=dict(calls=len(gate_cost),total_ms=sum(gate_cost),maximum_ms=max(gate_cost,default=0.),share_of_decision_time=sum(gate_cost)/sum(costs)),decision_cost_ms=dict(median=float(np.median(costs)),p95=float(np.percentile(costs,95)),maximum=max(costs)),
             scope='저장 입력의 최초 제어 차이까지만. 로컬 재생 판단함수 비용이며 실주행/Jetson 지연 아님.')
 with (HERE/f'{label}_{mode}_{round_id}.json').open('x',encoding='utf-8') as f:json.dump(result,f,ensure_ascii=False,indent=2)
 print(json.dumps(result,ensure_ascii=False))
def main():
 parser=argparse.ArgumentParser();parser.add_argument('--child',nargs=3);parser.add_argument('--prepare-only',action='store_true');args=parser.parse_args()
 if args.child:return child(*args.child)
 if args.prepare_only:return prepare()
 assert BASE.is_dir()
 for label,path in TRACES.items():
  if not path.exists():raise FileNotFoundError(path)
  # Two crossed orders, after Gazebo completes. Same inputs; no overlap with a live run.
  modes=[('current','1'),('before','1'),('before','2'),('current','2')] if label in ('normal_bypass','restart_merge') else [('current','1'),('before','1')]
  for mode,round_id in modes:
   subprocess.run([sys.executable,'-I','-B',str(Path(__file__).resolve()),'--child',mode,label,round_id],check=True,cwd=BASE/mode)
if __name__=='__main__':main()
