"""r24와 r25 같은 저장 입력 대조. 실제 경로 변화/실차 검증이 아니다."""
import hashlib,json,sys,tempfile,zipfile
from pathlib import Path
from dataclasses import replace
HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
MODE=sys.argv[1]
assert MODE in ('base','connected')
temp=None
if MODE=='base':
    temp=tempfile.TemporaryDirectory(prefix='arena_r24_before_r25_')
    with zipfile.ZipFile(HERE/'before.zip') as z:z.extractall(temp.name)
    sys.path.insert(0,str(Path(temp.name)/'src/arena_autonomy'))
from arena_autonomy.follow_trace import replay_call,comparable_decision,equal_output,freeze
from arena_autonomy.local_follow import FollowConfig,LocalFollow
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
import arena_autonomy.partial_observation as module
def digest(p):return hashlib.sha256(Path(p).read_bytes()).hexdigest()
specs=[
 ('solo_on','solo_corner_approach_connected_v1/run1/follow_decisions.jsonl',None),
 ('solo_off','solo_corner_approach_connected_v1/run1/follow_decisions.jsonl',False),
 ('traffic_off','rear_overtake_measured_v1/left_rear_overtakes/run1/follow_decisions.jsonl',None)]
results={}
for name,relative,override in specs:
    src=HERE.parent/relative
    before=digest(src);count=resets=rawdiff=0;outrows=[]
    for line in src.open():
        e=json.loads(line);d=e['data']
        if e['kind']=='header':
            cfg=FollowConfig(**d['follow_config']);mc=MotionConfig(**d['motion_config'])
            if override is not None:cfg=replace(cfg,oblique_gap_trial=override)
            controller=(BypassHandoffFollow(cfg,HandoffExecutionConfig.from_mapping(d['execution_config']),allow_simulation_trial=True)
                        if 'execution_config' in d else LocalFollow(cfg))
        elif e['kind']=='reset':
            controller.reset();resets+=1
            outrows.append(dict(seq=e['seq'],reset=d.get('reason')))
        elif e['kind']=='call':
            output=freeze(replay_call(controller,d['input'],mc));comp=comparable_decision(output)
            rawdiff+=not equal_output(comp,comparable_decision(freeze(d['output'])))
            det=output[2];normal=det.get('normal') or det
            partial=normal.get('partial_observation') or {}
            row=dict(seq=e['seq'],t=d['input']['reference_s'],speed=output[0],steer=output[1],
                     output_sha256=hashlib.sha256(json.dumps(comp,sort_keys=True,separators=(',',':')).encode()).hexdigest(),
                     reason=det.get('reason'),partial_reason=partial.get('reason'),partial_allowed=partial.get('allowed'),
                     lateral_metric=partial.get('lateral_metric'))
            outrows.append(row);count+=1
    assert digest(src)==before
    with (HERE/f'{MODE}_{name}.jsonl').open('w') as f:
        for row in outrows:f.write(json.dumps(row,ensure_ascii=False)+'\n')
    results[name]=dict(calls=count,resets=resets,raw_mismatches=rawdiff,source=relative,source_sha256=before,
                       oblique=cfg.oblique_gap_trial,output=f'{MODE}_{name}.jsonl')
report=dict(mode=MODE,policy_path=module.__file__,policy_sha256=digest(module.__file__),datasets=results,
            scope='same stored input comparisons, current r24 base not identical to historical r19')
(HERE/f'replay_{MODE}.json').write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
print(json.dumps(report,ensure_ascii=False))
if temp:temp.cleanup()
