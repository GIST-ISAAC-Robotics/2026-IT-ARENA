"""원본 교통 기록의 함수별 비용 분해. cProfile 비용은 실시간 지연 수치가 아니다."""
from pathlib import Path
import cProfile
import hashlib
import json
import pstats

from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import restore_call,freeze,equal_output,comparable_decision,controller_mode
from arena_autonomy.lidar_motion import MotionConfig,MotionUnavailable
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene

ROOT=Path(__file__).resolve().parents[4]
HERE=Path(__file__).resolve().parent
TRACE=ROOT/'artifacts/validation/2026-11-02/official_traffic_v1/p1/follow_decisions.jsonl'


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    sources=list((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))+[Path(__file__)]
    before={str(p.relative_to(ROOT)):digest(p) for p in sources}
    profiles={};counts={};mismatches=[];calls=0
    for line in TRACE.open(encoding='utf-8'):
        e=json.loads(line);data=e['data']
        if e['kind']=='header':
            assert data['schema']==5
            ctl=BypassHandoffFollow(FollowConfig(**data['follow_config']),
                HandoffExecutionConfig.from_mapping(data['execution_config']),allow_simulation_trial=True)
            motion=MotionConfig(**data['motion_config'])
        elif e['kind']=='reset':ctl.reset()
        elif e['kind']=='call':
            calls+=1
            d,scan,history=restore_call(data['input'],motion)
            try:
                sc=motion_scene(scan,history,d['reference_s'],timing_verified=d['timing_verified'],
                    epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
                mode=ctl.mode
                profiler=profiles.setdefault(mode,cProfile.Profile())
                counts[mode]=counts.get(mode,0)+1
                actual=profiler.runcall(ctl.command,sc,history,d['speed_mps'],d['side'],
                    d['previous_steering_rad'],d['dt_s'],raw_scan=scan)
            except (MotionUnavailable,ValueError) as error:
                ctl.reset();actual=(0.,0.,dict(reason='follow_input_stop',behavior='HOLD',
                    input_error=str(error),mode=controller_mode(ctl)))
            if not equal_output(comparable_decision(freeze(actual)),comparable_decision(data['output'])):
                mismatches.append(e['seq'])
    groups={}
    for mode,profiler in profiles.items():
        profiler.dump_stats(str(HERE/('functions_'+mode.lower()+'.pstats')))
        stats=pstats.Stats(profiler);functions=[]
        for (file,line,name),(primitive,total,own,cumulative,callers) in stats.stats.items():
            if 'arena_autonomy' not in file:continue
            functions.append(dict(file=file.split('arena_autonomy/')[-1],line=line,name=name,
                primitive_calls=primitive,calls=total,own_s=own,cumulative_s=cumulative))
        groups[mode]=dict(calls=counts[mode],profile_total_s=stats.total_tt,
            top_cumulative=sorted(functions,key=lambda x:x['cumulative_s'],reverse=True)[:35],
            top_own=sorted(functions,key=lambda x:x['own_s'],reverse=True)[:20])
    unchanged=all(digest(ROOT/p)==h for p,h in before.items())
    result=dict(passed=not mismatches and unchanged,calls=calls,output_mismatches=mismatches,
        source_unchanged=unchanged,source_sha256=before,groups=groups,
        scope='cProfile-instrumented command functions, mode before call; not uninstrumented latency or Jetson deadline')
    (HERE/'function_costs.json').write_text(json.dumps(result,ensure_ascii=False,indent=1),encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k not in ('groups','source_sha256')}))
    return not result['passed']


if __name__=='__main__':raise SystemExit(main())
