"""실제 moving 실행의 센서 및 액추에이터 이력을 고정하고 fixed 명령만 다시 계산한다."""
from dataclasses import replace
import argparse
import hashlib
import json
from pathlib import Path

from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


def measure(folder):
    report=json.loads((folder/'report.json').read_text())
    assert report['run_valid'] and report['handoff_execution']['passed']
    g=report['green_observed_sim_s']; duration=report['case_window']['duration_s']
    sums=dict(calls=0,original_zeros=0,fixed_zeros=0,higher_in_moving=0,lower_in_moving=0,
              steering_changes=0,mode_changes=0)
    total_gain=0.;digest=hashlib.sha256();footer=None
    with (folder/'follow_decisions.jsonl').open('rb') as stream:
        for seq,line in enumerate(stream):
            digest.update(line);e=json.loads(line);d=e['data']
            assert e['seq']==seq and footer is None
            if e['kind']=='header':
                assert d['execution_config']['moving_follow_reserve'] is True
                cfg=FollowConfig(**d['follow_config']);motion=MotionConfig(**d['motion_config'])
                fixed=BypassHandoffFollow(cfg,replace(HandoffExecutionConfig.from_mapping(d['execution_config']),
                                                     moving_follow_reserve=False),allow_simulation_trial=True)
            elif e['kind']=='reset':fixed.reset()
            elif e['kind']=='call':
                v,angle,detail=replay_call(fixed,d['input'],motion)
                before,old_angle,original=d['output'];t=d['input']['reference_s']
                if g<=t<=g+duration:
                    sums['calls']+=1;sums['original_zeros']+=before==0;sums['fixed_zeros']+=v==0
                    sums['higher_in_moving']+=before>v+1e-8;sums['lower_in_moving']+=before<v-1e-8
                    sums['steering_changes']+=abs(angle-old_angle)>1e-8
                    sums['mode_changes']+=detail.get('handoff_state')!=original.get('handoff_state')
                    total_gain+=before-v
            elif e['kind']=='footer':footer=d
    assert digest.hexdigest()==report['follow_decisions']['sha256']
    assert footer and all(footer[k]==0 for k in ('capture_errors','failed','dropped','pending')) and footer['written']==seq
    return dict(**sums,mean_command_gain_mps=total_gain/sums['calls'],trace_sha256=digest.hexdigest(),
                scope='같은 moving 궤적의 명령 차이. fixed 실제 주행 거리·안전 결과가 아님.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('out',type=Path)
    args=p.parse_args();r=measure(args.run)
    with args.out.open('x',encoding='utf-8') as f:json.dump(r,f,ensure_ascii=False,indent=2)
    print(json.dumps(r,ensure_ascii=False))
