"""고정 센서 재생: 이동 앞차 속도에 '우회 여유보다 먼 거리'의 접근분을 더한 후보."""
import argparse
from collections import defaultdict
import hashlib
import json
from pathlib import Path
from arena_autonomy.bypass_handoff import BypassHandoffFollow,HandoffExecutionConfig
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig


class ClosingProbe(BypassHandoffFollow):
    def _follow_entry_space_current(self,scene,normal,previous,target):
        v,d=super()._follow_entry_space_current(scene,normal,previous,target)
        if (d['reason'] not in ('advancing_patch_follow','observed_entry_distance_reserve') or
                not self.monitor.motion_evidence() or self.monitor.forward_rate_mps<=0.):
            return v,d
        excess=max(0.,(d['rear_axle_to_patch_m']-d['reserve_m'])/self.execution.observation_window_s)
        # 기존 1.2 m 진입 여유 및 0.8초 창 유지. 전진 증거가 있는 구간에서만
        # 앞차의 관측 전진율과 남은 거리를 따로 더한다. 정상 G1 상한은 넘지 않는다.
        pace=self.monitor.forward_rate_mps+excess
        return min(target,pace),dict(d,reason='advancing_patch_close',cap_mps=pace,applied=pace<target)


def measure(folder):
    r=json.loads((folder/'report.json').read_text());green=r['green_observed_sim_s'];case=r['case']
    phases=defaultdict(lambda:dict(calls=0,higher=0,lower=0,zero_released=0,changed_steer=0,
                                  changed_state=0,added_command_distance_m=0.,max_extra_mps=0.))
    digest=hashlib.sha256();prev=None;footer=None
    with (folder/'follow_decisions.jsonl').open('rb') as stream:
        for seq,line in enumerate(stream):
            digest.update(line);e=json.loads(line);d=e['data']
            assert seq==e['seq'] and footer is None
            if e['kind']=='header':
                config=HandoffExecutionConfig.from_mapping(d['execution_config']);assert config.moving_follow_reserve
                ctrl=ClosingProbe(FollowConfig(**d['follow_config']),config,allow_simulation_trial=True)
                motion=MotionConfig(**d['motion_config'])
            elif e['kind']=='reset':ctrl.reset();prev=None
            elif e['kind']=='footer':footer=d
            elif e['kind']=='call':
                v,angle,decision=replay_call(ctrl,d['input'],motion)
                t=d['input']['reference_s']-green
                end=r['case_window']['duration_s']
                phase='moving' if case=='slow_lead' or t<4 else ('stopped' if t<9 else 'restarted')
                old,old_angle,od=d['output'];v=float(v);angle=float(angle);dv=v-old
                if prev and 0<t-prev[0]<=.10 and prev[0]>=0 and t<=end:
                    phases[prev[1]]['added_command_distance_m']+=prev[2]*(t-prev[0])
                if 0<=t<=end:
                    s=phases[phase];s['calls']+=1;s['higher']+=dv>1e-8;s['lower']+=dv< -1e-8
                    s['zero_released']+=old<=.001 and v>.001
                    s['changed_steer']+=abs(angle-old_angle)>1e-8
                    s['changed_state']+=decision['handoff_state']!=od['handoff_state']
                    s['max_extra_mps']=max(s['max_extra_mps'],dv)
                prev=t,phase,dv
    assert footer and footer['written']==seq and all(footer[k]==0 for k in ('failed','dropped','capture_errors','pending'))
    assert digest.hexdigest()==r['follow_decisions']['sha256']
    return dict(case=case,phases=phases,trace_sha256=digest.hexdigest(),scope='동일 moving 센서/이력. 속도 명령 차이의 적분은 실제 거리 이득이나 급정지 안전 증명이 아니다.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);p.add_argument('output',type=Path)
    a=p.parse_args();r=measure(a.run)
    serialized=json.dumps(r,ensure_ascii=False,indent=2)
    with a.output.open('x',encoding='utf-8') as f:f.write(serialized)
    print(json.dumps(r,ensure_ascii=False))
