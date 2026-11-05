"""실제 추종 주행의 녹색 2.5초 부근 현재 광선에서 진입 폭 계산 한 장면씩 뽑는다."""
from pathlib import Path
import json
from arena_autonomy.bypass_handoff import front_patch
from arena_autonomy.bypass_entry_trial import straight_reference
from arena_autonomy.follow_trace import restore_call,motion_scene
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.moving_pass import available_pass_offsets
from review import lines,read
HERE=Path(__file__).resolve().parent
def example(run):
    tg=read(run/'report.json')['case_window']['green_sim_s'];rows=lines(run/'follow_decisions.jsonl')
    head=next(rows)['data'];cfg=FollowConfig(**head['follow_config']);motion=MotionConfig(**head['motion_config'])
    for e in rows:
        if e['kind']!='call':continue
        inp=e['data']['input']
        if not tg+2.5<=inp['reference_s']<=tg+3.0:continue
        inp,scan,hist=restore_call(inp,motion);scene=motion_scene(scan,hist,inp['reference_s'],
            timing_verified=inp['timing_verified'],epoch=inp['epoch'],rear_half_angle_deg=inp['rear_half_angle_deg'])
        fit,ref=straight_reference(scene,cfg)
        patch,_=front_patch(scene,head['execution_config']['front_patch_policy'])
        if fit is not None and patch is not None:
            offsets,report=available_pass_offsets(scene,cfg,ref['walls'],patch)
            return dict(t_after_green_s=scene.t_ref_s-tg,proposed_offsets_m=offsets,sides=report)
    return None
if __name__=='__main__':
    out=dict(scope='observed_lane_width_comparison_only_not_motion_authorization',
             centered=example(HERE.parent/'moving_follow_continuity_v1/closed_loop_v2/run1'),
             offset=example(HERE/'closed_loop_side_follow_1/run1'))
    assert out['centered'] and out['offset']
    with (HERE/'width_example.json').open('x',encoding='utf-8') as f:json.dump(out,f,indent=1);f.write('\n')
    print(out)
