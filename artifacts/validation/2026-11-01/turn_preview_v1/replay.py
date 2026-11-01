"""11/1 실제 완주 센서 입력에서 선택형 코너 속도 변화·정지/조향 보존을 확인한다."""
from collections import Counter
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from arena_autonomy.follow_trace import replay_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig, LocalFollow


def run(source):
    counts=Counter(); reasons=Counter(); digest=hashlib.sha256(); changes=[]
    with source.open('rb') as f:
        for line in f:
            digest.update(line); e=json.loads(line);d=e['data']
            if e['kind']=='header':
                cfg=FollowConfig(**d['follow_config'])
                assert cfg.blend_open_speed_by_length and not cfg.preview_turn_speed
                ctrl=LocalFollow(replace(cfg,preview_turn_speed=True)); motion=MotionConfig(**d['motion_config'])
            elif e['kind']=='reset':ctrl.reset()
            elif e['kind']=='call':
                v,angle,detail=replay_call(ctrl,d['input'],motion)
                old,old_angle,_=d['output']; t=d['input']['reference_s']
                counts['calls']+=1
                counts['released_zero']+=int(old<=1.e-9<v)
                counts['new_zero']+=int(v<=1.e-9<old)
                counts['angle_changed']+=int(abs(angle-old_angle)>1.e-9)
                counts['lower']+=int(v<old-1.e-9)
                if v>old+1.e-9:
                    counts['higher']+=1
                    changes.append({'t':t,'old_mps':old,'new_mps':v,'source_s':detail['source_first_s']})
                reason=detail.get('turn_preview')
                if reason:reasons[reason]+=1
    return dict(input_sha256=digest.hexdigest(),counts=dict(counts),reasons=dict(reasons),
                increased_scans=len({x['source_s'] for x in changes}),
                highest_extra_mps=max((x['new_mps']-x['old_mps'] for x in changes),default=0.),
                command_changes=changes,scope='기존 주행 입력. 평균 실제속도·완주시간·더 빠른 궤적의 성능이 아니다.')


if __name__=='__main__':
    base=Path(__file__).resolve().parent
    result=run(base.parent/'corner_approach_v1/solo_2p0/follow_decisions.jsonl')
    with (base/'fixed_input.json').open('x',encoding='utf-8') as f:json.dump(result,f,indent=2,ensure_ascii=False)
    print(result['counts'],result['reasons'])
    assert result['counts']['higher'] and not any(result['counts'][k] for k in ('released_zero','new_zero','lower','angle_changed'))
