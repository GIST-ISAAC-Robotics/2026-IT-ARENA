"""현재 재관측 벽의 중심 복귀 경로를 원 점유 검사로 비교. 저장 입력의 국소 후보이며 폐루프 아님."""
from pathlib import Path
import json
import sys
import traceback

HERE=Path(__file__).resolve().parent
ORIGINAL=HERE.parent/'traffic_current_route_r478_v1'
sys.path.insert(0,str(ORIGINAL/'analysis_v1'))
from inspect_failure import restore_scene
from arena_autonomy.follow_trace import freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.moving_pass import MovingEntryConfig
from arena_autonomy.bypass_entry_trial import entry_path,_compare_entries


def main():
    rows=[json.loads(line) for line in (ORIGINAL/'run1/follow_decisions.jsonl').read_text(encoding='utf8').splitlines()]
    header=next(r['data']['header'] for r in rows if r['kind']=='runtime_evaluator_header')
    cfg=FollowConfig(**header['follow_config'])
    entry=MovingEntryConfig(speed_cap_mps=.9,profile='medium_v1',transitions_m=(.6,.9,1.2))
    results=[]
    for seq in (524,533,542,551,560,563):
        row=next(r for r in rows if r['kind']=='call' and r['seq']==seq)
        d,scan,history,scene=restore_scene(row['data']['input'],MotionConfig(**header['motion_config']))
        original=row['data']['output'];e=original[2]['moving_pass']['entry'];wall=e['reference']['walls']['left']
        fit=(wall['slope'],wall['intercept_m'],*wall['observed_x_m'])
        result={'seq':seq,'reference_s':d['reference_s'],'original_speed_mps':original[0],
            'original_steering_rad':original[1],'actual_speed_mps':d['speed_mps'],
            'tracker':original[2]['moving_pass']['target']}
        try:
            path=entry_path(fit,0.,cfg,entry,1.2)
            proposal=_compare_entries(scene,d['reference_s'],d['speed_mps'],d['previous_steering_rad'],d['dt_s'],
                config=cfg,entry=entry,guide={'reference':e['reference'],'path':path,'offset_m':0.,
                    'transition_m':1.2,'outer_side':'left','stationary_outer_boundary':True})
            result['proposal']=proposal
            c=proposal['candidates'][0]
            print(seq,d['reference_s'],'old',round(original[0],4),'new',round(c.get('target_speed_mps',0),4),
                'angle',round(c.get('steering_rad',0),4),'clearance',c.get('command_clearance_m'),
                c['reason'],'region',c.get('motion_region'),flush=True)
        except Exception as error:
            result.update(error=type(error).__name__,message=str(error),traceback=traceback.format_exc())
            print(seq,type(error).__name__,str(error),flush=True)
        results.append(result)
    (HERE/'center_return_probe.json').write_text(json.dumps(freeze({'scope':'원 scan/history와 공개 Python scene 경로; native evaluator 전체 재생 아님; 현재 관측 wall로 새 중심 경로만 비교','rows':results}),ensure_ascii=False,indent=2)+'\n',encoding='utf8')


if __name__=='__main__':main()
