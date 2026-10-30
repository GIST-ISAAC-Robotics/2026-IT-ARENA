"""같은 저장 입력에서 오프라인 후보(.225 등록값)와 런타임 후보의 판단을 대조한다. 주행 아님.

입력은 g2_inner_boundary_v1/trace2의 합성 입력이다. 기록 당시 명령과 다른 등록값이라 개루프 입력 재생이며,
두 구현의 동등성만 본다. 오프라인 전역 치환이 호출 뒤 복원되는지도 확인한다.
"""
import importlib.util
import json
from pathlib import Path
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface'),str(ROOT/'scripts')]
OFFLINE=ROOT/'artifacts/validation/2026-10-30/g2_inner_boundary_v1'
from arena_autonomy.bypass_corridor_trial import CorridorGuide
from arena_autonomy.bypass_entry_follow import BypassEntryFollow,BypassExecutionConfig
from arena_autonomy.bypass_entry_trial import EntryConfig
from arena_autonomy.follow_trace import equal_output,freeze,restore_call
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.scene_motion import motion_scene

spec=importlib.util.spec_from_file_location('offline_candidate',OFFLINE/'candidate.py')
offline=importlib.util.module_from_spec(spec);sys.modules['offline_candidate']=offline;spec.loader.exec_module(offline)
POLICY=offline.CorridorPolicy(surface=True,plan=True,onset=True,surface_error_budget_m=.06)


def comparable(value):
    if isinstance(value,dict):
        return {k:comparable(v) for k,v in value.items() if k!='processing_ms'}
    if isinstance(value,list):
        return [comparable(v) for v in value]
    return value


def run(path):
    calls=matched=0;differences=[];selected=0
    for line in path.open(encoding='utf-8'):
        event=json.loads(line);d=event['data']
        if event['kind']=='header':
            config=FollowConfig(**d['config']);entry=dict(d['entry'])
            entry['offsets_m']=tuple(entry['offsets_m']);entry['transitions_m']=tuple(entry['transitions_m'])
            motion=MotionConfig(**d['motion'])
            old=offline.CorridorGuide(config,EntryConfig(**entry),corridor_policy=POLICY,**d['guide_kwargs'])
            new=CorridorGuide(config,EntryConfig(**entry),**d['guide_kwargs'])
        elif event['kind']=='call':
            raw,scan,history=restore_call(d['input'],motion)
            scene=motion_scene(scan,history,raw['reference_s'],timing_verified=raw['timing_verified'],
                epoch=raw['epoch'],rear_half_angle_deg=raw['rear_half_angle_deg'])
            arguments=(scene,history,raw['speed_mps'],raw['previous_steering_rad'],raw['dt_s'])
            a=comparable(freeze(old.evaluate(*arguments)));b=comparable(freeze(new.evaluate(*arguments)))
            same=equal_output(a,b)
            calls+=1;matched+=same;selected+=b.get('selected') is not None
            if not same and len(differences)<3:
                differences.append(dict(seq=event['seq'],keys=[k for k in set(a)|set(b) if not equal_output(a.get(k),b.get(k))]))
    return dict(calls=calls,matched=matched,selected=selected,differences=differences,passed=calls>0 and matched==calls)


def main():
    result={}
    for name in ('B_center_s17','B_center_s23','B_edge_right_s17'):
        result[name]=run(OFFLINE/'trace2'/name/'input_output.jsonl')
        print(name,result[name],flush=True)
    (BASE/'offline_runtime.json').write_text(json.dumps(result,indent=2)+'\n',encoding='utf-8')
    return 0 if all(r['passed'] for r in result.values()) else 1


if __name__=='__main__':
    raise SystemExit(main())
