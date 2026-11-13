"""저장 실패 장면의 독립 전체영역 반례와 현재 검사 연결. 새 주행 아님."""
from pathlib import Path
from dataclasses import replace
import copy
import hashlib
import importlib.util
import json
import math
import sys

A=Path(__file__).resolve().parent;ROOT=A.parents[3]
sys.path[:0]=[str(ROOT/'src/arena_autonomy'),str(ROOT/'src/arena_vehicle_interface')]
import candidate
from arena_autonomy.follow_trace import restore_call,freeze
from arena_autonomy.lidar_motion import MotionConfig
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.numeric_kernels import configure
from arena_autonomy.scene_motion import motion_scene
from arena_autonomy.pass_release_trial import PassReleaseSelector


def main():
    target=A/'probes.json';assert not target.exists()
    source=A.parent/'final_geometry_crawl_drive_r353_v1/run1/follow_decisions.jsonl'
    source_before=hashlib.sha256(source.read_bytes()).hexdigest()
    records=[json.loads(l) for l in source.open()]
    header=records[0]['data'];calls={r['seq']:r['data'] for r in records if r['kind']=='call'}
    results={r['seq']:r for r in json.loads((A/'python_replay/decisions.json').read_bytes())}
    cfg=FollowConfig(**header['follow_config']);motion=MotionConfig(**header['motion_config'])
    configure('native_v1')
    spec=importlib.util.spec_from_file_location('r361_current',A.parent/'solo_crawl_retry_r352_v1/candidate.py')
    current_policy=importlib.util.module_from_spec(spec);spec.loader.exec_module(current_policy)
    from overlap import plan_reference
    checks=[];current_checks=[]
    for seq in (808,828,976):
        call=calls[seq];raw=call['input'];d,scan,history=restore_call(raw,motion)
        def scene_for(s):
            return motion_scene(s,history,d['reference_s'],timing_verified=d['timing_verified'],
                                epoch=d['epoch'],rear_half_angle_deg=d['rear_half_angle_deg'])
        scene=scene_for(scan);normal=call['output'][2]['normal'];shadow=results[seq]['shadow'][0]
        baseline=(shadow['baseline_speed_mps'],shadow['baseline_steering_rad'],
                  {**normal,'reason':normal['baseline_reason'],'behavior':'FOLLOW'})
        selected=(call['output'][0],call['output'][1],normal)
        owner=PassReleaseSelector(cfg)
        for side,values in normal['selector_sides'].items():owner.sides[side].update(values)
        saved=freeze((owner.sides,baseline,selected,d))
        accepted,e=candidate.assess(owner,scene,baseline,selected,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
        assert e['applied'] and accepted[0]==.1
        assert freeze((owner.sides,baseline,selected,d))==saved
        checks.append(dict(seq=seq,kind='original',evidence=e))
        for label,speed in [('actual_point15',.15),('actual_point35',.35)]:
            answer,proof=candidate.assess(owner,scene,baseline,selected,speed,d['previous_steering_rad'],d['dt_s'])
            physical=candidate.ps.motion_region(scene,speed,.1,baseline[1],d['previous_steering_rad'],d['dt_s'],cfg,owner.lat)
            assert answer is selected and not proof['applied']
            assert candidate.ps.region_blocked(physical,owner.config) is not None
            checks.append(dict(seq=seq,kind=label,evidence=proof,actual_region=physical))
        for label in ('unknown_front','occupied_front'):
            ranges=list(scan.ranges_m)
            for ray in scene.rays:
                if abs(math.atan2(ray.direction_xy[1],ray.direction_xy[0]))<math.radians(60):
                    ranges[ray.source_index]=math.inf if label=='unknown_front' else .09
            altered=replace(scan,ranges_m=tuple(ranges));altered_scene=scene_for(altered)
            answer,proof=candidate.assess(owner,altered_scene,baseline,selected,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
            assert answer is selected and proof['eligible'] and not proof['applied']
            checks.append(dict(seq=seq,kind=label,evidence=proof))
        masked=replace(scene,rays=tuple(replace(r,status='masked',hit_xy_m=None)
            if abs(math.atan2(r.direction_xy[1],r.direction_xy[0]))<math.radians(60) else r for r in scene.rays))
        answer,proof=candidate.assess(owner,masked,baseline,selected,d['speed_mps'],d['previous_steering_rad'],d['dt_s'])
        assert answer is selected and proof['eligible'] and not proof['applied']
        checks.append(dict(seq=seq,kind='synthetic_front_mask',evidence=proof))
        plan=(accepted[0],accepted[1],{**call['output'][2],'normal':accepted[2],'behavior':'FOLLOW'})
        # 실제 미래 궤적을 만들지 않는다. 같은 취득 입력을 현재 검사에 통째로 전달한다.
        checked=current_policy.rebase(plan,plan_reference(raw),raw,scene,history,cfg,require_current_region=True)
        assert checked is not None and checked['speed_mps']==.1
        assert checked['current_route_proof']['accepted'] is True
        current_checks.append(dict(seq=seq,scenario='same_recorded_current_input',output=freeze(checked)))
        later=calls.get(seq+1)
        if later is not None:
            now,now_scan,now_history=restore_call(later['input'],motion)
            now_scene=motion_scene(now_scan,now_history,now['reference_s'],timing_verified=now['timing_verified'],
                epoch=now['epoch'],rear_half_angle_deg=now['rear_half_angle_deg'])
            checked=current_policy.rebase(plan,plan_reference(raw),later['input'],now_scene,now_history,cfg,require_current_region=True)
            current_checks.append(dict(seq=seq,next_seq=seq+1,scenario='next_recorded_current_input',output=freeze(checked)))
    report=dict(checks=checks,current_checks=current_checks,source_sha256=source_before,
                source_unchanged=hashlib.sha256(source.read_bytes()).hexdigest()==source_before,
                scope='three saved states and synthetic sensor counterexamples; current-check math only, no publication, guard process, or closed-loop motion')
    target.write_text(json.dumps(report,ensure_ascii=False,indent=2)+'\n')
    print(json.dumps(dict(geometry_checks=len(checks),current_checks=len(current_checks),
        current_positive=sum(c['output'] is not None and c['output']['speed_mps']>0 for c in current_checks),
        source_unchanged=report['source_unchanged'])),flush=True)
    assert report['source_unchanged']


if __name__=='__main__':main()
