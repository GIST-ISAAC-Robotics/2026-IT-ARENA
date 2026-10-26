"""중앙 복귀 보류 OFF 저장 출력 대조 뒤 ON의 이상 궤적을 비교한다. 다른 시험 옵션은 ON이다."""
import argparse
import copy
import hashlib
import json
from pathlib import Path
import sys
import zipfile
import yaml
from shapely.geometry import MultiLineString
from arena_autonomy.follow_trace import freeze,replay_call
from arena_autonomy.local_follow import FollowConfig
from arena_autonomy.m3_execution_trial import ExecutionConfig,M3ExecutionFollow
from probe_grid_departure_observation import motion_history
from simulate_grid_departure import simulate
from probe_grid_follow import ROOT,CAMERA,VEHICLE,RUNTIME,registry,wall_sections,json_value
from audit_local_pursuit_road import road_polygon
from audit_grid_departure_motion import audit
from surface_contact_eval import VehicleGeometry


def rejected_yield_diagnostic_only(expected, actual):
    """명령/상태는 같고, 앞서 검사한 거절 yield 기록 하나만 추가된 경우다.

    전체 출력 동일 판정과 별개다. 선택 후보, 명령 거절, 해제 장부 등 다른
    차이를 지우거나 수치 허용오차를 적용하지 않는다.
    """
    try:
        old_checks = expected[2]['m3_lateral']['checks']
        new_checks = actual[2]['m3_lateral']['checks']
        check = new_checks['yield']
        rejected = (check.get('eligible_for_trial') is False or
                    (check.get('eligible_for_trial') is True and
                     check.get('command',{}).get('accepted') is False))
        if 'yield' in old_checks or not rejected:
            return False
        reduced = copy.deepcopy(actual)
        del reduced[2]['m3_lateral']['checks']['yield']
        return reduced == expected
    except (KeyError, TypeError, IndexError):
        return False


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--allow-rejected-yield-diagnostic',action='store_true',
        help='계획 또는 실제 명령 검사에서 거절된 새 yield 항목만 추가된 차이를 분리해 비교')
    parser.add_argument('--compare-yield-priority',action='store_true',
        help='직진에서 가능한 양보로 바뀐 실제 명령을 기록하고 새 궤적으로 비교; 출력 동일 검증 아님')
    args=parser.parse_args();out=args.output.resolve()
    if not out.is_relative_to(ROOT/'artifacts/validation'):parser.error('new validation output required')
    out.mkdir(parents=True,exist_ok=False)
    base=ROOT/'artifacts/validation/2026-10-26/m5_start_constraints_v1'
    vehicle=yaml.safe_load(VEHICLE.read_text(encoding='utf-8'))['vehicle']
    sensor=yaml.safe_load(CAMERA.read_text(encoding='utf-8'))['lidar'];wb=vehicle['drivetrain']['wheelbase_m']
    slots=registry(json.loads((RUNTIME/'scene.json').read_text(encoding='utf-8')),(RUNTIME/'world.sdf').read_bytes(),vehicle)
    segments=wall_sections((RUNTIME/'world.sdf').read_bytes(),sensor['xyz_m'][2])
    walls=MultiLineString(segments.tolist());road=road_polygon(RUNTIME/'world.sdf')
    geometry=VehicleGeometry(wheelbase_m=wb,track_width_m=vehicle['drivetrain']['track_width_m'])
    paths={Path(__file__),CAMERA,VEHICLE,RUNTIME/'world.sdf',RUNTIME/'scene.json'}
    paths.update((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    for module in tuple(sys.modules.values()):
        name=getattr(module,'__file__',None)
        if name:
            p=Path(name).resolve()
            if p.is_relative_to(ROOT) and p.suffix=='.py':paths.add(p)
    frozen={p.relative_to(ROOT).as_posix():p.read_bytes() for p in paths}
    with zipfile.ZipFile(out/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for n,b in frozen.items():z.writestr(n,b)
    cfg=FollowConfig(wheelbase_m=wb,occupancy_timing_mode='per_ray_candidate',
        boundary_evidence_mode='corner_distance_candidate',corridor_fallback_enabled=True)
    ex=ExecutionConfig(command_fallback=True,release_to_baseline=True,small_center_straight=True,
        straight_surface_commitment=True,rounded_occupancy=True,straight_with_threat=True)
    motion=motion_history(wb,sensor).config
    references={}
    for name,ref,index in (
        ('pause',base/'run41_departure_pause/data/grid2_ego5_pause_nominal/report.json',None),
        ('pause_delay',base/'run41_departure_pause/data/grid2_ego5_pause_delay50/report.json',None),
        ('five',base/'run20_straight_dynamics/data/report.json',1)):
        raw=ref.read_bytes();saved=json.loads(raw);saved=saved if index is None else saved['cases'][index]
        controller=M3ExecutionFollow(cfg,ex,allow_simulation_trial=True);diff=[];diagnostic=[];material=[];details=[];count=0
        for i,call in enumerate(saved['calls']):
            if call['explicit_stop']:break
            actual=replay_call(controller,call['captured_input'],motion);count+=1
            expected=[call['request_mps'],call['request_steering'],call['metadata']]
            actual=freeze(actual)
            if actual!=expected:
                diff.append(i)
                if rejected_yield_diagnostic_only(expected,actual):diagnostic.append(i)
                else:
                    material.append(i)
                    details.append(dict(index=i,old_command=expected[:2],new_command=actual[:2],
                        old_candidate=expected[2].get('m3_candidate'),new_candidate=actual[2].get('m3_candidate'),
                        old_reason=expected[2].get('reason'),new_reason=actual[2].get('reason')))
        references[name]=dict(path=ref.relative_to(ROOT).as_posix(),sha256=hashlib.sha256(raw).hexdigest(),
            unchanged=ref.read_bytes()==raw,calls=count,output_differences=diff,
            rejected_yield_diagnostic_only=diagnostic,other_differences=material,
            other_difference_commands=details,summary=saved['summary'])
    full_match=all(not r['output_differences'] for r in references.values())
    off_ok=all(r['unchanged'] and not r['other_differences'] and
        (args.allow_rejected_yield_diagnostic or not r['output_differences']) for r in references.values())
    # run53에서 확인한 정책 변경을 명시 비교한다. 이 경로는 이전 출력 동일 검사 성공이 아니다.
    # 다른 유형의 명령 변화나 원시 입력 변경은 여전히 운동 실행 전에 거절한다.
    comparison_ok=all(r['unchanged'] and r['calls']>0 and all(
        d['old_candidate']=='straight' and d['new_candidate']=='yield' and
        d['old_command'][0]==d['new_command'][0] and
        d['old_reason']==d['new_reason']=='m3_execution_trial'
        for d in r['other_difference_commands']) for r in references.values())
    reference_gate=off_ok or (args.compare_yield_priority and comparison_ok)
    (out/'off_replay.json').write_text(json.dumps(references,ensure_ascii=False,indent=2),encoding='utf-8')
    if not reference_gate:return 1
    cases=[]
    for name,count,delay,pause,fault in (('pause',2,0.,True,False),('pause_delay',2,.05,True,False),
                                        ('five',5,0.,False,False),('five_missing',5,0.,False,True)):
        selected=slots[:count];ego=next(s for s in selected if s['index']==5)
        result=simulate(ego,[s for s in selected if s!=ego],segments,road,vehicle,sensor,rounded=True,
            straight_priority=True,defer_center_on_straight=True,additional_scan_delay_s=delay,fault=fault,
            **(dict(peer_stop_s=1.8,peer_resume_s=3.8) if pause else {}))
        result['continuous_audit']=audit(result,selected,walls,road,geometry,ego_index=5)
        summary=result['summary'];summary['continuous_audit']=result['continuous_audit']
        summary['scan_schedule_valid']=all(row['frame']==max(s['frame'] for s in result['scans']
            if round(s['delivered_s']*1e9)<=round(row['t_s']*1e9)) for row in result['calls'])
        summary['expected_calls']=sum(round((.6018+i*.02)*1e9)>=round((.6018+delay)*1e9) for i in range(270))
        summary['deferred_straight_calls']=sum(c['metadata'].get('m3_lateral',{}).get('deferred_center',{}).get('ok',False)
            and c['metadata'].get('m3_candidate')=='straight' for c in result['calls'])
        case_dir=out/(f'grid{count}_ego5_'+name);case_dir.mkdir()
        (case_dir/'report.json').write_text(json.dumps(json_value(result),ensure_ascii=False,allow_nan=False),encoding='utf-8')
        cases.append(dict(name=name,**summary));print(json.dumps(cases[-1]),flush=True)
    unchanged=all((ROOT/n).read_bytes()==b for n,b in frozen.items())
    integrity=unchanged and reference_gate and all(c['scan_schedule_valid'] and c['calls']==c['expected_calls'] and
        c['continuous_audit']['passed'] and c['motion_input_failures']==0 and c['explicit_stop_checked'] and
        c['stopped_after_request'] and c['fault_positive_requests']==0 and c['fault_positive_safe']==0 for c in cases)
    summary=dict(cases=cases,off_replay=references,sources_unchanged=unchanged,integrity_pass=integrity,
        reference_all_outputs_equal=full_match,
        reference_equivalence_gate_passed=off_ok,
        reference_comparison_policy=('yield_priority_behavior_comparison' if args.compare_yield_priority else
            'rejected_yield_diagnostic_only' if args.allow_rejected_yield_diagnostic else 'exact'),
        source_sha256={n:hashlib.sha256(b).hexdigest() for n,b in frozen.items()},
        scope='deferred-center OFF references have other trial flags ON; explicit new ideal trajectories, no physical proof')
    (out/'summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
    return 0 if integrity and len(cases)==4 else 1


if __name__=='__main__':raise SystemExit(main())
