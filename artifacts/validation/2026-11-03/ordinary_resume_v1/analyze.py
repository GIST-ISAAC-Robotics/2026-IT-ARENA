"""일반 STOP→START 1회 실행의 사후 판정. 정답은 평가에서만 사용한다."""
import hashlib
import json
import math
from pathlib import Path
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
sys.path.insert(0, str(ROOT/'scripts'))
from surface_contact_eval import SurfaceMap, VehicleGeometry, rectangle
from race_ops_eval import Course


def main(name='run1'):
    assert name in ('run1','run2')
    run = HERE/name
    report = json.loads((run/'report.json').read_bytes())
    audit = json.loads((HERE/f'{name}_source_audit.json').read_bytes())
    steps = report['steps']
    named = {s['step']: s for s in steps}
    resume = named.get('resume_closed', {})
    final = named.get('final_continuous_stop', {}).get('evidence') or {}
    first = named.get('ordinary_stopped', {}).get('stop_evidence') or {}
    progress = named.get('ordinary_progress', {})
    poses = [json.loads(s)[:4] for s in (run/'ego_poses.jsonl').open() if s.strip()]
    coverage = bool(poses and all(0 < b[0]-a[0] <= .05 for a,b in zip(poses, poses[1:])))
    world = ROOT/'src/arena_gazebo/worlds/it_arena_official/world.sdf'
    surface, course, geometry = SurfaceMap.from_world(world), Course(), VehicleGeometry()
    radius = math.hypot(geometry.contact_patch_length_m/2, geometry.wheel_width_m/2)
    wall, three, max_off, branches = 0, 0, 0, 0
    for t,x,y,yaw in poses:
        c,s = math.cos(yaw), math.sin(yaw)
        off, hit = 0, False
        for lx,ly in geometry.wheel_centers().values():
            shape = rectangle(x+lx*c-ly*s, y+lx*s+ly*c, 0., 2*radius, 2*radius)
            off += not surface.road_contains(shape)
            hit |= surface.walls.intersects(shape)
        max_off = max(max_off, off)
        wall += hit
        three += off > 2
        branches += course.in_branch(x,y)
    stopped = [s for s in steps if s['step']=='stopped']
    gates = dict(runtime_completed=report.get('completed') is True and audit['runtime_exit']==0,
        first_start_requires_signal=named.get('first_start_cli_rejected',{}).get('code')==3,
        visual_start=named.get('visual_started',{}).get('source')=='visual',
        moving_before_stop=named.get('moving_before_stop',{}).get('body_speed_mps',0.)>.1,
        no_reset=not any(s['step']=='press_recovery_reset' for s in steps),
        one_accepted_resume=sum(s['step']=='press_resume_start' and s.get('stage')!='rejected_by_adapter' for s in steps)==1,
        resume_permitted=resume.get('last',{}).get('outcome')=='permitted',
        same_race=progress.get('same_race') is True, progressed=progress.get('progress_m',0.)>=3.,
        first_continuous_stop=first.get('passed') is True, final_continuous_stop=final.get('passed') is True,
        stop_policy_and_mcu=len(stopped)==2 and all(s.get('policy',{}).get('accepted') is True for s in stopped)
            and report.get('final_state',{}).get('mcu',{}).get('state')=='ESTOP_LATCHED',
        pose_coverage=coverage, conservative_surface=wall==0 and three==0, main_course=branches==0,
        source_unchanged=report.get('source_unchanged') is True and not audit['source_mismatches'],
        cleanup=not report.get('remaining_test_pids') and report.get('gazebo_stop_service',{}).get('verified') is True
            and all(not c['forced'] for c in report.get('auxiliary_cleanup',[])))
    result=dict(passed=all(gates.values()),gates=gates,failed=[k for k,v in gates.items() if not v],
        raw_report_passed=report.get('passed'),raw_error=report.get('error'),
        evaluator_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        resume=resume, progress=progress, first_stop=first, final_stop=final,
        surface=dict(samples=len(poses),max_possible_off_wheels=max_off,three_plus_possible_samples=three,
                     wall_possible_samples=wall,branch_samples=branches),
        scope='가상 MCU·자동HTTP·선두1칸·상향20도 단독 제자리 STOP→START 1회. 원시 report.passed 초기값과 별도. 모든 조향각 접지 외곽 평가이며 접촉 센서/실물·전체 랩 검증 아님')
    (HERE/f'{name}_analysis.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__=='__main__':
    raise SystemExit(main(*sys.argv[1:]))
