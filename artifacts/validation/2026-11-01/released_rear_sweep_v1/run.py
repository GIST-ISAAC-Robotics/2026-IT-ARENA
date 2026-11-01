"""빠른 앞차의 후반부 이동 후보 비교. 각본/제어 기본값은 유지한다."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
import race_traffic_course as course
import validate_g1_traffic as evaluator
from traffic_actors import overlay_case,actor_specs,check_additional_routes
from prelap_signal_eval import FIXTURE,fixture_inputs
from g1_course_eval import check_full_course
from g1_handoff_eval import inspect_header

PARENT=HERE.parent/'mixed_course_v1/course.json'
OLD_OVERLAY=ROOT/'artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1/actor_overlay.json'
PRIOR=HERE.parent/'yield_progress_v1/course_paced'
CONFIG=HERE/'course.json';OVERLAY=HERE/'actor_overlay.json';OUTPUT=HERE/'run1'
ENV=HERE.parent/'turn_preview_v1/environment.sh'
DURATION=78.


def expected():
    cfg=json.loads(PARENT.read_bytes());overlay=json.loads(OLD_OVERLAY.read_bytes())
    case=cfg['cases'][0]
    assert case['schedule']==[dict(from_s=0.,to_s=4.,speed_mps=.35),
        dict(from_s=4.,to_s=9.,speed_mps=0.),dict(from_s=9.,to_s=155.,speed_mps=.35)]
    case.update(duration_s=DURATION,expected='재출발 후 빨라지는 앞차를 센서로 따라가며 순진행 한 랩 및 STOP.')
    case['schedule'][2:]=[dict(from_s=9.,to_s=20.,speed_mps=.55),dict(from_s=20.,to_s=DURATION,speed_mps=.825)]
    cfg.update(registered_at='2026-11-01 빠른 앞차 한 랩',
        scope='기존 초기 정지/재출발과 옆차 유지. 앞차 속도와 시험 창만 변경한 78초 비교.')
    overlay['additional_actors'][0]['schedule'][-1]['to_s']=DURATION
    overlay['scope']='기존 옆차가 15초까지 이동한 뒤 잔디 쪽에 정지. 이번 창은 78초.'
    return cfg,overlay


def geometry(cfg,overlay):
    from audit_local_pursuit_road import road_polygon
    from build_experimental_track import obstacle_polygons
    from g1_traffic_eval import validate_schedule,actor_travel_m
    case=overlay_case(cfg['cases'][0],OVERLAY)
    assert case['additional_actors']==overlay['additional_actors']
    for actor in [case,*case['additional_actors']]:validate_schedule(actor['schedule'],DURATION)
    line,scene=evaluator.load_centerline(FIXTURE)
    road=road_polygon(FIXTURE/'world.sdf');obstacles=obstacle_polygons(FIXTURE/'world.sdf')
    start=evaluator.actor_start_s(cfg,scene,0,case)
    primary=check_full_course(cfg,line,case,start,road,obstacles)
    _,other=check_additional_routes(cfg,case,actor_specs(cfg,case),scene,0,line,road,obstacles,
        evaluator.actor_start_s,evaluator.check_route,evaluator.actor_pose_at)
    return dict(primary=primary,additional=other,scheduled_travel_m=actor_travel_m(case['schedule'],DURATION),
        scope='평가 상대의 기하 배치만 확인. 자차 주행 및 실물 상대차를 보장하지 않음.')


def prerequisites():
    r=json.loads((PRIOR/'report.json').read_bytes())
    if (not all(r[k] for k in ('run_valid','source_unchanged','shutdown_clean'))
            or not all(r['stop_checks'].values())
            or not all(r['gates'][k] for k in ('full_lap','no_actor_contact','no_sampled_overlap','two_wheel_surface'))):
        raise ValueError('대표 코스 복귀/완주/정지의 선행 근거 필요')
    h=inspect_header(PRIOR/'follow_decisions.jsonl','paced',progress=True)
    if not h['passed']:raise ValueError('선행 실행의 선택형 구성 불일치')
    replay=json.loads((HERE/'replay_result.json').read_bytes())
    if replay[0]['counts'].get('zero_released',0)==0 or any(x['counts'].get('new_zero',0) for x in replay):
        raise ValueError('원래 출력 재현/정체 출력 개선 없이 재실행하지 않는다')
    return {str((PRIOR/'report.json').relative_to(ROOT)):sha256((PRIOR/'report.json').read_bytes()).hexdigest()}


def facility(args,cfg,case):
    wanted,overlay=expected()
    if (cfg!=wanted or json.loads(OVERLAY.read_bytes())!=overlay
            or args.config.resolve()!=CONFIG or args.actor_overlay.resolve()!=OVERLAY
            or args.output.resolve()!=OUTPUT or args.mode!='g1g2' or args.seed!=0
            or args.handoff_reserve!='paced' or not args.handoff_yield_progress or not args.handoff_rear_sweep
            or not args.record_follow_decisions or args.case!='stop_restart_full_course'
            or case!=overlay_case(cfg['cases'][0],OVERLAY)):
        raise ValueError('등록한 78초/센서/선택형 주행 구성 불일치')
    files=set(evaluator.source_paths(CONFIG,actor_overlay=OVERLAY))
    files.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    files.update(p.relative_to(ROOT).as_posix() for p in (Path(__file__).resolve(),HERE/'course.launch.py',
        HERE/'geometry.json',HERE/'trial_plan.json',ENV,PARENT,OLD_OVERLAY,
        ROOT/'config/tests/yield_progress_r1_sha256.json',ROOT/'config/tests/released_rear_sweep_r1_sha256.json',
        HERE/'replay_result.json',
        ROOT/'artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1/record_chase.py'))
    hashes=fixture_inputs();hashes.update({p:sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(files)})
    return hashes


def registration():
    cfg,overlay=expected()
    if json.loads(CONFIG.read_bytes())!=cfg:raise ValueError('등록 각본 변경')
    if geometry(cfg,overlay)!=json.loads((HERE/'geometry.json').read_bytes()):raise ValueError('상대 기하 변경')
    args=['scripts/validate_g1_traffic.py','--mode','g1g2','--handoff-reserve','paced','--handoff-yield-progress','--handoff-rear-sweep',
        '--team-signal-course','--config',str(CONFIG),'--actor-overlay',str(OVERLAY),
        '--case','stop_restart_full_course','--seed','0','--record-follow-decisions','--output',str(OUTPUT)]
    request=SimpleNamespace(config=CONFIG,actor_overlay=OVERLAY,output=OUTPUT,mode='g1g2',seed=0,
        handoff_reserve='paced',handoff_yield_progress=True,handoff_rear_sweep=True,record_follow_decisions=True,case='stop_restart_full_course')
    return dict(args=args,source_sha256=facility(request,cfg,overlay_case(cfg['cases'][0],OVERLAY)),
        prior_report_sha256=prerequisites(),scope='78초 1랩 평가. 기본값/물성/센서/보호 변화 없음.')


def write(path,data):
    with path.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--execute',action='store_true');a=p.parse_args()
    if OUTPUT.exists():raise FileExistsError('기존 실행 덮어쓰기 금지')
    if a.prepare_only:
        cfg,overlay=expected();write(CONFIG,cfg);write(OVERLAY,overlay)
        write(HERE/'geometry.json',geometry(cfg,overlay));write(HERE/'registered.json',registration())
    else:
        data=registration()
        if data!=json.loads((HERE/'registered.json').read_bytes()):raise ValueError('등록 후 소스 변경')
        if a.execute:
            course.inputs=facility
            def entry(output):
                if Path(output).resolve()!=OUTPUT:raise ValueError('등록 출력 이외 실행')
                return HERE/'course.launch.py'
            course.entry_for_output=entry
            sys.argv=data['args'];raise SystemExit(evaluator.main())
        with (HERE/'run1_console.log').open('x') as f:
            r=subprocess.run(['bash',str(ENV),sys.executable,'-B',str(Path(__file__).resolve()),'--execute'],
                cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        with (HERE/'run1_exit.log').open('x') as f:f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
