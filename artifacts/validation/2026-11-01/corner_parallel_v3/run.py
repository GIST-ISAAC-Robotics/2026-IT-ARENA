"""등록한 17초 코너 동행 각본 한 번. 기본 제어값과 원래 평가 판정은 보존한다."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import sys
import prepare as plan

ROOT=plan.ROOT; HERE=plan.HERE
sys.path.insert(0,str(ROOT/'scripts'))
import validate_g1_traffic as evaluator
import race_traffic_course as course
from traffic_actors import overlay_case
from prelap_signal_eval import fixture_inputs

CONFIG=HERE/'course.json';OVERLAY=HERE/'actor_overlay.json';OUTPUT=HERE/'run1'
ARGS=['scripts/validate_g1_traffic.py','--mode','g1g2','--handoff-reserve','paced',
      '--handoff-yield-progress','--handoff-rear-sweep','--team-signal-course',
      '--config',str(CONFIG),'--actor-overlay',str(OVERLAY),'--case','first_curve',
      '--seed','0','--record-follow-decisions','--output',str(OUTPUT)]
original_route=evaluator.check_route


def route(config,line,case,start,road=None,obstacles=()):
    if case.get('actor_lateral_offset_m',0.)==plan.LATERAL:
        return plan.side_route(config,line,case,start,road,obstacles)
    return original_route(config,line,case,start,road,obstacles)


def inputs(args,cfg,case):
    wanted,overlay=plan.expected()
    if (cfg!=wanted or json.loads(OVERLAY.read_bytes())!=overlay
            or args.output.resolve()!=OUTPUT or args.config.resolve()!=CONFIG
            or args.actor_overlay.resolve()!=OVERLAY or args.seed!=0 or args.mode!='g1g2'
            or args.handoff_reserve!='paced' or not args.handoff_yield_progress
            or not args.handoff_rear_sweep or not args.record_follow_decisions
            or case!=overlay_case(cfg['cases'][0],OVERLAY)):
        raise ValueError('등록 코너 동행 조건 불일치')
    return source_hashes()


def source_hashes():
    result=fixture_inputs()
    paths=set(evaluator.source_paths(CONFIG,actor_overlay=OVERLAY))
    paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    paths.update(p.relative_to(ROOT).as_posix() for p in (HERE/'prepare.py',HERE/'run.py',
        HERE/'geometry.json',HERE/'README.md',plan.REFERENCE,plan.PARENT,
        ROOT/'config/tests/released_fringe_budget_r1_sha256.json'))
    result.update({n:sha256((ROOT/n).read_bytes()).hexdigest() for n in sorted(paths)})
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--register',action='store_true')
    args=parser.parse_args()
    if OUTPUT.exists():raise FileExistsError(OUTPUT)
    if args.register:
        cfg,overlay=plan.expected()
        if json.loads(CONFIG.read_bytes())!=cfg or json.loads(OVERLAY.read_bytes())!=overlay:
            raise ValueError('등록 설정 변경')
        if json.loads((HERE/'geometry.json').read_bytes())!=plan.geometry(cfg,overlay):
            raise ValueError('등록 기하 변경')
        plan.write(HERE/'registered.json',dict(args=ARGS,source_sha256=source_hashes()))
    else:
        registered=json.loads((HERE/'registered.json').read_bytes())
        if registered!=dict(args=ARGS,source_sha256=source_hashes()):raise ValueError('등록 이후 소스 변경')
        evaluator.check_route=route
        course.inputs=inputs
        from prelap_signal_eval import ENTRY
        def entry(output):
            if Path(output).resolve()!=OUTPUT:raise ValueError('등록 외 출력')
            return ENTRY
        course.entry_for_output=entry
        sys.argv=ARGS
        raise SystemExit(evaluator.main())
