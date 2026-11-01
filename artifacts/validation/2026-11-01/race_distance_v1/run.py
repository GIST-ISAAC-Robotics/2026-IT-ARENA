"""별도 프로세스의 평가기만 5랩으로 확장한다. 제어 입력은 기존 센서와 동일하다."""
import argparse
from hashlib import sha256
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from prepare import HERE,ROOT,expected,geometry,CHECK_FIVE

sys.path.insert(0,str(ROOT/'scripts'))
import race_traffic_course
import g1_course_eval
import validate_g1_traffic
from traffic_actors import overlay_case
from prelap_signal_eval import fixture_inputs
from g1_handoff_eval import inspect_header

ENV=HERE.parent/'turn_preview_v1/environment.sh'
CONFIG=HERE/'course.json';OVERLAY=HERE/'actor_overlay.json';OUTPUT=HERE/'run1'


def short_results():
    prior=HERE.parent/'rolling_lead_v1';out={}
    for name in ('run_recent','stop_recent'):
        r=json.loads((prior/name/'report.json').read_bytes())
        if (not r['run_valid'] or not r['source_unchanged'] or not r['shutdown_clean']
                or not all(r['stop_checks'].values()) or not r['gates']['no_actor_contact']
                or not r['gates']['no_sampled_overlap'] or not r['gates']['inside_road']):
            raise ValueError('짧은 실제 후보 주행의 유효성·접촉·정지·노면 확인 필요')
        if name=='run_recent' and not r['passed']:raise ValueError('첫 코너 추종을 먼저 해결해야 함')
        head=inspect_header(prior/name/'follow_decisions.jsonl','recent')
        if not head['passed']:raise ValueError('실제 선택 구성 불일치')
        out[name]=sha256((prior/name/'report.json').read_bytes()).hexdigest()
    return out


def parameters():
    config=json.loads(CONFIG.read_bytes());overlay=json.loads(OVERLAY.read_bytes())
    if (config,overlay)!=expected():raise ValueError('등록 각본·기존 판정 변경')
    if geometry(config,overlay)!=json.loads((HERE/'geometry.json').read_bytes()):raise ValueError('상대 경로 기하 변경')
    return config,overlay


def facility(args,config,case):
    cfg,_=parameters()
    if (args.config.resolve()!=CONFIG or args.actor_overlay.resolve()!=OVERLAY
            or args.output.resolve()!=OUTPUT or args.mode!='g1g2' or args.seed!=0
            or args.handoff_reserve!='recent' or not args.record_follow_decisions
            or case!=overlay_case(cfg['cases'][0],OVERLAY) or config!=cfg):
        raise ValueError('등록 5랩 평가 인자 불일치')
    files=set(validate_g1_traffic.source_paths(CONFIG,actor_overlay=OVERLAY))
    files.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    files.update(p.relative_to(ROOT).as_posix() for p in (Path(__file__).resolve(),HERE/'prepare.py',
        HERE/'course.launch.py',HERE/'geometry.json',CHECK_FIVE,ENV,
        ROOT/'artifacts/validation/2026-10-31/g1_continuous_prefix_course_v1/record_chase.py'))
    inputs=fixture_inputs();inputs.update({p:sha256((ROOT/p).read_bytes()).hexdigest() for p in files})
    return inputs


def registration():
    config,_=parameters()
    args=['scripts/validate_g1_traffic.py','--mode','g1g2','--handoff-reserve','recent','--team-signal-course',
        '--config',str(CONFIG),'--actor-overlay',str(OVERLAY),'--case','stop_restart_full_course',
        '--seed','0','--record-follow-decisions','--output',str(OUTPUT)]
    request=SimpleNamespace(config=CONFIG,actor_overlay=OVERLAY,output=OUTPUT,mode='g1g2',
                            seed=0,handoff_reserve='recent',record_follow_decisions=True)
    return dict(args=args,source_sha256=facility(request,config,overlay_case(config['cases'][0],OVERLAY)),
        short_run_report_sha256=short_results(),scope='300초·233.1645m 이상 순진행 5랩. 평가기 각본·종료이지 자차 랩 인식이나 5대 자율 상대차 경기 아님.')


def execute():
    reg=json.loads((HERE/'registered.json').read_bytes())
    if registration()!=reg:raise ValueError('등록 후 변경')
    spec=importlib.util.spec_from_file_location('saved_five_lap_eval',CHECK_FIVE)
    candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)
    # 기존 10/29에 사용한 평가 전용 랩 개수 검사. 자차 노드·지도·관측에 전달하지 않는다.
    g1_course_eval.check_full_course=candidate.check_full_course
    race_traffic_course.inputs=facility
    def entry(output):
        if Path(output).resolve()!=OUTPUT:raise ValueError('등록된 출력만 허용')
        return HERE/'course.launch.py'
    race_traffic_course.entry_for_output=entry
    sys.argv=list(reg['args'])
    return validate_g1_traffic.main()


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--execute',action='store_true');a=p.parse_args()
    if OUTPUT.exists():raise FileExistsError('이전 결과 보존')
    data=registration()
    if a.prepare_only:
        with (HERE/'registered.json').open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    elif a.execute:raise SystemExit(execute())
    else:
        if data!=json.loads((HERE/'registered.json').read_bytes()):raise ValueError('등록 후 변경')
        with (HERE/'run1_console.log').open('x') as f:
            r=subprocess.run(['bash',str(ENV),sys.executable,'-B',str(Path(__file__).resolve()),'--execute'],
                             cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        with (HERE/'run1_exit.log').open('x') as f:f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
