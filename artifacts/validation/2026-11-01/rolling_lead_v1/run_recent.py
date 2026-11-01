"""최근 전진 근거만 켠 단독 상대차 비교. 소스·각본을 등록한 실행만 한 번 수행한다."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
from course import CONFIG,expected,geometry

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from validate_g1_traffic import source_paths,load_config
from g1_handoff_eval import launch_arguments
ENV=BASE.parent/'turn_preview_v1/environment.sh'


def register(which):
    curve=which=='curve';name='run_recent' if curve else 'stop_recent'
    output=BASE/name
    config=CONFIG if curve else BASE.parent/'race_follow_pair_v1/stop_restart.json'
    case='first_curve' if curve else 'stop_restart'
    reference=BASE/'run1' if curve else BASE.parent/'race_follow_closing_v1/stop'
    report=json.loads((reference/'report.json').read_bytes())
    if not report['run_valid'] or not report['source_unchanged'] or report['case']!=case:
        raise ValueError('유효한 기준 실행이 없음')
    cfg=load_config(config)
    launch_arguments('g1g2','recent',cfg['launch'],True)
    if curve and (cfg!=expected() or geometry(cfg)!=json.loads((BASE/'geometry.json').read_bytes())):
        raise ValueError('원래 코너 각본·판정 변화')
    args=['scripts/validate_g1_traffic.py','--mode','g1g2','--handoff-reserve','recent',
          '--config',str(config),'--case',case,'--seed','0','--record-follow-decisions','--output',str(output)]
    inputs={}
    if curve:
        from race_traffic_course import inputs as facility
        request=SimpleNamespace(config=config,actor_overlay=None,mode='g1g2',seed=0,
                                record_follow_decisions=True,output=output,case=case)
        inputs.update(facility(request,cfg,cfg['cases'][0]));args.append('--team-signal-course')
    paths=set(source_paths(config))
    paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    paths.update(p.relative_to(ROOT).as_posix() for p in (Path(__file__).resolve(),ENV,reference/'report.json',
        BASE/'recent_forward_candidate.py',BASE/'run1_recent_forward.json',BASE/'stop_recent_forward.json'))
    inputs.update({p:sha256((ROOT/p).read_bytes()).hexdigest() for p in paths})
    return dict(args=args,source_sha256=inputs,reference=reference.relative_to(ROOT).as_posix(),
        scope='기존 1.5 m/s·간격·노면·STOP·상대 각본 유지, 별도 3스캔 전진 근거만 선택. 후보가 실행 후 변경되면 재사용 금지.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case',choices=('curve','stop'))
    p.add_argument('--prepare-only',action='store_true');a=p.parse_args()
    name='run_recent' if a.case=='curve' else 'stop_recent';reg=BASE/(name+'_registered.json')
    if (BASE/name).exists():raise FileExistsError('실행 원본 덮어쓰기 금지')
    data=register(a.case)
    if a.prepare_only:
        with reg.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    else:
        if json.loads(reg.read_bytes())!=data:raise ValueError('실행 등록 후 변경')
        with (BASE/(name+'_console.log')).open('x') as f:
            r=subprocess.run(['bash',str(ENV),sys.executable,'-B',*data['args']],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        with (BASE/(name+'_exit.log')).open('x') as f:f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
