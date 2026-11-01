"""고정 소스를 등록하고 양보 갱신만 선택한 정지/코스 비교를 각 한 번 수행한다."""
from pathlib import Path
from hashlib import sha256
from types import SimpleNamespace
import argparse,json,subprocess,sys

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
import race_traffic_course as course
from traffic_actors import overlay_case
from validate_g1_traffic import source_paths,load_config
from g1_handoff_eval import launch_arguments
ENV=BASE.parent/'turn_preview_v1/environment.sh'
course.BASE=BASE;course.ENTRY=BASE/'course.launch.py'

def register(which,reserve):
    iscourse=which=='course'
    output=BASE/(which+'_'+reserve)
    config=course.CONFIG if iscourse else BASE.parent/'race_follow_pair_v1/stop_restart.json'
    case='stop_restart_full_course' if iscourse else 'stop_restart'
    cfg=load_config(config)
    launch_arguments('g1g2',reserve,cfg['launch'],True,progress=True)
    entry=str(BASE/'course_entry.py') if iscourse else 'scripts/validate_g1_traffic.py'
    args=[entry,'--mode','g1g2','--handoff-reserve',reserve,'--handoff-yield-progress',
        '--config',str(config),'--case',case,'--seed','0','--record-follow-decisions','--output',str(output)]
    sources={}
    overlay=course.OVERLAY if iscourse else None
    if iscourse:
        request=SimpleNamespace(mode='g1g2',config=config,actor_overlay=overlay,seed=0,case=case,
            record_follow_decisions=True,output=output)
        sources.update(course.inputs(request,cfg,overlay_case(cfg['cases'][0],overlay)))
        args+=['--team-signal-course','--actor-overlay',str(overlay)]
    paths=set(source_paths(config,actor_overlay=overlay))
    paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    paths.update(p.relative_to(ROOT).as_posix() for p in (Path(__file__).resolve(),BASE/'course_entry.py',
        BASE/'course.launch.py',ENV,BASE/'replay_result.json',ROOT/'config/tests/yield_progress_r1_sha256.json'))
    sources.update({p:sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(paths)})
    return dict(args=args,source_sha256=sources,scope='현재 면의 상대전진 근거로만 양보 예산 갱신. 기존 센서·속도·물성·각본·평가선 유지.')

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('case',choices=('stop','course'))
    p.add_argument('--reserve',choices=('paced','closing'),default='paced')
    p.add_argument('--prepare-only',action='store_true');a=p.parse_args()
    name=a.case+'_'+a.reserve
    if (BASE/name).exists():raise FileExistsError('실행 원본 재사용 금지')
    reg=BASE/(name+'_registered.json');data=register(a.case,a.reserve)
    if a.prepare_only:
        with reg.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    else:
        if json.loads(reg.read_bytes())!=data:raise ValueError('등록 후 소스 변경')
        with (BASE/(name+'_console.log')).open('x') as f:
            r=subprocess.run(['bash',str(ENV),sys.executable,'-B',*data['args']],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        with (BASE/(name+'_exit.log')).open('x') as f:f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
