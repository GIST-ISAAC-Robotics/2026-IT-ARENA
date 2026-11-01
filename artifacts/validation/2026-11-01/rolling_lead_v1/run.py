"""기존 출구와 차간격을 유지한 비교를 사전 등록하고 한 번 실행한다."""
import argparse
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from course import CONFIG, PARENT, expected, geometry
from validate_g1_traffic import source_paths

ENV=BASE.parent/'turn_preview_v1/environment.sh'


def registration():
    from race_traffic_course import inputs,entry_for_output
    request=SimpleNamespace(config=CONFIG,actor_overlay=None,mode='g1g2',seed=0,
                            record_follow_decisions=True,output=BASE/'run1',case='first_curve')
    cfg=json.loads(CONFIG.read_bytes())
    if cfg!=expected() or geometry(cfg)!=json.loads((BASE/'geometry.json').read_bytes()):
        raise ValueError('등록한 경로·기존 코너 판정과 불일치')
    files=inputs(request,cfg,cfg['cases'][0])
    entry=entry_for_output(request.output)
    paths=set(source_paths(CONFIG))
    paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    paths.update(p.relative_to(ROOT).as_posix() for p in (PARENT,entry,ENV,Path(__file__).resolve(),BASE/'course.py',BASE/'geometry.json',BASE/'measure.py'))
    files.update({p:sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(paths)})
    return dict(args=['scripts/validate_g1_traffic.py','--mode','g1g2','--handoff-reserve','closing','--team-signal-course',
                    '--config',str(CONFIG),'--case','first_curve','--seed','0','--record-follow-decisions','--output',str(request.output)],
        source_sha256=files,scope='1.5 m/s 상한 통합 제어의 단독 상대 차량 추종. 속도 단계는 평가기만 사용하며 실제 상대 동역학이 아니다.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--prepare-only',action='store_true')
    a=p.parse_args();reg=BASE/'registered.json'
    if (BASE/'run1').exists():raise FileExistsError('기존 실행을 덮어쓰지 않는다')
    data=registration()
    if a.prepare_only:
        with reg.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    else:
        if data!=json.loads(reg.read_bytes()):raise ValueError('등록 후 입력 변경')
        with (BASE/'run1_console.log').open('x') as f:
            r=subprocess.run(['bash',str(ENV),sys.executable,'-B',*data['args']],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        with (BASE/'run1_exit.log').open('x') as f:f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
