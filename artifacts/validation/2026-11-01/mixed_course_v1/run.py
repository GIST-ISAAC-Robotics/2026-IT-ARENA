"""같은 시험을 다른 소스·각본으로 실수로 덮어쓰지 않도록 등록 후 한 번 실행한다."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from race_traffic_course import CONFIG, OVERLAY, inputs
from traffic_actors import overlay_case
from validate_g1_traffic import source_paths


def register(reserve):
    from types import SimpleNamespace
    output=BASE/'run1'; case='stop_restart_full_course'
    cfg=json.loads(CONFIG.read_text(encoding='utf-8'))
    request=SimpleNamespace(mode='g1g2',config=CONFIG,actor_overlay=OVERLAY,seed=0,case=case,
                            record_follow_decisions=True,output=output)
    sources=inputs(request,cfg,overlay_case(cfg['cases'][0],OVERLAY))
    paths=set(source_paths(CONFIG,actor_overlay=OVERLAY))
    paths.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    paths.update([Path(__file__).resolve().relative_to(ROOT).as_posix(),'scripts/g1_course_eval.py',
                  (BASE.parent/'turn_preview_v1/environment.sh').relative_to(ROOT).as_posix()])
    sources.update({k:hashlib.sha256((ROOT/k).read_bytes()).hexdigest() for k in paths})
    args=['scripts/validate_g1_traffic.py','--mode','g1g2','--handoff-reserve',reserve,
          '--team-signal-course','--config',str(CONFIG),'--actor-overlay',str(OVERLAY),'--case',case,
          '--seed','0','--record-follow-decisions','--output',str(output)]
    return dict(args=args,source_sha256=sources,scope='규정 신호 형상·1.5 m/s·앞차/왼쪽 차·155초. 독립 3대 경기 아님.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reserve',choices=('fixed','moving','closing'),required=True)
    p.add_argument('--prepare-only',action='store_true')
    opts=p.parse_args(); data=register(opts.reserve); reg=BASE/'registered.json'
    if (BASE/'run1').exists():raise FileExistsError('이미 실행한 출력')
    if opts.prepare_only:
        with reg.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    else:
        if data!=json.loads(reg.read_text(encoding='utf-8')):raise ValueError('등록 후 소스·각본 변경')
        with (BASE/'run1_console.log').open('x') as log:
            result=subprocess.run([sys.executable,'-B',*data['args']],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        raise SystemExit(result.returncode)
