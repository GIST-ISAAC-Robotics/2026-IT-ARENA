"""증속률 후보를 현재 2×5 신호에서 실행한다. 과거 3등과의 차이는 따로 보존한다."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
PRIOR=BASE.parent/'corner_approach_v1/solo_10kmh_pre_manifest.json'
ENV=BASE.parent/'turn_preview_v1/environment.sh'
sys.path.insert(0,str(ROOT/'scripts'))
from prelap_signal_eval import fixture_inputs


def registration():
    entry=ROOT/'src/arena_bringup/launch/simulation.launch.py'
    spec=importlib.util.spec_from_file_location('speed_profile_compare',entry)
    launch=importlib.util.module_from_spec(spec);spec.loader.exec_module(launch)
    name='g1_open_10kmh_rise15';original='g1_open_10kmh_approach'
    old=launch.SPEED_PROFILES[original];new=launch.SPEED_PROFILES[name]
    if old['acceleration_mps2']!=1. or new!={**old,'acceleration_mps2':1.5}:
        raise ValueError('등록된 변경은 명령 증속률 1→1.5 m/s² 하나')
    config=json.loads(PRIOR.read_text(encoding='utf-8'));args=list(config['args'])
    ix=args.index('--speed-profile')+1
    assert args[ix]==original and '--disable-speed-bump' in args and args[-2]=='--output'
    args[ix]=name
    args=[*args[:-2],'--team-signal-prelap','--video','--output',str(BASE/'run1')]
    files=set(config['source_sha256'])
    files.update(p.relative_to(ROOT).as_posix() for p in (PRIOR,ENV,Path(__file__).resolve()))
    files.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    hashes=fixture_inputs()
    hashes.update({k:hashlib.sha256((ROOT/k).read_bytes()).hexdigest() for k in sorted(files)})
    return dict(args=args,baseline=old,trial=new,source_sha256=hashes,
        scope='계획·발행 제어는 증속률만 바뀌지만 실행 시설은 2×5와 촬영으로 이전한다. 3등 주행 대비 효과 원인을 증속률 하나로 단정하지 않는다.')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepare-only',action='store_true')
    opts=p.parse_args();data=registration();reg=BASE/'registered.json'
    if (BASE/'run1').exists():raise FileExistsError('기존 출력이 있다')
    if opts.prepare_only:
        with reg.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    else:
        if data!=json.loads(reg.read_text(encoding='utf-8')):raise ValueError('사전 등록 이후 변경')
        with (BASE/'run1_console.log').open('x') as f:
            r=subprocess.run(['bash',str(ENV),sys.executable,'-B',*data['args']],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        with (BASE/'run1_exit.log').open('x') as f:f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
