"""2×5 증속 시험과 같은 조건에서 코너 보완 하나만 더한 프리랩."""
import argparse
from hashlib import sha256
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
from zipfile import ZipFile

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from prelap_signal_eval import fixture_inputs

RISE=HERE.parent/'race_acceleration_v1'
ENV=HERE.parent/'turn_preview_v1/environment.sh'


def register():
    prior=json.loads((RISE/'registered.json').read_bytes())
    oldrun=json.loads((RISE/'run1/report.json').read_bytes())
    if not (oldrun['passed'] and oldrun['source_unchanged'] and oldrun['stop_stable']
            and oldrun['shutdown_clean'] and oldrun['team_signal_evidence']['passed']):
        raise ValueError('증속 시험이 유효하게 끝나지 않아 결합 비교를 시작하지 않는다')
    spec=importlib.util.spec_from_file_location('registered_speed_profiles',ROOT/'src/arena_bringup/launch/simulation.launch.py')
    mod=importlib.util.module_from_spec(spec);spec.loader.exec_module(mod)
    old=mod.SPEED_PROFILES['g1_open_10kmh_rise15'];name='g1_open_10kmh_turn15';new=mod.SPEED_PROFILES[name]
    if new!={**old,'g1_preview_turn_speed':True} or old!=prior['trial']:
        raise ValueError('비교 구성 변경은 코너 미리 보기 하나여야 한다')
    args=list(prior['args']);i=args.index('--speed-profile')+1
    if args[i]!='g1_open_10kmh_rise15' or args[-2:]!=['--output',str(RISE/'run1')]:
        raise ValueError('기준 실행 인자 변경')
    args[i]=name;args[-1]=str(HERE/'run1');files=set(prior['source_sha256'])
    files.update(p.relative_to(ROOT).as_posix() for p in (
        Path(__file__).resolve(),RISE/'registered.json',RISE/'run1/report.json',
        HERE/'combined_replay.json',HERE/'reference_input.json',ENV))
    sources=fixture_inputs();sources.update({p:sha256((ROOT/p).read_bytes()).hexdigest() for p in sorted(files)})
    changed={p for p,h in prior['source_sha256'].items() if sources[p]!=h}
    if changed!={'src/arena_bringup/launch/simulation.launch.py','scripts/validate_basic_autonomy.py'}:
        raise ValueError(f'기준 시험에서 등록/실행 연결 외 입력이 달라짐: {sorted(changed)}')
    archive=HERE/'before.zip'
    with ZipFile(archive) as z:
        if any(sha256(z.read(p)).hexdigest()!=prior['source_sha256'][p] for p in changed):
            raise ValueError('변경 전 소스가 증속 기준 시험과 다름')
    sources[archive.relative_to(ROOT).as_posix()]=sha256(archive.read_bytes()).hexdigest()
    return dict(args=args,source_sha256=sources,baseline=old,trial=new,changed_inputs=sorted(changed),
                scope='같은 2×5·영상·10 km/h 상한·명령 증속률 1.5에서 코너 보완만 비교하는 단독 1회.')


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--prepare-only',action='store_true');a=p.parse_args()
    if (HERE/'run1').exists():raise FileExistsError('기존 실행')
    data=register();reg=HERE/'registered.json'
    if a.prepare_only:
        with reg.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    else:
        if data!=json.loads(reg.read_bytes()):raise ValueError('등록 후 입력 변경')
        with (HERE/'run1_console.log').open('x') as f:
            r=subprocess.run(['bash',str(ENV),sys.executable,'-B',*data['args']],cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        with (HERE/'run1_exit.log').open('x') as f:f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
