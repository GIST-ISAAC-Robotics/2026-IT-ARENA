"""단독 2 m/s 완주본과 같은 명령에서 속도 프로필만 바꾸는 시험."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
REF=BASE.parent/'corner_approach_v1/solo_2p0_pre_manifest.json'


def manifest():
    ref=json.loads(REF.read_text(encoding='utf-8'))
    args=ref['command_arguments']; args=args[args.index('scripts/validate_basic_autonomy.py'):]
    assert args[args.index('--speed-profile')+1]=='g1_open_2p0_approach'
    args[args.index('--speed-profile')+1]='g1_open_2p0_turn'
    assert args[-2]=='--output'; args[-1]=str(BASE/'run1')
    changes=sorted(set(ref['source_sha256'])|{'src/arena_autonomy/arena_autonomy/turn_preview.py',
        'scripts/build_experimental_track.py',Path(__file__).resolve().relative_to(ROOT).as_posix()})
    sha=lambda p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest()
    return dict(args=args,source_sha256={k:sha(k) for k in changes},reference=REF.relative_to(ROOT).as_posix(),
                scope='같은 임시 신호 2 m/s 비교, 명시적 턴 속도 프로필만 선택; 실차·교통 판정 아님')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepare-only',action='store_true')
    opts=p.parse_args(); artifact=BASE/'registered.json'; data=manifest()
    if opts.prepare_only:
        if (BASE/'run1').exists():raise FileExistsError('기존 실행을 다시 등록하지 않는다')
        with artifact.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    else:
        if data!=json.loads(artifact.read_text(encoding='utf-8')) or (BASE/'run1').exists():
            raise ValueError('새 출력과 등록 입력이 필요하다')
        with (BASE/'run1_console.log').open('x') as log:
            result=subprocess.run([sys.executable,'-B',*data['args']],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        raise SystemExit(result.returncode)
