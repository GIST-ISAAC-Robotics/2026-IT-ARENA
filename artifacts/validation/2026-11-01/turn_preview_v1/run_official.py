"""임시 신호의 적용 실패를 반복하지 않고, 성공한 2×5 프리랩과 같은 코스로 턴 후보를 비교."""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path

BASE=Path(__file__).resolve().parent;ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from prelap_signal_eval import fixture_inputs


def registration():
    baseline=BASE.parent/'official_prelap_v1/registered2.json'
    old=json.loads(baseline.read_text(encoding='utf-8'))
    args=list(old['args']);ix=args.index('--speed-profile')+1
    assert args[ix]=='g1_open_2p0_approach' and '--team-signal-prelap' in args and '--video' in args
    assert args[-2]=='--output'
    args[ix]='g1_open_2p0_turn';args[-1]=str(BASE/'run4')
    hashes=fixture_inputs()
    files=set(old['source_sha256'])
    files.update(p.relative_to(ROOT).as_posix() for p in
                 (baseline,BASE/'environment.sh',Path(__file__).resolve()))
    files.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    hashes.update({k:hashlib.sha256((ROOT/k).read_bytes()).hexdigest() for k in sorted(files)})
    return dict(args=args,source_sha256=hashes,reference=baseline.relative_to(ROOT).as_posix(),
                scope='2×5 신호·3인칭 영상 47.143초 프리랩과 동일한 조건, 코너 속도 후보만 명시 선택')


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepare-only',action='store_true')
    opts=p.parse_args();reg=BASE/'run4_registered.json';data=registration()
    if (BASE/'run4').exists():raise FileExistsError('이미 수행한 출력')
    if opts.prepare_only:
        with reg.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    else:
        if data!=json.loads(reg.read_text(encoding='utf-8')):raise ValueError('등록 뒤 입력 변경')
        with (BASE/'run4_console.log').open('x') as f:
            r=subprocess.run(['bash',str(BASE/'environment.sh'),sys.executable,'-B',*data['args']],
                cwd=ROOT,stdout=f,stderr=subprocess.STDOUT)
        with (BASE/'run4_exit.log').open('x') as f:f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
