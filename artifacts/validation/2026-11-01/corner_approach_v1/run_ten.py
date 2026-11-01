"""완주한 2 m/s 실행과 조건을 맞춰 10 km/h 프로필을 한 번 실행한다(WSL)."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys


if __name__=='__main__':
    here=Path(__file__).resolve().parent
    root=here.parents[3]
    ref=json.loads((here/'solo_2p0_pre_manifest.json').read_text())
    old=ref['command_arguments']
    # shell 문자열을 재생하지 않는다. 등록한 인자 배열에서 검사기 경로부터 가져온다.
    start=next(i for i,v in enumerate(old) if str(v).replace('\\','/').endswith('scripts/validate_basic_autonomy.py'))
    args=old[start:]
    args[0]='scripts/validate_basic_autonomy.py'
    profile=args.index('--speed-profile')+1
    assert args[profile]=='g1_open_2p0_approach' and '--disable-speed-bump' in args and '--g1-race-path' in args
    args[profile]='g1_open_10kmh_approach'
    run=here/'solo_10kmh'
    assert args[-2]=='--output' and not run.exists()
    args[-1]=str(run)
    hashes={rel:hashlib.sha256((root/rel).read_bytes()).hexdigest() for rel in ref['source_sha256']}
    changed=[k for k,v in hashes.items() if ref['source_sha256'][k]!=v]
    pre=here/'solo_10kmh_pre_manifest.json'
    with pre.open('x') as f:
        json.dump(dict(scope='단독 10 km/h 상한; 변경된 소스와 이전 2.0 실행을 구분한다.',
            base=ref['reference_manifest'],reference='solo_2p0_pre_manifest.json',args=args,
            source_sha256=hashes,changes_relative_to_2p0=changed),f,indent=2,ensure_ascii=False)
    with (here/'solo_10kmh_console.log').open('x') as log:
        result=subprocess.run([sys.executable,'-B',*args],cwd=root,stdout=log,stderr=subprocess.STDOUT)
    sys.exit(result.returncode)
