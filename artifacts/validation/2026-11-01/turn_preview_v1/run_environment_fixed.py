"""run1 센서 초기화 실패 이후, 검증된 셸로만 별도 run2에 원래 시험을 실행한다."""
import argparse
import hashlib
import json
import subprocess
import sys
from pathlib import Path
from run import BASE,ROOT,manifest


def registration():
    data=manifest()
    old=json.loads((BASE/'registered.json').read_text(encoding='utf-8'))
    if data!=old:raise ValueError('차량·센서·평가 입력이 run1과 달라짐')
    data['args']=[*data['args'][:-1],str(BASE/'run2')]
    data['source_sha256']=dict(data['source_sha256'])
    for p in (BASE/'environment.sh',Path(__file__).resolve(),BASE/'run1/report.json'):
        data['source_sha256'][p.relative_to(ROOT).as_posix()]=hashlib.sha256(p.read_bytes()).hexdigest()
    return data


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--prepare-only',action='store_true')
    a=p.parse_args();reg=BASE/'run2_registered.json';data=registration()
    if (BASE/'run2').exists():raise FileExistsError('기존 실행 덮어쓰기 금지')
    if a.prepare_only:
        with reg.open('x',encoding='utf-8') as f:json.dump(data,f,ensure_ascii=False,indent=2)
    else:
        if json.loads(reg.read_text(encoding='utf-8'))!=data:raise ValueError('실행 전 등록과 다름')
        with (BASE/'run2_console.log').open('x') as out:
            r=subprocess.run(['bash',str(BASE/'environment.sh'),sys.executable,'-B',*data['args']],
                             cwd=ROOT,stdout=out,stderr=subprocess.STDOUT)
        with (BASE/'run2_exit.log').open('x') as f:f.write(f'ACTUAL_EXIT_CODE={r.returncode}\n')
        raise SystemExit(r.returncode)
