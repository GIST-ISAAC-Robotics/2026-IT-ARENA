"""등록한 입력의 해시와 출력 경로를 확인한 뒤 기존 교통 시험기를 그대로 실행."""
import subprocess
import sys
from prepare import HERE,ROOT,read,sha
def main(name):
    if not name.isidentifier():raise ValueError('run name required')
    folder=HERE/name;reg=read(folder/'registered.json');out=folder/'run1'
    if out.exists() or not all(sha(ROOT/p)==h for p,h in reg['sources'].items()):return 2
    args=[str(out) if v=='OUTPUT' else str(ROOT/v) if v.startswith(('src/','artifacts/')) else v for v in reg['args']]
    return subprocess.call([sys.executable,'-B',*args],cwd=ROOT)
if __name__=='__main__':sys.exit(main(sys.argv[1]))
