"""등록 해시·실행 환경이 그대로인지 확인한 뒤 등록 인자 그대로 기존 교통 검사기를 실행한다. 사용: run.py A|B"""
import hashlib, json, subprocess, sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def main(key):
    registered = json.loads((HERE/'registered.json').read_text(encoding='utf-8'))
    run = registered['runs'][key]
    output = ROOT/run['folder']/'run1'
    changed = [p for p, h in registered['sources'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
    if output.exists() or changed or hashlib.sha256((HERE/'source_snapshot.zip').read_bytes()).hexdigest() != registered['snapshot_sha256']:
        print('refused: existing output, changed sources or snapshot', changed[:10])
        return 2
    other = [k for k in registered['runs'] if k != key]
    assert all(ROOT/registered['runs'][k]['folder'] != ROOT/run['folder'] for k in other)
    if subprocess.call([sys.executable, '-B', str(HERE/'check_runtime.py'), key], cwd=ROOT) != 0:
        print('refused: runtime executables do not resolve to registered sources')
        return 2
    args = [str(output) if value == 'OUTPUT' else value for value in run['args']]
    return subprocess.call([sys.executable, '-B', *args], cwd=ROOT)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
