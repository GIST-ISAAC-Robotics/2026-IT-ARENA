"""등록 해시가 그대로인지 확인한 뒤 기존 교통 검사기를 등록 인자 그대로 실행한다."""
import json
import hashlib
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def main(case):
    folder = HERE/case
    registered = json.loads((folder/'registered.json').read_text(encoding='utf-8'))
    output = folder/'run2'
    changed = [p for p, h in registered['sources'].items() if hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != h]
    if output.exists() or changed:
        print('refused: existing output or changed sources', changed[:10])
        return 2
    args = [str(output) if value == 'OUTPUT' else value for value in registered['args']]
    return subprocess.call([sys.executable, '-B', *args], cwd=ROOT)


if __name__ == '__main__':
    sys.exit(main(sys.argv[1]))
