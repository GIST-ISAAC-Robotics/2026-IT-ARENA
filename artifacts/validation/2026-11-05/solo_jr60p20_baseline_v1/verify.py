"""등록 입력 해시와 진입점 인자를 다시 확인한다. 사용: verify.py before|after (WSL python3 -I -B)."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]


def main(stage):
    registered = json.loads((HERE/'registered.json').read_bytes())
    changed = [p for p, d in registered['input_sha256'].items()
               if not (ROOT/p).is_file() or hashlib.sha256((ROOT/p).read_bytes()).hexdigest() != d]
    args = json.loads(subprocess.run([sys.executable, '-I', '-B', 'scripts/run_current_solo.py',
                                      registered['entry']['output'], '--print-args'],
                                     cwd=ROOT, check=True, capture_output=True, text=True).stdout)
    result = dict(stage=stage, inputs=len(registered['input_sha256']), changed=changed,
                  print_args_match=args == registered['entry']['print_args'])
    print(json.dumps(result, ensure_ascii=False))
    return 0 if not changed and result['print_args_match'] else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1]))
