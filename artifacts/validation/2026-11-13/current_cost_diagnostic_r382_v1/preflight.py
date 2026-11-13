"""번들 import/전체 입력 해시만 확인한다. rebase·주행·센서 수신을 실행하지 않는다."""
import argparse
import json
import platform
from pathlib import Path
import sys
import time

from support import A, load_comparison, verify_manifest, check_imports


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--backend', choices=('python', 'native_v1'), default='native_v1')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    assert not args.output.exists(), 'do not overwrite preflight'
    began = time.perf_counter_ns()
    manifest = verify_manifest()
    compare = load_comparison()
    imported = check_imports(compare, args.backend)
    result = dict(passed=True, backend=args.backend, manifest_file_count=len(manifest['files']),
        python=sys.version, machine=platform.machine(), system=platform.system(),
        import_and_integrity_wall_ms=(time.perf_counter_ns()-began)/1e6,
        imports=imported, rebase_calls=0, devices_opened=False, native_built_by_preflight=False,
        scope='import/해시 사전 검사이며 계산 비용/출력 동등성/명령 유효성 검사가 아니다.')
    with args.output.open('x', encoding='utf8') as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2); stream.write('\n')
    print(json.dumps(dict(passed=True, backend=args.backend, files=len(manifest['files']),
                         candidate_chain=len(imported['candidate_chain']), rebase_calls=0)))


if __name__ == '__main__': main()
