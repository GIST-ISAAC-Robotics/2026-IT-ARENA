"""검사기 main을 그대로 실행하되 `ros2 launch` 직전에 argv만 저장하고 즉시 종료한다(Gazebo 미실행).

사용: python3 -B dryargs_runner.py <output_dir> <validate_g1_traffic 인자...(--output 제외)>
종료 코드 0은 launch argv 회수, 3은 launch 전 검사기 거부(ValueError 등)다.
"""
import json
import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT/'scripts'))


def main():
    output, *arguments = sys.argv[1:]
    original = subprocess.Popen

    def capture(command, *args, **kwargs):
        if list(command[:2]) == ['ros2', 'launch']:
            Path(output, 'dry_launch_argv.json').write_text(json.dumps(list(command), indent=1)+'\n', encoding='utf-8')
            sys.stdout.flush()
            os._exit(0)
        return original(command, *args, **kwargs)
    subprocess.Popen = capture
    import validate_g1_traffic as harness
    sys.argv = ['validate_g1_traffic.py', *arguments, '--output', output]
    try:
        harness.main()
    except (ValueError, SystemExit) as error:
        print(f'REFUSED {type(error).__name__}: {error}')
        return 3
    print('launch was not reached')
    return 1


if __name__ == '__main__':
    raise SystemExit(main())
