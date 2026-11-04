"""수정된 검출기의 기본 수평 차량 실제 출발 확인. 그리드 3에서 짧은 거리만 달린다. 랩 시간 근거가 아니다."""
from hashlib import sha256
import json
from pathlib import Path
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]
BASE = HERE.parent/'official_facility_v1'
ENV = BASE.parents[1]/'2026-11-01/turn_preview_v1/environment.sh'
OUTPUT = HERE/'start1'
SOURCES = ('src/arena_autonomy/arena_autonomy/race_start.py', 'src/arena_autonomy/arena_autonomy/race_start_local.py',
           'src/arena_autonomy/arena_autonomy/vision_process.py', 'src/arena_bringup/launch/simulation.launch.py',
           'src/arena_gazebo/scripts/traffic_light_controller.py', 'src/arena_gazebo/worlds/it_arena_official/world.sdf')


def main():
    if OUTPUT.exists():
        raise FileExistsError('기존 실행')
    args = json.loads((BASE/'run1_registered.json').read_bytes())['args']
    index = args.index('--video')
    args = [*args[:index], *args[index+1:]]
    laps = args.index('--laps')
    args[laps:laps+2] = ['--laps', '1', '--target-progress-m', '6']
    args[-1] = str(OUTPUT)
    record = dict(args=args, source_sha256={p: sha256((ROOT/p).read_bytes()).hexdigest() for p in SOURCES},
                  scope='출발 확인 전용 6 m. 영상 없음. 기본 수평 차량.')
    (HERE/'start1_registered.json').write_text(json.dumps(record, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    with (HERE/'start1_console.log').open('x') as stream:
        result = subprocess.run(['bash', str(ENV), sys.executable, '-B', *args], cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
    (HERE/'start1.exit').write_text(f'ACTUAL_EXIT_CODE={result.returncode}\n', encoding='utf-8')
    raise SystemExit(result.returncode)


if __name__ == '__main__':
    main()
