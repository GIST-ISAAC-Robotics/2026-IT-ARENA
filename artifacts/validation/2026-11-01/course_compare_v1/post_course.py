"""대표 코스의 기존 명령 분류와 고밀도 진행 재집계를 새 실행에 같은 도구로 적용한다."""
import json
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE.parent / 'mixed_course_v1'))
from command_time import analyze
from dense_progress import audit

RUN = BASE / 'run_paced'
for name, value in (('run_paced_command_time.json', analyze(RUN)), ('run_paced_dense_progress.json', audit(RUN))):
    with (BASE / name).open('x', encoding='utf-8') as f:
        json.dump(value, f, ensure_ascii=False, indent=2)
print('done')
