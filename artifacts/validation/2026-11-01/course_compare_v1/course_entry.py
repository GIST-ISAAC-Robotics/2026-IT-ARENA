"""등록된 155초 코스를 기존 평가기로 실행한다. 출력 폴더와 진입점 경로만 이 폴더로 지정한다.

평가기는 대표 코스 출력을 mixed_course_v1 아래로 제한한다. 그 폴더의 기존 입력과 run1을 건드리지
않도록 평가기 소스는 바꾸지 않고 진입점에서만 지정한다. 코스·옆차·신호·판정·측정은 그대로다.
"""
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE.parents[3] / 'scripts'))
import race_traffic_course as course
import validate_g1_traffic as evaluator

course.BASE = BASE
course.ENTRY = BASE / 'course.launch.py'
raise SystemExit(evaluator.main())
