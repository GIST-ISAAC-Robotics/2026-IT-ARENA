"""등록된 파생 각본을 기존 평가기로 실행한다. 기준 각본 경로만 이 등록 파일로 명시 지정한다.

평가기는 G1/G2 비교를 원래 각본으로 제한한다. 이 시험은 그 비교가 아니라 후보가 새로 노출되는
빠른 앞차 급정지를 재는 별도 등록 시험이므로, 평가기 소스는 바꾸지 않고 진입점에서만 지정한다.
판정·측정·보호층·제어 코드는 평가기 그대로다.
"""
from pathlib import Path
import sys

BASE = Path(__file__).resolve().parent
sys.path.insert(0, str(BASE.parents[3] / 'scripts'))
import validate_g1_traffic as evaluator

evaluator.CONFIG = BASE / 'fast_stop.json'
raise SystemExit(evaluator.main())
