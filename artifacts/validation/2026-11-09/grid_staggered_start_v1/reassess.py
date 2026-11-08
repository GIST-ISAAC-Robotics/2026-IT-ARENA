"""등록된 assess.py를 바꾸지 않고, run1/report.json 대신 재평가 report만 읽게 하여 보충 판정을 다시 집계한다.
새 주행이 아니다. 나머지 원시 기록(trace·자세·접촉·노면·무결성·프로세스·런타임 확인)은 원 실행 파일을 그대로 쓴다."""
from pathlib import Path
import sys

import assess

A = Path(__file__).resolve().parent
REPORT = A/'run1/report.json'
REEVALUATED = A/'reeval/report_reevaluated.json'
original_read = assess.read
assess.read = lambda p: original_read(REEVALUATED if Path(p) == REPORT else p)
assert not (A/'reeval/assessment_reevaluated.json').exists()
sys.exit(assess.main(A/'run1', A/'surface_run1.json', A/'postflight_integrity_run1.json',
                     A/'postflight_processes_run1.txt', A/'geometry.json', A/'reeval/assessment_reevaluated.json',
                     A/'runtime_check.json'))
