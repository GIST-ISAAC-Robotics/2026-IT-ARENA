"""기존 fail-fast batch에 명시적인 solver wrapper 실행 명령만 전달한다."""
from pathlib import Path
import sys
from run_probe_batch import load_plan, run_plan

base = Path(__file__).resolve().parent
name = sys.argv[1]
if name not in ('d1_dantzig', 'p1_pgs'):
    raise SystemExit('unregistered case')
if (base/f'status_{name}.json').exists():
    raise SystemExit('status exists; refuse overwriting')
result = run_plan(load_plan(base/f'plan_{name}.json'), base/f'status_{name}.json',
                  command=[sys.executable, '-B', 'scripts/probe_solver_compare.py'])
raise SystemExit(result)
