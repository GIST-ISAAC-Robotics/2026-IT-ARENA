"""r328 저장 입력에서 공통 연결 수정의 최초 차이와 기존 정지 구간을 확인한다."""
import importlib.util
import json
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('r330_replay', A.parent/'old610_corner_r330_v1/replay.py')
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)
m.A = A
sys.argv = [__file__, 'common']
m.main()
rows = json.loads((A/'common_rows.json').read_text())
changed = [r for r in rows if r['command_changed']]
window = [r for r in rows if 31.04 <= r['time'] <= 31.34]
summary = dict(first_change=changed[0] if changed else None,
               previous_stop_window=window,
               continued_calls_after_original_stop=sum(r['speed'] > 0 and r['time'] >= 31.14 for r in rows),
               scope='same recorded trajectory, not closed-loop driving')
(A/'transition.json').write_text(json.dumps(summary, indent=2)+'\n')
print(json.dumps(summary, indent=2))
