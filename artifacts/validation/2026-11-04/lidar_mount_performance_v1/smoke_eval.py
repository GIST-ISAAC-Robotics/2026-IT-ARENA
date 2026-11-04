"""평가기 동작 확인: 11/3 기존 실행에 같은 집계를 적용한다(출력만, 파일 미작성)."""
import importlib.util, json
from pathlib import Path
HERE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('ev', HERE/'evaluate.py')
ev = importlib.util.module_from_spec(spec); spec.loader.exec_module(ev)
ev.HERE = HERE.parents[1]/'2026-11-03/solo_post_free_v1'
print(json.dumps(ev.solo('run1'), default=str)[:1500])
ev.HERE = HERE.parents[1]/'2026-11-03/right_reapproach_v3'
r = ev.traffic('run1'); r.pop('actor_scan_rows', None); print(json.dumps(r, default=str)[:2000])
