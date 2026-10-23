"""v1의 입력/코드를 보존하고 동일 네 스냅샷에 수정 증거 함수만 대조한다."""
import importlib.util
from pathlib import Path

BASE = Path(__file__).resolve().parent
spec = importlib.util.spec_from_file_location('saved_probe', BASE.parent/'g1_corner_evidence_v1/probe.py')
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)
probe.BASE = BASE
probe.main()
