"""원/선택 정책의 이전 r187 전체 출력과 대조한다."""
import importlib.util
from pathlib import Path

A=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('preserved_r190_replay',A.parent/'guard_policy_module_r190_v1/replay.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.A=A
module.main()
