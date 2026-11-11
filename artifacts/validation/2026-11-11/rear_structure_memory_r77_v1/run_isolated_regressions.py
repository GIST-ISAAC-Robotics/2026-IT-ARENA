"""실제 런타임 파일을 바꾸지 않고 기존 구조/부분 관측 검사에 후보 mixin을 연결한다."""
import importlib.util
from pathlib import Path
import pytest
import arena_autonomy.known_structure as runtime
HERE=Path(__file__).resolve().parent
spec=importlib.util.spec_from_file_location('r77_regression_candidate',HERE/'known_structure_candidate.py')
candidate=importlib.util.module_from_spec(spec);spec.loader.exec_module(candidate)
runtime.StructureAwareMixin=candidate.StructureAwareMixin
raise SystemExit(pytest.main(['-q',
    'tests/test_known_structure_policy.py','tests/test_structure_route_speed.py',
    'tests/test_partial_observation.py','tests/test_g1_partial_policy.py',
    'tests/test_g1_partial_traffic.py','tests/test_g1_guard_partial_coupling.py',
    'tests/test_recovery_threat_capture.py','tests/test_partial_lateral_metric.py',
    'tests/test_forward_metric_gap.py','tests/test_oblique_sweep_radius.py',
    'tests/test_native_scan_pipeline.py',str(HERE/'test_candidate.py')]))
