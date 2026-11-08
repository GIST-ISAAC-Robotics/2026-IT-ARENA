from pathlib import Path
import importlib.util, sys
import pytest
A=Path(__file__).resolve().parent
name='arena_autonomy.piecewise_wall'
spec=importlib.util.spec_from_file_location(name,A/'piecewise_wall.py')
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
sys.modules[name]=module
raise SystemExit(pytest.main(['-q','tests/test_piecewise_wall.py',str(A/'test_piecewise_noise_staged.py')]))
