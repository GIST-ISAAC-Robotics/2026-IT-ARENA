import common_probe as probe
import json,sys
import pytest
targets=sys.argv[1:] or [str(probe.A/'test_r494.py'),str(probe.ROOT/'tests/test_moving_pass_current_return.py')]
code=pytest.main(['-q','-p','no:cacheprovider',*targets])
(probe.A/'test_receipt.json').write_text(json.dumps(dict(exit_code=code,
    numpy=probe.np.__version__,two_dimensional_cross_compatibility_calls=probe.cross_calls,
    production_numpy_unchanged=True,device_execution=False,targets=targets),indent=2),encoding='utf-8')
raise SystemExit(code)
