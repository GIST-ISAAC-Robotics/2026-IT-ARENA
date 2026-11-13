"""네이티브 빌드의 실제 ARM import를 강제한다. 설정/장치 변경 없음."""
from pathlib import Path
import json,os,sys
import numpy
from arena_autonomy import numeric_kernels,_numeric_kernels
numeric_kernels.configure('native_v1')
root=Path(__file__).resolve().parent
assert Path(_numeric_kernels.__file__).resolve().is_relative_to(root/'src/arena_autonomy')
assert hasattr(_numeric_kernels,'masked') and hasattr(_numeric_kernels,'classes')
report=dict(machine=os.uname().machine,python=sys.version,numpy=numpy.__version__,
    native_path=_numeric_kernels.__file__,abi=_numeric_kernels.ABI_VERSION,classification=numeric_kernels.classification_enabled(),
    settings_changed=False,devices_opened=False)
(root/'import_preflight.json').write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
