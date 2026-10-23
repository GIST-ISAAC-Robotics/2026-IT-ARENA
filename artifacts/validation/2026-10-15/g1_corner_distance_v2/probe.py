"""v1를 재사용하되 새 경계 실패 상세와 현재 wrapper를 별도 보존한다."""
import importlib.util
from pathlib import Path
import zipfile

BASE=Path(__file__).resolve().parent
old=BASE.parent/'g1_corner_distance_v1/probe.py'
spec=importlib.util.spec_from_file_location('corner_probe_v1',old)
module=importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
module.BASE=BASE
module.main()
with zipfile.ZipFile(BASE/'wrapper_source.zip','x',zipfile.ZIP_DEFLATED) as archive:
    archive.write(__file__,'probe.py')
