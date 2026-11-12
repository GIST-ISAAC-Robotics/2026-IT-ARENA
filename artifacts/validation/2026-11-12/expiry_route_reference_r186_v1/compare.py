"""r182 원439판단과 선택 벽을 묶은 만료 참조 후보의 저장 입력 대조."""
import importlib.util
from pathlib import Path
import candidate

A=Path(__file__).resolve().parent
source=candidate.install()
with (A/'modified_baseline_command.py').open('x',encoding='utf8') as stream:stream.write(source)
path=A.parent/'corner_departure_r185_v1/replay_reset.py'
spec=importlib.util.spec_from_file_location('r186_comparison',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
module.A=A
module.retain_reference_reset=candidate.reset_reference
module.main()
