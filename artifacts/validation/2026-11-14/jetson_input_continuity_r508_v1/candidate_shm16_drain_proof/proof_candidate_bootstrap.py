"""r500 파일 불변; 이 시험의 프로세스 안에서만 명시 후보를 읽는다."""
import hashlib
from pathlib import Path
from arena_autonomy.race_runtime_policy import current_route_priority as module
path=Path(__file__).with_name('current_route_priority.py')
assert hashlib.sha256(Path(module.__file__).read_bytes()).hexdigest()=='23adb7767de1f64b4892a19956b33adae0dbb08a36cb9de86e8ff51891992651'
exec(compile(path.read_bytes(),str(path),'exec'),module.__dict__)
CANDIDATE_SHA=hashlib.sha256(path.read_bytes()).hexdigest()
