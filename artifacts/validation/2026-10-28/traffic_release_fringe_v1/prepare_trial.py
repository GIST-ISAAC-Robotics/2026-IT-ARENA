"""직전155 s 각본을 그대로 쓰는 후속 실행 파일을 준비한다. 실행은 하지 않는다."""
from pathlib import Path
BASE = Path(__file__).resolve().parent
OLD = BASE.parent/'traffic_recovery_connection_v1'
for name in ('run1_snapshot.py', 'run1_trial.sh'):
    target = BASE/name
    if target.exists():
        raise FileExistsError(target)
    content = (OLD/name).read_text(encoding='utf-8')
    content = content.replace('artifacts/validation/2026-10-28/traffic_recovery_connection_v1',
                              'artifacts/validation/2026-10-28/traffic_release_fringe_v1')
    target.write_text(content, encoding='utf-8', newline='\n')
