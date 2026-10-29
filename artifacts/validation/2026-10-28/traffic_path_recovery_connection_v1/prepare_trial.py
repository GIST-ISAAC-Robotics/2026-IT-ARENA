"""같은155 s 각본에 새 선택 옵션 하나만 추가한다. 이 파일은 주행하지 않는다."""
from pathlib import Path
BASE=Path(__file__).resolve().parent
OLD=BASE.parent/'traffic_release_fringe_v1'
for name in ('run1_snapshot.py','run1_trial.sh'):
    target=BASE/name
    if target.exists():raise FileExistsError(target)
    content=(OLD/name).read_text(encoding='utf-8').replace(
        'artifacts/validation/2026-10-28/traffic_release_fringe_v1',
        'artifacts/validation/2026-10-28/traffic_path_recovery_connection_v1')
    if name.endswith('.sh'):
        assert content.count('--traffic-recovery-trial --actor-overlay')==1
        content=content.replace('--traffic-recovery-trial --actor-overlay',
                                '--traffic-recovery-trial --traffic-path-recovery-trial --actor-overlay')
    target.write_text(content,encoding='utf-8',newline='\n')
