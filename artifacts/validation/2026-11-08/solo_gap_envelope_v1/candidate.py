"""운행 연결 없는 저장 입력 후보. 무반환 폭의 평가 반경만 실제 제동 외곽으로 대조."""
from pathlib import Path
import types

ROOT=Path(__file__).resolve().parents[4]
PATH=ROOT/'src/arena_autonomy/arena_autonomy/partial_observation.py'
source=PATH.read_text(encoding='utf8')
old='radius=max(.6,.16+speed*(now-source+.10)+speed*speed/4)'
new='''radius=max(
                (math.hypot(length/2+.05,width/2+.05)
                 +math.hypot(history.config.lidar_x_m,history.config.lidar_y_m)
                 +(1+max(abs(math.tan(steering)),abs(math.tan(previous_steering)))/2)
                   *(speed*(now-source+.10)+speed*speed/4+.06))
                    if self.oblique_metric_enabled else .6,
                .16+speed*(now-source+.10)+speed*speed/4)'''
assert source.count(old)==1
module=types.ModuleType('arena_autonomy.partial_observation_candidate')
module.__package__='arena_autonomy'
exec(compile(source.replace(old,new),str(PATH)+'_artifact_candidate','exec'),module.__dict__)
Candidate=module.PartialObservationPolicy


def recovery_policy():
    """회복/점별 시각 처리는 같은 소스 그대로, 기반 평가 반경만 교체한다."""
    path=ROOT/'src/arena_autonomy/arena_autonomy/traffic_recovery_trial.py'
    source=path.read_text(encoding='utf8')
    import_line='from arena_autonomy.partial_observation import PartialObservationPolicy'
    assert source.count(import_line)==1
    recovery=types.ModuleType('arena_autonomy.recovery_candidate')
    recovery.__package__='arena_autonomy'
    recovery.PartialObservationPolicy=Candidate
    exec(compile(source.replace(import_line,'# supplied candidate base'),str(path)+'_artifact_candidate','exec'),recovery.__dict__)
    return recovery.RecoveryPolicy
