"""저장 입력 비교 전용. 기존 코드 파일/기본 정책을 수정하지 않는 사선 공백 후보."""
import inspect
import textwrap
from arena_autonomy import partial_observation as module


def install():
    original=module.PartialObservationPolicy.evaluate
    source=textwrap.dedent(inspect.getsource(original))
    assert source.count('math.radians(60)')==2
    source=source.replace('math.radians(60)','math.radians(30)')
    source=source.replace('minimum_angle_deg=60.','minimum_angle_deg=30.')
    namespace=dict(vars(module))
    exec(compile(source,'<saved-input-oblique-gap-candidate>','exec'),namespace)
    module.PartialObservationPolicy.evaluate=namespace['evaluate']
    return original
