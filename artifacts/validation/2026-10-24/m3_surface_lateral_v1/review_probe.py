"""GPT 검토 반례의 거절 원인. 실행 결과는 오프라인 시제품의 제한 조건이다."""
import json
from pathlib import Path
import sys

ROOT = next(p for p in Path(__file__).resolve().parents if (p/'src/arena_autonomy').is_dir())
sys.path[:0] = [str(ROOT/'tests'), str(ROOT/'src/arena_autonomy')]
import test_lateral_intent as fixture
import arena_autonomy.lateral_intent as li

cases = {}
for name, d0, actor in [('left', -.09, (-.4, .13)), ('right', .09, (-.4, -.13)),
                         ('empty_offset', -.09, None), ('stopped', 0., (.9, 0.))]:
    extra = [] if actor is None else [fixture.car(*actor, d0)]
    scene, path = fixture.straight(d0, extra=extra)
    out, _ = fixture.run(scene, path)
    cases[name] = {k: v for k, v in out.items() if k not in ('path',)}
    if name == 'stopped':
        frame = li.PathFrame(path)
        state = li.ego_state(frame, fixture.GRASS)
        cand = li.bypass_candidate(frame, state, li.ObservationGrid(scene, fixture.GRASS).returns,
                                   'right', fixture.GRASS, stationary=True)
        check = li.surface_check(frame, cand, fixture.GRASS, current_pose=(0, 0, 0))
        cases[name]['bypass'] = {'target': cand['d_target_m'], 'problems': cand['problems'],
                                 'max_steering': cand['max_abs_steering_rad'],
                                 'surface': {k: v for k, v in check.items() if k not in ('wheel_l', 'wheel_s')}}

destination = Path(sys.argv[1])
destination.parent.mkdir(parents=True, exist_ok=True)
destination.write_text(json.dumps(cases, ensure_ascii=False, indent=2), encoding='utf-8')
for name, out in cases.items():
    print(name, out['mode'], out.get('reasons'))
    for key, check in out.get('checks', {}).items():
        print(' ', key, check['d_target_m'], check.get('problems'), check.get('reasons'),
              'unknown=', check.get('new_cells_unobserved'), 'hidden=', check.get('hidden_reach_cells'))
    if 'bypass' in out:
        print(' ', out['bypass'])
