import cProfile, pstats, sys
from pathlib import Path
sys.path.insert(0, 'artifacts/validation/2026-10-24/m3_surface_lateral_v1')
import screen_lateral as sl
motion, follow, calls = sl.load_inputs(sl.RECORDS['e3_onroad_g1'])
cfg = sl.variants()['grass_hard']
sel = [c for c in calls if c['kind'] == 'call' and c['data']['input']['reference_s'] and 4.0 <= c['data']['input']['reference_s'] <= 4.4]
pr = cProfile.Profile(); pr.enable()
rows = sl.run_record('x', cfg, motion, follow, sel)
pr.disable()
print(len(rows), [round(r['step_ms'], 1) for r in rows[:5]])
pstats.Stats(pr).sort_stats('cumtime').print_stats(18)
