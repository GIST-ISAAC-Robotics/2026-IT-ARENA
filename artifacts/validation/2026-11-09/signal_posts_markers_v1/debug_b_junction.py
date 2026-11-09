import json, sys, numpy as np
from replay_tools import HERE, build_controller, read_events
import arena_autonomy.local_follow as lf
from arena_autonomy.follow_trace import freeze, replay_call
FLAGS = dict(narrow_occluder_bridge=True, junction_continuity=True)
WIN = [float(a) for a in sys.argv[1:3]]
orig_agree, orig_suff = lf.path_agrees, None
log = []
def agree(a, b, tol):
    r = orig_agree(a, b, tol)
    if CUR[0] is not None and WIN[0] <= CUR[0] <= WIN[1]:
        pa = np.asarray(a); pb = np.asarray(b)
        log.append(dict(t=CUR[0], result=r, a_x=[round(float(pa[:,0].min()),2), round(float(pa[:,0].max()),2)], b_x=[round(float(pb[:,0].min()),2), round(float(pb[:,0].max()),2)],
                        a_y0=round(float(pa[np.argmin(np.abs(pa[:,0]-.3)),1]),3), b_y0=round(float(pb[np.argmin(np.abs(pb[:,0]-.3)),1]),3)))
    return r
lf.path_agrees = agree
CUR = [None]
c = mc = None
for e in read_events(HERE/'b_solo_posts/follow_decisions.jsonl'):
    d = e['data']
    if e['kind'] == 'header':
        c, mc = build_controller(dict(d, follow_config={**d['follow_config'], **FLAGS})); continue
    if e['kind'] == 'reset': c.reset(); continue
    if e['kind'] != 'call': continue
    CUR[0] = d['input']['reference_s']
    out = freeze(replay_call(c, d['input'], mc))
    if WIN[0] <= CUR[0] <= WIN[1]:
        det = out[2]
        print(round(CUR[0],2), round(out[0],2), det.get('reason'), det.get('boundary_selection'), det.get('boundary_candidate_reason'), det.get('boundary_candidate_ahead_m'),
              'recent' if getattr(c,'recent_path',None) is not None else '-', 'path' if c.path is not None else '-')
        for x in log: print('   agree', x)
        log.clear()
    if CUR[0] > WIN[1]: break
