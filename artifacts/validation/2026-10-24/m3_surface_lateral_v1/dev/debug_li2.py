import math, sys
import numpy as np
sys.path[:0] = ['tests']
from test_lateral_intent import *
from arena_autonomy.lateral_intent import *
import arena_autonomy.lateral_intent as li
scene, path = straight(-.09)
cfg = GRASS
frame = PathFrame(path); state = ego_state(frame, cfg); grid = ObservationGrid(scene, cfg)
corr = corridor_reference(frame, grid.returns, state['s0']-.6, state['s0']+1.2, cfg)
cmask, _ = corridor_mask(grid, frame, corr, cfg); seeds = grid.hidden_seeds(cmask)
ex, ey = grid.centers()
hold = offset_candidate(frame, state, state['d0'], cfg, 'hold'); center = offset_candidate(frame, state, 0., cfg, 'center')
poses, times = lateral_sweep(state, center, frame, cfg, cfg.commit_horizon_s, .35)
print('sweep poses', np.round(poses[[0, len(poses)//2, -1]], 4), 'times', times[[0, -1]])
first = sweep_cells(grid, poses, times, cfg, steering_protrusion_m(center['max_abs_steering_rad'], cfg))
hp, ht = lateral_sweep(state, hold, frame, cfg, cfg.commit_horizon_s, .35)
hfirst = sweep_cells(grid, hp, ht, cfg, 0.)
new = np.isfinite(first) & ~np.isfinite(hfirst) & (grid.state != EGO)
obs = grid.observed_free()
cells = np.column_stack((ex[new], ey[new]))
print('new cells x range', cells[:, 0].min(), cells[:, 0].max(), 'y range', cells[:, 1].min(), cells[:, 1].max())
bad = new & ~obs
print('unobserved new', np.column_stack((ex[bad], ey[bad])), grid.state[bad])
tau = first[new] + 0.1
sx = np.column_stack((ex[seeds], ey[seeds]))
reach, best = hidden_reach(sx, cells, tau, cfg, .075)
print('reach cells', cells[reach], 'tau', tau[reach], 'best', best[reach])
# nearest seeds
for c in cells[reach][:3]:
    d = np.hypot(sx[:, 0]-c[0], sx[:, 1]-c[1]); k = np.argsort(d)[:3]; print(c, sx[k], d[k])
print('seed x max', sx[:, 0].max(), 'n', len(sx))
fr = sx[sx[:, 0] > -.1]; print('seeds ahead of rear', fr[:10], len(fr))
print('---- argmin seeds')
for c, tt in zip(cells[reach][4:8], tau[reach][4:8]):
    r1, b1 = [], []
    best_t, best_q = 9, None
    for q in sx:
        _, b = hidden_reach(q[None], c[None], np.array([tt]), cfg, .075)
        if b[0] < best_t:
            best_t, best_q = b[0], q
    print(c, tt, best_t, best_q)
    i, j, _ = grid.index(best_q[None])
    hx, hy = 8, 5
    win = grid.state[i[0]-hx:i[0]+hx+1, j[0]-hy:j[0]+hy+1]
    print('window states', np.unique(win, return_counts=True), 'corridor', cmask[i[0], j[0]])
