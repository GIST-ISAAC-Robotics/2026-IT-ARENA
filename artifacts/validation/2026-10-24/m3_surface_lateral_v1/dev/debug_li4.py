import math, sys
import numpy as np
sys.path[:0] = ['tests']
from test_lateral_intent import *
from arena_autonomy.lateral_intent import *
path = np.column_stack((np.linspace(-.25, 1.6, 36), np.zeros(36)))
frame = PathFrame(path); state = ego_state(frame, GRASS)
cand = offset_candidate(frame, state, -.22, GRASS)
r = surface_check(frame, cand, GRASS, current_pose=(0, 0, 0))
print({k: r[k] for k in ('ok', 'reason', 'road_only_ok', 'two_wheel_ok', 'grass_side', 'outer_unconfirmed_samples')})
wl = r['wheel_l']; inner = np.abs(wl)+.006 <= .225-r['edge_margin_m']
bad = ~inner[:, [0, 2]].all(axis=1) & ~inner[:, [1, 3]].all(axis=1)
print('rows both pairs unconfirmed', np.flatnonzero(bad)[:10], wl[bad][:5])
scene, path = straight(-.09)
cfg = GRASS
frame = PathFrame(path); state = ego_state(frame, cfg); grid = ObservationGrid(scene, cfg)
corr = corridor_reference(frame, grid.returns, state['s0']-.6, state['s0']+1.2, cfg)
cs, cl = cell_coordinates(grid, frame)
seeds = grid.hidden_seeds(corridor_mask(cl, corr, cfg)); ex, ey = grid.centers(); sx = np.column_stack((ex[seeds], ey[seeds]))
hold = offset_candidate(frame, state, state['d0'], cfg, 'hold'); center = offset_candidate(frame, state, 0., cfg, 'center')
p, t = lateral_sweep(state, center, frame, cfg, cfg.commit_horizon_s, 0.)
print('poses', np.round(p[[0, -1]], 4), t[-1])
first = sweep_cells(grid, p, t, cfg, steering_protrusion_m(center['max_abs_steering_rad'], cfg))
hp, ht = lateral_sweep(state, hold, frame, cfg, cfg.commit_horizon_s, 0.)
hf = sweep_cells(grid, hp, ht, cfg, steering_protrusion_m(hold['max_abs_steering_rad'], cfg))
new = np.isfinite(first) & ~np.isfinite(hf) & (grid.state != EGO) & ~((ex < -.1) & (np.abs(ey) <= .075))
cells = np.column_stack((ex[new], ey[new])); tau = first[new]+.1
reach, best = hidden_reach(sx, cells, tau, cfg, .075)
print('reach cells', cells[reach], tau[reach], best[reach])
print('protrusion center', steering_protrusion_m(center['max_abs_steering_rad'], cfg), center['max_abs_steering_rad'])
