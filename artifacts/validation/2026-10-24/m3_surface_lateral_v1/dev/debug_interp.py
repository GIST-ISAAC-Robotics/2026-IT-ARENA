import sys
import numpy as np
sys.path[:0] = ['tests']
from test_lateral_intent import *
t = np.linspace(-.25, 1.6, 36); radius = .5
path = np.column_stack((radius*np.sin(t/radius), radius*(1-np.cos(t/radius))))
scene, _ = straight(0.)
grid = ObservationGrid(scene, GRASS); frame = PathFrame(path)
s4, l4 = cell_coordinates(grid, frame); s1, l1 = cell_coordinates(grid, frame, stride=1)
ex, ey = grid.centers()
err = np.abs(l4-l1); err[(np.abs(l1) > .45) | (s1 < 0) | (s1 > frame.end_s)] = 0
i = np.unravel_index(np.argmax(err), err.shape)
print('max err', err[i], 'at', ex[i], ey[i], 's1', s1[i], 'l1', l1[i], 'l4', l4[i], 'path end s', frame.end_s)
bad = err > .003
print('bad cells', bad.sum(), 'x range', ex[bad].min(), ex[bad].max(), 'y', ey[bad].min(), ey[bad].max(), 's', s1[bad].min(), s1[bad].max())
