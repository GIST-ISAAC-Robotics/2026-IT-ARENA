from test_surface_correspondence import observed_room
from arena_autonomy.surface_correspondence import surface_translation
import numpy as np,json
r,p,h=observed_room()
ids=np.flatnonzero(np.isfinite(r.ranges))
print(len(p),len(ids))
print(json.dumps(surface_translation(p-np.array([.03,.025]),p,ids),indent=2))
