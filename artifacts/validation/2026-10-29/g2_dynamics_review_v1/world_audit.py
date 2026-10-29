from pathlib import Path
import json
from analyze_contacts import world_geometry_equal
b=Path(__file__).resolve().parent
result=dict(same_world_after_verified_mesh_path_normalization=world_geometry_equal(b.parent/'g2_entry_left_v1/run1',b.parent/'g2_entry_left_v1/run2'),note='runtime tmp folder names differ, batched OBJ contents and all remaining world XML must match')
(b/'run2_world_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(result)
