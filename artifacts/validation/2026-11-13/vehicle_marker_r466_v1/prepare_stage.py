from pathlib import Path
import hashlib,json

HERE=Path(__file__).resolve().parent
REPO=HERE.parents[3]
paths=['src/arena_autonomy/arena_autonomy/vision_process.py',
       'src/arena_autonomy/arena_autonomy/vehicle_markers.py',
       'src/arena_autonomy/arena_autonomy/core.py',
       'src/arena_autonomy/arena_autonomy/local_path.py',
       'src/arena_autonomy/arena_autonomy/local_scene.py',
       'src/arena_autonomy/arena_autonomy/scene_motion.py',
       'src/arena_description/config/b0390_c1_cad_20261113_final_frontwire_6mm.yaml',
       'src/arena_description/config/vehicle.final_cad_2026_11_13.yaml',
       'tests/test_vehicle_marker_observer.py','tests/test_vision_process.py']
manifest={}
for rel in paths:
    data=(REPO/rel).read_bytes();manifest[rel]=hashlib.sha256(data).hexdigest()
    target=HERE/'original'/rel;target.parent.mkdir(parents=True,exist_ok=True)
    assert not target.exists();target.write_bytes(data)
    if rel.endswith('vision_process.py') or rel.endswith('vehicle_markers.py'):
        target=HERE/'stage'/rel;target.parent.mkdir(parents=True,exist_ok=True);target.write_bytes(data)
(HERE/'original_sha256.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps({'original_files':len(paths),'stage_existing_files':2}))
