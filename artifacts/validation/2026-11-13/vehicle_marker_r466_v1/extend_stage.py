import hashlib,json,shutil
from pathlib import Path
root=Path(__file__).resolve().parent;repo=root.parents[3]
manifest=json.loads((root/'original_sha256.json').read_text(encoding='utf-8'))
for name in ['local_pursuit.py','follow_trace.py','runtime_node_backend.py','g1_g2_node.py',
             'race_runtime_policy/evaluator.py','lidar_safety.py']:
    rel='src/arena_autonomy/arena_autonomy/'+name
    source=repo/rel;dest=root/'stage'/rel;original=root/'original'/rel
    if original.exists():raise RuntimeError('already captured '+rel)
    original.parent.mkdir(parents=True,exist_ok=True);dest.parent.mkdir(parents=True,exist_ok=True)
    shutil.copyfile(source,original);shutil.copyfile(source,dest)
    manifest[rel]=hashlib.sha256(source.read_bytes()).hexdigest()
(root/'original_sha256.json').write_text(json.dumps(manifest,indent=2)+'\n',encoding='utf-8')
