from pathlib import Path
import hashlib,json,tarfile,shutil
archive=Path('/home/q/c1_preview_r204_20261112.tar.gz')
assert hashlib.sha256(archive.read_bytes()).hexdigest()=='d1340c65a2c30b5f5a859e6806a8747791338b5c7cd90ae306d873843d286728'
root=Path('/home/q/it_arena_c1_preview_r204_20261112');root.mkdir(exist_ok=False)
with tarfile.open(archive) as t:
 for m in t.getmembers():assert (root/m.name).resolve().is_relative_to(root) and m.isfile()
 t.extractall(root)
m=json.loads((root/'artifacts/validation/2026-11-12/c1_nonreturn_live_r204_v1/source_manifest.json').read_bytes())
assert all(hashlib.sha256((root/p).read_bytes()).hexdigest()==h for p,h in m.items())
old=Path('/home/q/it_arena_c1_preview_r166b_20261112/stream_c1_native_preview')
assert hashlib.sha256(old.read_bytes()).hexdigest()=='d8f7d2a02ef47786022206e2a0278c6e7edc2d3ddb40bed2254706b590748e2d'
shutil.copy2(old,root/'stream_c1_native_preview')
print(json.dumps(dict(source_files=len(m),sdk_binary_unchanged=True)))
