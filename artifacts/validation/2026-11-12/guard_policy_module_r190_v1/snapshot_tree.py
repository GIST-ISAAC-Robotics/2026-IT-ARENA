from pathlib import Path
import shutil
import zipfile
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
destination=ROOT/'build/r190_before/arena_autonomy'
shutil.copytree(ROOT/'src/arena_autonomy/arena_autonomy',destination,
                ignore=shutil.ignore_patterns('__pycache__','*.pyc','guard_observation_policy.py'))
with zipfile.ZipFile(A/'before_sources.zip') as archive:
    for name in archive.namelist():
        target=destination/Path(name).name
        target.write_bytes(archive.read(name))
        assert target.read_bytes()==archive.read(name)
print('Preserved original package overlay under build/r190_before')
