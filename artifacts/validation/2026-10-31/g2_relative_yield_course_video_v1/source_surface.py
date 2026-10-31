"""기존 팀 시험 fixture를 바이트 그대로 복사한다. 공식 원본은 수정하지 않는다."""
import hashlib
from pathlib import Path
import shutil


def build_fixture(source, target):
    source, target = Path(source), Path(target)
    if target.exists():
        raise FileExistsError(target)
    before = {p.relative_to(source): hashlib.sha256(p.read_bytes()).hexdigest()
              for p in source.rglob('*') if p.is_file()}
    shutil.copytree(source, target)
    assert all(hashlib.sha256((target/p).read_bytes()).hexdigest() == digest
               == hashlib.sha256((source/p).read_bytes()).hexdigest()
               for p, digest in before.items())
    digest = before[Path('world.sdf')]
    return dict(original_world_sha256=digest, variant_world_sha256=digest,
                all_fixture_files_unchanged=len(before), grass_shift_m=0.,
                original_grass_top_m=.002, road_top_m=.003,
                official_original_modified=False, physical_track_claimed=False)
