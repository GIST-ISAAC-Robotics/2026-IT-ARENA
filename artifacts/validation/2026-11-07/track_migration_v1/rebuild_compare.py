"""현재 빌더로 v2026.11.01 프로필을 임시 경로에 재생성해 보존 실행 월드와 바이트 대조한다.

보존 월드(src/arena_gazebo/worlds/it_arena_official)는 읽기만 한다. provenance.json은 내용 비교에서
input_sha256.builder 하나만 차이로 허용하고, 그 외 모든 파일은 바이트가 같아야 한다.
"""
import hashlib
import json
import shutil
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'scripts'))
import build_official_track as builder  # noqa: E402

OUT = Path(__file__).resolve().parent / 'checks/rebuild_v2026_11_01.json'


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def tree(root):
    return {p.relative_to(root).as_posix(): digest(p) for p in sorted(root.rglob('*')) if p.is_file()}


def main():
    preserved = builder.DESTINATION
    before = tree(preserved)
    scratch_root = ROOT / 'build/official_track_generation'
    scratch_root.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='rebuild_v2026_11_01_', dir=scratch_root) as tmp:
        target = Path(tmp) / 'it_arena_official'
        shutil.copytree(preserved, target)  # build()는 기존 디렉터리에 덮어쓰는 방식이므로 같은 시작 상태
        builder.build(builder.PROFILE, target)
        rebuilt = tree(target)
        old_prov = json.loads((preserved / 'provenance.json').read_text(encoding='utf-8'))
        new_prov = json.loads((target / 'provenance.json').read_text(encoding='utf-8'))
    after = tree(preserved)
    differing = sorted(k for k in set(before) | set(rebuilt) if before.get(k) != rebuilt.get(k))
    prov_old, prov_new = json.loads(json.dumps(old_prov)), json.loads(json.dumps(new_prov))
    builder_old = prov_old['input_sha256'].pop('builder')
    builder_new = prov_new['input_sha256'].pop('builder')
    report = {
        'scope': 'rebuild official_v2026.11.01 profile with current builder into scratch; preserved world read-only',
        'preserved_world': preserved.relative_to(ROOT).as_posix(),
        'preserved_world_unchanged_during_check': before == after,
        'preserved_world_sdf_sha256': before['world.sdf'],
        'files_compared': len(before),
        'differing_files': differing,
        'provenance_equal_except_builder_hash': prov_old == prov_new,
        'builder_sha256_recorded': builder_old,
        'builder_sha256_current': builder_new,
    }
    report['passed'] = bool(report['preserved_world_unchanged_during_check'] and differing in ([], ['provenance.json'])
                            and report['provenance_equal_except_builder_hash'])
    OUT.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
