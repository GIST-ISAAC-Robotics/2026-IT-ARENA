"""기존 반례를 새 모듈로 실행한다. 보존 원 시험은 수정하지 않는다."""
import hashlib
import json
import re
from pathlib import Path
import pytest

A = Path(__file__).resolve().parent
ROOT = A.parents[3]


def main():
    target = ROOT/'build/r176_reused_tests'
    target.mkdir(exist_ok=True)
    cases = (
        ('released_current_r167_v1/test_released.py', 'import released',
         'from arena_autonomy.current_path_recheck import _released as released'),
        ('present_guard_current_r170_v1/test_current.py', 'import current as candidate',
         'from arena_autonomy.current_path_recheck import _present as candidate'),
        ('released_parallel_r173_v1/test_current.py', 'import current as released',
         'from arena_autonomy.current_path_recheck import _released_parallel as released'),
    )
    manifest, paths = [], []
    for i, (relative, old, new) in enumerate(cases):
        source = A.parent/relative
        text = source.read_text(encoding='utf8')
        assert text.count(old) == 1
        text = text.replace(old, new)
        text, count = re.subn(r'A\s*=\s*Path\(__file__\)\.resolve\(\)\.parent',
                              'A = Path('+repr(str(source.parent))+')', text)
        assert count == 1
        path = target/f'test_migrated_{i}.py'
        path.write_text(text, encoding='utf8')
        paths.append(str(path))
        manifest.append(dict(source=source.relative_to(ROOT).as_posix(),
                             sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                             replacements=[old+' -> '+new, 'fixture directory preserved']))
    (A/'reused_test_manifest.json').write_text(json.dumps(manifest, indent=2)+'\n', encoding='utf8')
    return pytest.main(['-q', *paths, str(ROOT/'tests/test_current_path_recheck.py')])


if __name__ == '__main__':
    raise SystemExit(main())
