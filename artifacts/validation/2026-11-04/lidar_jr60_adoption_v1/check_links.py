"""변경한 Markdown의 상대 링크·앵커(GitHub 규칙 근사) 확인. 인자: 파일 목록. 실패 시 exit 1."""
import re
import sys
from pathlib import Path


def slug(text):
    text = re.sub(r'[`*_\[\]()]', '', text.strip().lower())
    text = re.sub(r'[^\w\- ]', '', text)
    return text.replace(' ', '-')


def anchors(path):
    found = set()
    for line in path.read_text(encoding='utf-8').splitlines():
        m = re.match(r'#+\s+(.*)', line)
        if m:
            found.add(slug(m.group(1)))
        found.update(re.findall(r'<a id="([^"]+)"', line))
    return found


bad = 0
for name in sys.argv[1:]:
    path = Path(name)
    for target in re.findall(r'\]\(([^)\s]+)\)', path.read_text(encoding='utf-8')):
        if re.match(r'[a-z]+:', target):
            continue
        file, _, anchor = target.partition('#')
        dest = (path.parent / file).resolve() if file else path.resolve()
        if not dest.exists():
            print('MISSING', name, target); bad += 1
        elif anchor and dest.suffix == '.md' and anchor not in anchors(dest):
            print('ANCHOR', name, target); bad += 1
print('checked', len(sys.argv) - 1, 'files, problems', bad)
sys.exit(1 if bad else 0)
