"""이번에 고친 문서의 상대 링크·앵커와 원본 ZIP 해시·보존 ZIP 불변을 확인한다."""
from hashlib import sha256
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[4]
DOCS = ['docs/PROJECT_CONTEXT.md', 'docs/DEVELOPMENT_PLAN.md', 'docs/README.md', 'docs/reports/OFFICIAL_UPDATE_2026_11_02.md',
        'docs/decisions/0028-speed-continuity-and-final-gearing.md', 'docs/activity/2026-11-02.md', 'docs/autonomy/BASIC_DEMO.md',
        'docs/HARDWARE_AND_RULE_QUESTIONS.md', 'README.md', 'artifacts/validation/2026-11-02/official_facility_v1/README.md',
        'artifacts/validation/2026-11-02/official_start_roi_v1/README.md',
        'artifacts/validation/2026-11-02/official_forward_v1/README.md',
        'artifacts/validation/2026-11-02/official_forward_v1/HANDOFF.md',
        'artifacts/validation/2026-11-02/official_traffic_v1/README.md', 'artifacts/validation/2026-11-02/official_traffic_v1/PLAN.md']
LINK = re.compile(r'\]\(<?([^)>\s]+)>?\)')


def anchors(path):
    found = set()
    for line in path.read_text(encoding='utf-8').splitlines():
        match = re.match(r'#+\s+(.*)', line)
        if match:
            found.add(re.sub(r'[^\w\- ]', '', match.group(1).strip().lower()).replace(' ', '-'))
        found.update(re.findall(r'<a id="([^"]+)"', line))
    return found


def main():
    missing, checked = [], 0
    for name in DOCS:
        path = ROOT/name
        for target in LINK.findall(path.read_text(encoding='utf-8')):
            if '://' in target:
                continue
            file, _, anchor = target.partition('#')
            resolved = (path.parent/file).resolve() if file else path
            checked += 1
            if not resolved.exists():
                missing.append((name, target))
            elif anchor and resolved.suffix == '.md' and anchor not in anchors(resolved):
                missing.append((name, target))
    tracked = subprocess.run(['git', 'status', '--porcelain', 'assets/track/official'], cwd=ROOT, stdout=subprocess.PIPE, text=True).stdout
    result = dict(checked_links=checked, missing=missing, official_assets_status=tracked,
                  zip_sha256=sha256((ROOT/'assets/track/official/v2026.11.01/it_arena_track_v2026.11.01.zip').read_bytes()).hexdigest())
    (Path(__file__).parent/'doc_links_check.json').write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8', newline='\n')
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
