"""이번 작업의 문서 링크·원시 기록 제외·기본 실행 불변만 검사한다."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import unicodedata
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent


def git(*args):
    return subprocess.check_output(['git', *args], cwd=ROOT, text=True).splitlines()


def anchors(text):
    result, counts = set(), {}
    fenced = False
    for line in text.splitlines():
        if line.lstrip().startswith('```'):
            fenced = not fenced
        if fenced or not re.match(r'^#{1,6} ', line):
            continue
        title = re.sub(r'^#{1,6} +', '', line).strip().lower()
        title = re.sub(r'\[([^]]+)\]\([^)]+\)', r'\1', title)
        title = ''.join(c for c in title if c in '-_ ' or unicodedata.category(c)[0] in 'LN')
        title = title.replace(' ', '-')
        count = counts.get(title, 0)
        counts[title] = count + 1
        result.add(title + (f'-{count}' if count else ''))
    return result


def main(output=None):
    output = Path(output) if output is not None else BASE/'handoff_checks.json'
    if output.exists():
        raise FileExistsError(output)
    changed = sorted(set(git('diff','--name-only','HEAD') + git('ls-files','--others','--exclude-standard')))
    documents = [p for p in changed if p.endswith('.md') and (ROOT/p).is_file()]
    links, failures = 0, []
    for relative in documents:
        path = ROOT/relative
        for match in re.finditer(r'\[[^]\n]*\]\(([^)\n]+)\)', path.read_text(encoding='utf-8')):
            target = unquote(match.group(1).strip('<>'))
            if re.match(r'^[a-zA-Z][\w+.-]*:', target):
                continue
            link, _, anchor = target.partition('#')
            resolved = (path.parent/link).resolve() if link else path
            links += 1
            if not resolved.exists():
                failures.append(dict(document=relative,target=target,error='missing file'))
            elif anchor and resolved.suffix == '.md' and anchor not in anchors(resolved.read_text(encoding='utf-8')):
                failures.append(dict(document=relative,target=target,error='missing anchor'))
    raw_names = {'follow_decisions.jsonl','ego_poses.jsonl','actor_poses.jsonl','actor_contacts.jsonl',
                 'scans_received.jsonl','autonomy_status.jsonl','trace.json','trajectory.jsonl',
                 'actuation_trace.json','acquisition.json','braking_diagnostics.jsonl'}
    raw = [p for p in (ROOT/'artifacts/validation/2026-10-15').rglob('*')
           if p.is_file() and p.name in raw_names and
           p.relative_to(ROOT/'artifacts/validation/2026-10-15').parts[0].startswith(('g1_stress_','g1_solo_lap_'))]
    raw_bad = []
    for path in raw:
        rel = path.relative_to(ROOT).as_posix()
        ignored = subprocess.run(['git','check-ignore','-q',rel],cwd=ROOT).returncode == 0
        tracked = bool(git('ls-files','--',rel))
        if not ignored or tracked:
            raw_bad.append(rel)
    protected = ('src/arena_autonomy/','src/arena_bringup/','src/arena_gazebo/','src/arena_description/')
    protected_changes = [p for p in changed if p.startswith(protected)]
    historical_changes = [p for p in git('diff','--name-only','HEAD')
                          if p.startswith('artifacts/') and not p.startswith('artifacts/validation/2026-10-15/')]
    result = dict(document_count=len(documents), local_links_checked=links, link_failures=failures,
                  raw_files_checked=len(raw), raw_exclusion_failures=raw_bad,
                  protected_runtime_changes=protected_changes,historical_artifact_changes=historical_changes,
                  passed=not (failures or raw_bad or protected_changes or historical_changes),
                  scope='local handoff checks, not simulation or hardware validation',
                  documents_sha256={p:hashlib.sha256((ROOT/p).read_bytes()).hexdigest() for p in documents})
    output.write_text(json.dumps(result,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in result.items() if k!='documents_sha256'},ensure_ascii=False))
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path)
    raise SystemExit(main(parser.parse_args().output))
