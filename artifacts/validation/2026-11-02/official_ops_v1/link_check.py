from pathlib import Path
import re
base = Path('artifacts/validation/2026-11-02/official_ops_v1')
missing = []
for f in [base/'README.md', base/'HANDOFF.md', Path('docs/PROJECT_CONTEXT.md'), Path('docs/autonomy/RACE_OPERATOR_RUNBOOK.md'), Path('docs/DEVELOPMENT_PLAN.md')]:
    for link in re.findall(r'\]\(([^)#\s]+)', f.read_text(encoding='utf-8')):
        if not link.startswith('http') and not (f.parent/link).exists():
            missing.append((f.name, link))
print('missing', missing)
