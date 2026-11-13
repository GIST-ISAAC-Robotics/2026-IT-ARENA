from pathlib import Path
import hashlib,json,shutil
A=Path(__file__).resolve().parent;ROOT=A.parents[3]
files=['src/arena_autonomy/arena_autonomy/moving_pass.py','src/arena_autonomy/arena_autonomy/bypass_entry_trial.py']
rows=[]
for name in files:
    for part in ('base','stage'):
        p=A/part/name;p.parent.mkdir(parents=True,exist_ok=True)
        if p.exists():raise RuntimeError('already prepared')
        shutil.copyfile(ROOT/name,p)
    rows.append(dict(path=name,sha256=hashlib.sha256((ROOT/name).read_bytes()).hexdigest()))
(A/'source_before.json').write_text(json.dumps(rows,indent=2),encoding='utf-8')
print('two sources frozen')
