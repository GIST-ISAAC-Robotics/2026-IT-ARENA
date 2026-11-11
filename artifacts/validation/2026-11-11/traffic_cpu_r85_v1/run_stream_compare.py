"""Run each preserved source once with the same bounded-memory observer."""
import os
from pathlib import Path
import subprocess
import sys

jobs=[
 ('before','/home/q/it_arena_traffic_r82_20261111','/home/q/it_arena_traffic_r82_manifest_20261111.json'),
 ('after','/home/q/it_arena_traffic_r85_cpu_20261111','/home/q/it_arena_traffic_r85_cpu_manifest_20261111.json'),
]
for label,directory,manifest in jobs:
    root=Path(directory);env=os.environ.copy()
    env['PYTHONPATH']=':'.join(str(root/p) for p in ('src/arena_autonomy','src/arena_vehicle_interface','scripts'))
    subprocess.run([sys.executable,'-B','/home/q/r85_measure_stream_20261111.py',
        '--trace',str(root/'artifacts/r82_expected/follow_decisions.jsonl'),'--manifest',manifest,
        '--output',f'/home/q/r85_stream_{label}_20261111.json',
        '--decisions',f'/home/q/r85_stream_{label}_20261111.jsonl.gz'],
        cwd=root,env=env,check=True)
