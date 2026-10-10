"""동일 r6 반복 등록. 기존 산출물은 덮어쓰지 않는다."""
import copy
import hashlib
import json
from pathlib import Path
import subprocess
import zipfile

ROOT = Path(__file__).resolve().parents[4]
BASE = Path(__file__).resolve().parent
OLD = BASE.parent/'initial_startup_v1'
def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()
contract = json.loads((ROOT/'config/tests/scheduled_command_r6_sha256.json').read_text())
sources = {**contract['sha256'], **contract['implementation_sha256'], **contract['evaluation_tools_sha256']}
for path, digest in sources.items():
    assert sha(ROOT/path) == digest, path
plugin = ROOT/'install/arena_gazebo/lib/libarena-single-motor-system.so'
assert sha(plugin) == 'c48d4a99e42ba11cdf493439bf665495131aba89075c7e378a5c275597341f78'
plan = copy.deepcopy(json.loads((OLD/'plan_e2_embedded.json').read_text()))
plan['base'] = BASE.relative_to(ROOT).as_posix()
plan['cases'][0]['name'] = 'e3_embedded'
with (BASE/'plan_e3_embedded.json').open('x') as f: json.dump(plan,f,indent=2)
files = list(sources) + ['config/tests/scheduled_command_r6_sha256.json']
files += [p.relative_to(ROOT).as_posix() for p in BASE.iterdir() if p.is_file()]
with zipfile.ZipFile(BASE/'pre_source.zip','x',zipfile.ZIP_DEFLATED) as z:
    for path in files: z.write(ROOT/path,path)
with (BASE/'pre_manifest.json').open('x') as f:
    json.dump({'sha256':{p:sha(ROOT/p) for p in files}, 'plugin_sha256':sha(plugin),
               'git_status_before':subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True)},f,indent=2)
print('r6 sources and plugin unchanged; E3 registered')
