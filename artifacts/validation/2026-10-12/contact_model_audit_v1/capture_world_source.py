"""추가 확인한 내부 step 순서의 1차 원문을 별도로 보존한다."""
import hashlib
import json
from pathlib import Path
import urllib.request

base = Path(__file__).resolve().parent
url = 'https://raw.githubusercontent.com/dartsim/dart/v6.13.2/dart/simulation/World.cpp'
with urllib.request.urlopen(url, timeout=45) as response:
    data = response.read()
with (base/'upstream'/'World.cpp').open('xb') as stream:
    stream.write(data)
with (base/'upstream_world_source.json').open('x') as stream:
    json.dump({'url': url, 'version_tag': 'v6.13.2', 'sha256': hashlib.sha256(data).hexdigest()}, stream, indent=2)
print('Captured World.cpp step ordering')
