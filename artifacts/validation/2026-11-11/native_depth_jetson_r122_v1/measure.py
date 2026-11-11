"""격리 C++ 후보의 Jetson 원/후 비교. ROS·센서/구동 명령 없음."""
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

A = Path(__file__).resolve().parent
ROOT = A.parents[3]
SOURCE = A.parent/'native_depth_r121_v1'
sys.path.insert(0,str(SOURCE))


def main():
    manifest=json.loads((A/'bundle_manifest.json').read_bytes())
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    check=lambda:[n for n,v in manifest['files'].items() if digest(ROOT/n)!=v['sha256']]
    assert not check(),check()
    spec=importlib.util.spec_from_file_location('comparison_r121',SOURCE/'compare.py')
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    module.A=A
    (A/'evaluate.cpp').write_bytes((SOURCE/'evaluate.cpp').read_bytes())
    module.main()
    assert not check(),check()
    with (A/'integrity.json').open('x') as f:
        json.dump(dict(files=len(manifest['files']),source_unchanged=True),f,indent=2)


if __name__=='__main__':
    main()
