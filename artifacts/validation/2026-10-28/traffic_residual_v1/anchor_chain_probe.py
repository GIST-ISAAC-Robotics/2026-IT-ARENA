"""실패 스캔에서 정합 기준을 갈아 끼우지 않는 제한 사본 비교."""
from pathlib import Path
import hashlib
import json
import sys
import types

BASE=Path(__file__).resolve().parent

def main():
    source=(BASE/'chain_probe.py').read_text(encoding='utf-8')
    changed=source.replace("BASE/'chain_probe.json'", "BASE/'anchor_chain_probe.json'")
    old="""                points=transport_points(points,h,stamp,now)
                if not fit['accepted']:chain_valid=False
                if chain_valid:points+=np.asarray(fit['translation_m'])"""
    new="""                candidate_points=transport_points(anchor_points,h,stamp,now)
                chain_valid=bool(fit['accepted'] and now-stamp<=.3)
                points=candidate_points + np.asarray(fit['translation_m']) if chain_valid else candidate_points"""
    assert changed.count(old)==1;changed=changed.replace(old,new)
    old='            previous=(current,now)'
    new="""            if previous is None or chain_valid:
                previous=(current,now)
                anchor_points=points.copy()"""
    assert changed.count(old)==1;changed=changed.replace(old,new)
    target=BASE/'anchor_chain_effective.py'
    with target.open('x',encoding='utf-8') as stream:stream.write(changed)
    module=types.ModuleType('anchor_chain_trial');module.__file__=str(target)
    sys.modules[module.__name__]=module
    exec(compile(changed,str(target),'exec'),module.__dict__)
    module.main()
    print(json.dumps(dict(original_sha256=hashlib.sha256(source.encode()).hexdigest(),
                         effective_sha256=hashlib.sha256(changed.encode()).hexdigest())))

if __name__=='__main__':main()
