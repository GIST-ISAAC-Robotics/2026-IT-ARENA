"""같은 2.0 m/s 설정을 실제 2×5 RGB·3인칭 영상과 함께 한 번 실행한다."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

BASE=Path(__file__).resolve().parent
ROOT=BASE.parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from prelap_signal_eval import fixture_inputs


def arguments(attempt):
    original=BASE.parent/'corner_approach_v1/solo_2p0_pre_manifest.json'
    ref=json.loads(original.read_text(encoding='utf-8'))['command_arguments']
    args=ref[ref.index('scripts/validate_basic_autonomy.py'):]
    assert args[args.index('--speed-profile')+1]=='g1_open_2p0_approach'
    assert args[args.index('--grid-slot')+1]=='3' and args[-2]=='--output'
    return [*args[:-2],'--team-signal-prelap','--video','--output',str(BASE/f'run{attempt}')]


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--attempt',type=int,choices=(1,2),default=1)
    opts=p.parse_args()
    args=arguments(opts.attempt)
    pre=BASE/('registered.json' if opts.attempt==1 else 'registered2.json')
    sources=fixture_inputs()
    import hashlib
    geom='scripts/build_experimental_track.py'
    sources[geom]=hashlib.sha256((ROOT/geom).read_bytes()).hexdigest()
    # Gazebo를 열기 전에 실제 신호 월드에서 평면 충돌 감사가 가능한지 확인한다.
    from build_experimental_track import obstacle_polygons
    from prelap_signal_eval import FIXTURE
    assert len(obstacle_polygons(FIXTURE/'world.sdf'))>0
    ref=json.loads((BASE.parent/'corner_approach_v1/solo_2p0_pre_manifest.json').read_text(encoding='utf-8'))
    for rel in ref['source_sha256']:
        sources[rel]=hashlib.sha256((ROOT/rel).read_bytes()).hexdigest()
    sources[Path(__file__).resolve().relative_to(ROOT).as_posix()]=hashlib.sha256(Path(__file__).read_bytes()).hexdigest()
    evidence=dict(args=args,source_sha256=sources,
        source_delta_vs_solo_2p0={k:[v,sources[k]] for k,v in ref['source_sha256'].items() if v!=sources[k]})
    if opts.prepare_only:
        with pre.open('x',encoding='utf-8') as stream:json.dump(evidence,stream,indent=2,ensure_ascii=False)
        print('registered',len(sources),'inputs')
    else:
        assert json.loads(pre.read_text(encoding='utf-8'))==evidence and not (BASE/f'run{opts.attempt}').exists()
        with (BASE/('console.log' if opts.attempt==1 else 'console2.log')).open('x') as log:
            result=subprocess.run([sys.executable,'-B',*args],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT)
        sys.exit(result.returncode)
