"""선정 단독 1랩의 인자를 그대로 쓰고 직선 옵션·출력 경로만 변경한다."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
PRIOR=HERE.parent/'solo_post_free_v1/run1_registered.json'
TRIAL=HERE.parent/'solo_speed_envelope_v1'
ENV=HERE.parents[1]/'2026-11-01/turn_preview_v1/environment.sh'


def sha(path):
    with path.open('rb') as f:
        return hashlib.file_digest(f,'sha256').hexdigest()


def registration():
    prior=json.loads(PRIOR.read_bytes())
    deployment=json.loads((TRIAL/'staged_deployment.json').read_bytes())
    for name,digest in deployment['after'].items():
        assert sha(ROOT/name)==digest,name
    assert (HERE/'build.exit').read_text().strip()=='0'
    assert (HERE/'regression.exit').read_text().strip()=='0'
    args=list(prior['args'])
    assert args[-2]=='--output' and '--video' in args and '--g1-solo-straight-budget' not in args
    args[-1]=str(HERE/'run1'); args.insert(-2,'--g1-solo-straight-budget')
    # 현재 제어기 전체와 등록 후처리/실행기도 보존. 이전 파일을 조용히 제외하지 않는다.
    names=set(prior['source_sha256'])|set(deployment['after'])
    names.update(p.relative_to(ROOT).as_posix() for p in (ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py'))
    names.update((HERE/n).relative_to(ROOT).as_posix() for n in ['PLAN.md','run.py','run_once.sh','review.py','post.sh',
                                                                'speed_chain.py','baseline_speed_chain.json'])
    for p in [PRIOR,TRIAL/'staged_deployment.json',ROOT/'scripts/inspect_autonomy_video.py',
              ROOT/'scripts/retime_autonomy_video.py',ROOT/'scripts/surface_contact_eval.py',
              ROOT/'artifacts/validation/2026-11-01/race_continuity_v1/measure_solo.py']:
        names.add(p.relative_to(ROOT).as_posix())
    # colcon 비-symlink 설치/오래된 패키지에서도 실제 사용 파일을 대조한다.
    checks={}
    import importlib.util
    from ament_index_python.packages import get_package_share_directory
    for package,rel in [('arena_bringup','launch/g1_follow.launch.py'),
                        ('arena_description','models/arena_car/model.sdf.xacro')]:
        installed=Path(get_package_share_directory(package))/rel
        checks[str(installed)]=sha(installed)
        assert sha(installed)==sha(ROOT/'src'/package/rel)
    for short in ('local_pursuit','local_follow','solo_straight_budget'):
        spec=importlib.util.find_spec('arena_autonomy.'+short)
        module=Path(spec.origin)
        checks[str(module)]=sha(module)
        assert sha(module)==sha(ROOT/'src/arena_autonomy/arena_autonomy'/(short+'.py'))
    return dict(args=args,source_sha256={n:sha(ROOT/n) for n in sorted(names)},resolved_files=checks,
                baseline_sources_changed_since_run=sorted(n for n,h in prior['source_sha256'].items() if sha(ROOT/n)!=h),
                scope='단독 1랩 선택형 직선 속도 후보·전방 구도의 외부 카메라. 기둥 없음, 주행 센서·구동·속도/물성 판정선 유지.')


if __name__=='__main__':
    p=argparse.ArgumentParser()
    p.add_argument('--prepare-only',action='store_true')
    args=p.parse_args()
    if (HERE/'run1').exists():
        raise FileExistsError('existing trial output')
    dest=HERE/'run1_registered.json'
    value=registration()
    if args.prepare_only:
        with dest.open('x',encoding='utf8') as f:
            json.dump(value,f,ensure_ascii=False,indent=2)
    else:
        if value!=json.loads(dest.read_bytes()):
            raise ValueError('registered sources changed')
        with (HERE/'run1_console.log').open('x',encoding='utf8') as log:
            result=subprocess.run(['bash',str(ENV),sys.executable,'-B',*value['args']],cwd=ROOT,
                                  stdout=log,stderr=subprocess.STDOUT)
        raise SystemExit(result.returncode)
