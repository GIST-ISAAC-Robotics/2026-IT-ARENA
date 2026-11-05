"""동일 14초 앞차 각본으로 선택형 이동차 추월 1회. 등록 파일 별도·기존 출력 재사용 금지."""
from pathlib import Path
import hashlib
import json
import subprocess
import sys
import zipfile

HERE=Path(__file__).resolve().parent
ROOT=HERE.parents[3]
BASE=HERE.parent/'moving_follow_continuity_v1/closed_loop_v2'
def sha(p):return hashlib.sha256(p.read_bytes()).hexdigest()

def files(name):
    extra=set()
    if name.startswith('closed_loop_side_'):extra.add(HERE/'offset_course.json')
    if name.startswith('closed_loop_side_pass_'):
        extra.add(HERE/'closed_loop_side_follow_1/run1/report.json')
    return sorted(set((ROOT/'src/arena_autonomy/arena_autonomy').glob('*.py')) |
        set((ROOT/'src/arena_bringup/launch').glob('*.py')) |
        {ROOT/n for n in ('scripts/validate_g1_traffic.py','scripts/g1_handoff_eval.py',
             'scripts/g1_traffic_eval.py','scripts/surface_contact_eval.py',
             'src/arena_description/config/b0183_c1_jr60p20_sim.yaml',
             'src/arena_gazebo/worlds/it_arena_official/world.sdf',
             'src/arena_gazebo/worlds/it_arena_official/scene.json')} |
        {BASE/'registered.json',HERE.parent/'moving_pass_baseline_v1/course.json',
         HERE/'review.py',HERE/'run_once.py',HERE/'run_once.sh'} | extra)

def manifest(name):
    previous=json.loads((BASE/'registered.json').read_text(encoding='utf-8'))
    args=list(previous['args']);args[args.index('--output')+1]='OUTPUT'
    side=name.startswith('closed_loop_side_')
    follow=name.startswith('closed_loop_side_follow_')
    if side:
        args[args.index('--config')+1]=(HERE/'offset_course.json').relative_to(ROOT).as_posix()
    if not follow:args.insert(args.index('--output'),'--moving-pass-trial')
    return dict(scope='저속 이동차 14초, 상대 0.30 m/s. 측방 배치는 평가기 전용.',
       actor_offset_m=-.12 if side else 0.,variant='follow' if follow else 'pass',
       args=args, sources={p.relative_to(ROOT).as_posix():sha(p) for p in files(name)},
       comparison=('오른쪽 0.12 m 별도 배치. 같은 배치의 OFF 실주행을 기준으로 사용.'
                   if side else 'r8 중앙 앞차 closed_loop_v2, 차체 주행 4.500 m. 상대 동작/월드/시간/보호/STOP 동일.'),
       acceptance=dict(valid='기록 및 실행 무결성, 실제 moving_pass_trial ON, STOP/정상 종료, 수집 후 입력 불변.',
           actor='모든 1초 구간에서 상대 전진 >=0.25 m; 같은 직선 장면만.',
           overtake='뒤(중심 -0.20 m 이하) → 옆(전후 차이 0.10 m 이내, 측방 0.15 m 이상) → 앞(+0.20 m) → 자차 본래 중앙선 복귀(횡편차 0.06 m 이내) 뒤 0.20 m 이상 전진. 순서·실제 후보 완료 기록 둘 다 필요.',
           withdrawal='선행차가 계속 앞에 있을 때 포기 1회·자차 중앙선 0.06 m 이내 복귀·0.50 m 이상 추가 전진, 실측 접촉/노면/종료 기준 유지. 추월 성공과 별개.',
           exposure='접촉 메시지·겹침·벽 접촉 0, 연속 하계 간격 양수, 세 바퀴 이상 이탈 0. 절대 간격 실측 별도 공개. 옛 추종 150 mm·계속 뒤 유지·차체 전체 도로 판정도 원본 그대로 공개.',
           benefit='같은 14초 r8 추종 진행거리보다 커야 함. 출발 직후 가속이나 순위만 바뀐 것은 성공 아님.',
           limitation='1회 각본 표본. 실물·변속·빠른 상대·코너·다른 차량과 병렬 주행 미검증.'))

def main():
    action,name=sys.argv[1:];folder=HERE/name
    if not name.startswith('closed_loop_') or Path(name).name!=name:raise ValueError('new local folder required')
    reg=folder/'registered.json'
    if action=='register':
        folder.mkdir(exist_ok=False)
        data=manifest(name);reg.write_text(json.dumps(data,indent=2,ensure_ascii=False)+'\n',encoding='utf-8')
        with zipfile.ZipFile(folder/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
            for p in files(name):z.writestr(p.relative_to(ROOT).as_posix(),p.read_bytes())
    elif action=='run':
        data=json.loads(reg.read_text(encoding='utf-8'));out=folder/'run1'
        if data!=manifest(name) or out.exists():raise SystemExit('registered input mismatch or output present')
        args=[str(out) if v=='OUTPUT' else str(ROOT/v) if v.startswith(('src/','artifacts/')) else v for v in data['args']]
        raise SystemExit(subprocess.call([sys.executable,'-B',*args],cwd=ROOT))
    else:raise ValueError(action)
if __name__=='__main__':main()
