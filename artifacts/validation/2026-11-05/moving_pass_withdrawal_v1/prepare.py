"""상대가 추월 진입 후 빨라지는 짧은 직선. 평가 전용 각본·입력 사전등록."""
from pathlib import Path
import hashlib
import json
import sys
import zipfile

HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
PREV=HERE.parent/'moving_pass_v1'
def read(path):return json.loads(path.read_text(encoding='utf-8'))
def save(path,data):
    with path.open('x',encoding='utf-8') as f:json.dump(data,f,indent=2,ensure_ascii=False);f.write('\n')
def sha(path):return hashlib.sha256(path.read_bytes()).hexdigest()

def course():
    cfg=read(PREV/'offset_course.json')
    cfg['scope']='평가 각본 전용: 추월 진입 후 오른쪽 앞차가 증속하는 직선 10초.'
    case=cfg['cases'][0];case['duration_s']=10.
    case['schedule']=[dict(from_s=a,to_s=b,speed_mps=v) for a,b,v in
        [(0.,4.,.30),(4.,4.2,.40),(4.2,4.4,.50),(4.4,10.,.58)]]
    case['expected']='진입한 자차가 지속 속도 차를 잃는 경우. 이동차가 여전히 앞인지, 관측한 경로로 뒤에 합류해 계속 가는지를 구분.'
    return cfg

def register(name):
    child=HERE/name;child.mkdir(exist_ok=False)
    previous=read(PREV/'closed_loop_side_pass_2/registered.json')
    paths=[ROOT/p for p in previous['sources'] if p.startswith(('src/','scripts/'))]
    if name=='baseline_r12':
        assert all(sha(p)==previous['sources'][p.relative_to(ROOT).as_posix()] for p in paths),'r12 source mismatch'
    paths+=list(HERE.glob('*.py'))+[HERE/'run.sh',HERE/'course.json',HERE/'course_geometry.json',PREV/'closed_loop_side_pass_2/registered.json']
    args=previous['args'][:]
    args[args.index('--config')+1]=(HERE/'course.json').relative_to(ROOT).as_posix()
    assert '--moving-pass-trial' in args and '--handoff-continuous-pace' in args
    data=dict(args=args,sources={p.relative_to(ROOT).as_posix():sha(p) for p in sorted(paths)},
        scope='첫 저속차 추월 전용 0.65 m/s 구성의 진입 후 속도 차 상실. 빠른 상대 추월 성공 시험 아님.',
        expected=dict(actor_motion='첫 4초 1초 평균 .25~.35, 5~10초 1초 평균 .53~.63 m/s; 궤적·실행 무결성.',
            trigger='앞차 실제 증속 전에 추월 진입. 증속 시 상대가 0.20 m 이상 앞. 불발은 기능 통과로 세지 않는다.',
            withdrawal='진입 뒤 명시적 포기, 상대가 계속 앞, 원래 직선 중심 ±0.06 m 복귀 후 0.50 m 이상 연속 전진. 정지만 하는 것은 실패.',
            exposure='실제 접촉·샘플 겹침·벽 충돌 0, 연속 간격 하계 양수, 세 바퀴 이상 노면 이탈 0, 명시 STOP/정상 종료.',
            alternative='실제로 추월해 복귀하는 예외라면 포기 성공과 구분해 보고. 진입 전에 추종을 선택했다면 노출 불성립.'))
    save(child/'registered.json',data)
    with zipfile.ZipFile(child/'sources.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in sorted(paths):z.writestr(p.relative_to(ROOT).as_posix(),p.read_bytes())

if __name__=='__main__':
    if sys.argv[1]=='course':save(HERE/'course.json',course())
    elif sys.argv[1]=='register':register(sys.argv[2])
    else:raise ValueError('course or register')
