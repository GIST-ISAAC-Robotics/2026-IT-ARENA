"""새 코스의 등속 앞차 추월을 같은 실행기로 준비한다. 평가 각본만 변경."""
import datetime
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import zipfile
HERE=Path(__file__).resolve().parent;ROOT=HERE.parents[3]
OLD=HERE.parent/'return_guide_front_v1'
sha=lambda p:hashlib.sha256(Path(p).read_bytes()).hexdigest()

def prepare():
    cfg=json.loads((OLD/'new_track_course.json').read_text(encoding='utf-8'))
    case=next(c for c in cfg['cases'] if c['name']=='straight_moving_lead')
    case.update(duration_s=14.,schedule=[dict(from_s=0.,to_s=14.,speed_mps=.30)],
        expected='오른쪽12 cm 등속0.30 m/s 상대. 뒤→옆→앞→본선 복귀 뒤 최소0.50 m 전진, 현재 두 바퀴 잔디 정책·접촉0')
    cfg['cases']=[case]
    cfg['registered_at']='2026-11-07 신규 코스 정상 저속 추월 준비(미실행)'
    cfg['scope']='새 직선14초 등속 상대 추월, 원래 가속/양보 장면과 구별. 정답은 평가용'
    (HERE/'new_track_course.json').write_text(json.dumps(cfg,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    for name in ('run.py','run.sh','assess.py'):
        text=(OLD/name).read_text(encoding='utf-8').replace('return_guide_front_v1','steady_pass_newcourse_v1')
        if name=='assess.py':
            a=text.index('        returned, first, after =');b=text.index('    result = dict(',a)
            text=text[:a]+'''        first = None
        returned = 0.
        for t, x, y, p in track:
            if complete and t >= complete[0] and x >= .20 and abs(y) <= .06:
                first = p if first is None else first
                returned = max(returned,p-first)
            else:
                first = None
        behavior = dict(
            moving_pass_on=header['execution_config'].get('moving_pass_trial') is True,
            handoff_execution=bool(report.get('handoff_execution', {}).get('passed')),
            actor_followed_schedule=all(.25 <= v <= .35 for v in rates),
            entered=entry is not None,
            started_behind=track[0][1] < -.20,
            observed_beside=any(abs(x) <= .20 for _,x,_,_ in track),
            finished_ahead=track[-1][1] >= .20,
            completed_return_and_progress=bool(complete) and returned >= .50,
            no_permanent_stop=motion['longest_below_005_after_motion_s'] <= .50)
        extra = dict(entry_after_green_s=entry,pass_completed_times_s=complete,
            post_return_progress_m=returned,final_relative=track[-1],
            max_abs_lateral_m=max(abs(y) for _,_,y,_ in track))
'''+text[b:]
        (HERE/name).write_text(text,encoding='utf-8',newline='\n')

def register():
    folder=HERE/'straight_moving_lead';folder.mkdir(exist_ok=False)
    old=json.loads((OLD/'straight_moving_lead/registered.json').read_text(encoding='utf-8'))
    args=old['args'];args[args.index('--config')+1]=(HERE/'new_track_course.json').relative_to(ROOT).as_posix()
    files=set(subprocess.check_output(['git','ls-files','-co','--exclude-standard','src','scripts','config'],cwd=ROOT,text=True).splitlines())
    files=sorted({p for p in files if (ROOT/p).is_file()}|{p.relative_to(ROOT).as_posix() for p in HERE.iterdir() if p.is_file()})
    with zipfile.ZipFile(folder/'source_snapshot.zip','x',zipfile.ZIP_DEFLATED) as z:
        for p in files:z.write(ROOT/p,p)
    data=dict(registered_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
        scope='new track one steady low-speed pass, no speed cap change, selected flat grass retained',
        args=args,sources={p:sha(ROOT/p) for p in files},snapshot_sha256=sha(folder/'source_snapshot.zip'))
    (folder/'registered.json').write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n',encoding='utf-8')
    print('registered',len(files))

if __name__=='__main__':{'prepare':prepare,'register':register}[sys.argv[1]]()
