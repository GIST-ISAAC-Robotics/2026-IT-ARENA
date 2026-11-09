"""선택한 고해상도 렌더 프레임만 PNG로 보존. 크기 변경이나 생성형 편집 없음."""
from pathlib import Path
import argparse,hashlib,json
from PIL import Image,ImageDraw,ImageFont

ROOT=Path(__file__).resolve().parents[4]
OUT=Path(__file__).resolve().parents[1]
def sha(p):return hashlib.file_digest(p.open('rb'),'sha256').hexdigest()
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--source',type=Path,required=True)
    ap.add_argument('--reused-group',action='append',default=[],help='변경 없는 그룹의 기존 렌더: 이름=디렉터리')
    a=ap.parse_args();reused={name:Path(path) for name,path in (v.split('=',1) for v in a.reused_group)}
    dest=OUT/'05_closeups';dest.mkdir(exist_ok=True)
    groups=json.loads((a.source/'all_meta.json').read_text(encoding='utf-8'));items=[]
    for group in groups:
        for name,digest in group['source_sha256'].items():assert sha(ROOT/name)==digest,name
        render_dir=reused.get(group['name'],a.source/group['name'])
        if group['name'] in reused:
            previous=json.loads((render_dir.parent/'all_meta.json').read_text(encoding='utf-8'))
            old=next(g for g in previous if g['name']==group['name'])
            for key in ('shots','source_sha256','ego_style','actor_visual_note'):
                assert old[key]==group[key],(group['name'],key,'기존 렌더 입력 불일치')
        frames=list((render_dir/'frames').glob('f_*.ppm'))
        assert frames
        for shot in group['shots']:
            frame=min(frames,key=lambda f:abs(int(f.stem[2:])/1e9-shot['frame_time_s']))
            actual=int(frame.stem[2:])/1e9
            assert abs(actual-shot['frame_time_s'])<.15,(shot['name'],actual)
            p=dest/(shot['name']+'.png')
            with Image.open(frame) as im:
                im.load();assert im.size==(3840,2400);im.save(p,optimize=True)
            method='기록 자세·조향 + 홍보용 외관 재렌더' if group['ego_style']=='legacy_box_promotional' else '원 시각 모델·기록 자세·조향의 근접 재렌더'
            note=group['actor_visual_note']+'. 저장 자세 정지 렌더. 자차 조향은 '+('기록된 실제 관절각' if shot['ego_steering_source']=='recorded_measured_joint_angles' else '기록된 MCU 최종 명령의 Ackermann 환산')+' 반영. 바퀴 구름 회전은 재현하지 않음.'
            items.append(dict(file=p.relative_to(OUT).as_posix(),title=shot['title'],category='차량 근접 렌더',date='2026-11-09',
              width=3840,height=2400,sha256=sha(p),method=method,note=note,evidence=group['run'],
              recorded_sim_time_s=shot['time'],render_frame_s=actual,camera_pose=shot['camera_pose'],source_frame_sha256=sha(frame),
              render_source_directory=str(render_dir),
              ego_steering_deg=shot['ego_steering_deg'],ego_steering_source=shot['ego_steering_source'],
              source_sha256=group['source_sha256']))
    manifest=dict(items=items,scope='고해상도 정지 재렌더, 주행 검증 아님',source_directory=str(a.source))
    (OUT/'closeup_manifest.json').write_text(json.dumps(manifest,ensure_ascii=False,indent=2),encoding='utf-8')
    canvas=Image.new('RGB',(1640,4*550+100),'#edf1f4');d=ImageDraw.Draw(canvas)
    font=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',23);title=ImageFont.truetype('/mnt/c/Windows/Fonts/malgun.ttf',31)
    d.text((24,22),'차량 근접 렌더 · 8장',font=title,fill='#173449')
    for n,item in enumerate(items):
        im=Image.open(OUT/item['file']).convert('RGB');im.thumbnail((790,490),Image.Resampling.LANCZOS)
        x=15+n%2*815;y=90+n//2*550;canvas.paste(im,(x,y));d.text((x,y+495),item['title'],font=font,fill='#173449')
    canvas.save(OUT/'00_closeups.jpg',quality=94,subsampling=0)
    print(json.dumps({'images':len(items),'size':[3840,2400],'output':str(dest)}))
if __name__=='__main__':main()
