"""공식 현재 main/후방 마커 원문을 읽기 전용 API로 stage 근거에 보존한다."""
from pathlib import Path
from datetime import datetime,timezone
import hashlib,json,subprocess

HERE=Path(__file__).resolve().parent
OUT=HERE/'sources'
OUT.mkdir(exist_ok=False)
REMOTE='MOSW626/istech-it-arena'
def api(path,raw=False):
    command=['gh','api','repos/'+REMOTE+'/'+path]
    if raw:command+=['-H','Accept: application/vnd.github.raw+json']
    return subprocess.check_output(command)

head=api('commits/main');commit=json.loads(head)['sha']
(OUT/'main_commit.json').write_bytes(head)
files={'MANUAL.md':api('contents/MANUAL.md?ref='+commit,True),
       'issue1.json':api('issues/1'),'issue1_comments.json':api('issues/1/comments?per_page=100')}
for name,data in files.items():(OUT/name).write_bytes(data)
repo=HERE.parents[3]
image=repo/'artifacts/markers/2026-09-28/id49_preview/marker_DICT_4X4_50_id49.png'
(OUT/'registered_gist_id49.png').write_bytes(image.read_bytes())
manifest={'observed_at_utc':datetime.now(timezone.utc).isoformat(),'repository':REMOTE,'main_commit':commit,
    'manual_url':f'https://github.com/{REMOTE}/blob/{commit}/MANUAL.md',
    'participant_source':'https://github.com/'+REMOTE+'/issues/1',
    'registered_gist_image_original':str(image.relative_to(repo)),
    'files_sha256':{p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in OUT.iterdir() if p.is_file()}}
(OUT/'manifest.json').write_text(json.dumps(manifest,indent=2),encoding='utf-8')
print(json.dumps({'main_commit':commit,'file_count':len(manifest['files_sha256'])}))
for item in [json.loads(files['issue1.json']),*json.loads(files['issue1_comments.json'])]:
    print(json.dumps({'url':item['html_url'],'created_at':item['created_at'],
                      'body':item['body']},ensure_ascii=False))
