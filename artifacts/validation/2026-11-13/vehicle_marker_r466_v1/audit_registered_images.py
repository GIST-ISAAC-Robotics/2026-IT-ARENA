"""제출 PNG의 검은 코드 비율/사전을 확인한다. 원 이미지 파일은 수정하지 않는다."""
from pathlib import Path
import hashlib,json,re,subprocess,urllib.request
import cv2
import numpy as np

HERE=Path(__file__).resolve().parent
OUT=HERE/'sources'
meta=json.loads((OUT/'manifest.json').read_text())
commit=meta['main_commit']
generator=subprocess.check_output(['gh','api',f'repos/MOSW626/istech-it-arena/contents/tools/make_vehicle_marker.py?ref={commit}',
                                  '-H','Accept: application/vnd.github.raw+json'])
(OUT/'official_make_vehicle_marker.py').write_bytes(generator)
comments=json.loads((OUT/'issue1_comments.json').read_text())
names={5772491077:'UNIST',5779593670:'POSTECH',5828019968:'KAIST',5858748518:'GIST',5978592693:'DGIST'}
result=[]
for item in comments:
    team=names.get(item['id'])
    if team is None:continue
    url=re.findall(r'https://github.com/user-attachments/assets/[a-f0-9-]+',item['body'])[0]
    with urllib.request.urlopen(urllib.request.Request(url,headers={'User-Agent':'IT-ARENA-read-only-audit'}),timeout=30) as response:
        content=response.read(10_000_001)
    assert len(content)<=10_000_000
    target=OUT/(team.lower()+'_submitted_image.bin');target.write_bytes(content)
    gray=cv2.imdecode(np.frombuffer(content,np.uint8),cv2.IMREAD_GRAYSCALE)
    record={'team':team,'comment_url':item['html_url'],'image_url':url,'file':target.name,
            'sha256':hashlib.sha256(content).hexdigest(),'decoded':gray is not None,'ids':{}}
    if gray is not None:
        record['shape_px']=list(gray.shape)
        y,x=np.where(gray<128)
        if len(x):
            record['dark_bbox_xywh_px']=[int(x.min()),int(y.min()),int(np.ptp(x)+1),int(np.ptp(y)+1)]
            record['black_code_m_if_entire_image_printed_50mm']=[.05*(np.ptp(x)+1)/gray.shape[1],.05*(np.ptp(y)+1)/gray.shape[0]]
        # 7x7 원본은 quiet zone이 없을 수 있다. 진단 패딩은 파일에 저장하지 않는다.
        for dictionary in ('DICT_4X4_50','DICT_7X7_100'):
            d=cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco,dictionary))
            for padded in (False,True):
                image=cv2.copyMakeBorder(gray,30,30,30,30,cv2.BORDER_CONSTANT,value=255) if padded else gray
                corners,ids,_=cv2.aruco.ArucoDetector(d).detectMarkers(image)
                record['ids'][dictionary+('_diagnostic_white_border' if padded else '_raw')]=[] if ids is None else ids.flatten().tolist()
        if team=='GIST':record['matches_repo_registered_png']=content==(OUT/'registered_gist_id49.png').read_bytes()
    result.append(record)
(OUT/'registered_images_audit.json').write_text(json.dumps(result,indent=2),encoding='utf-8')
print(json.dumps(result,indent=2))
