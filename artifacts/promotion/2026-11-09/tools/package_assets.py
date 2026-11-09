"""선별 이미지의 원본 대조, 한국어 미리보기, 전달용 ZIP 생성."""
from pathlib import Path
import hashlib
import json
import zipfile
import math
from PIL import Image,ImageDraw,ImageFont

OUT=Path(__file__).resolve().parents[1]
ROOT=OUT.parents[2]
def sha(p):
    return hashlib.file_digest(p.open('rb'),'sha256').hexdigest()
def read(p):return json.loads(p.read_text(encoding='utf-8'))
figures=read(OUT/'figure_manifest.json');renders=read(OUT/'render_manifest.json')
closeups=read(OUT/'closeup_manifest.json')['items'] if (OUT/'closeup_manifest.json').exists() else []
items=renders['items']+figures['items']+closeups
count=len(items)
assert count==20+len(closeups)
for item in items:
    p=OUT/item['file']
    with Image.open(p) as im:
        im.verify()
    with Image.open(p) as im:
        assert [im.width,im.height]==[item['width'],item['height']],item['file']
    assert sha(p)==item['sha256'],item['file']
    if item.get('source_repo') or item.get('source_absolute'):
        src=ROOT/item['source_repo'] if item.get('source_repo') else Path(item['source_absolute'])
        assert src.exists() and sha(src)==item['source_sha256']==item['sha256'],item['file']
    elif isinstance(item.get('source_sha256'),dict):
        for name,digest in item['source_sha256'].items():assert sha(ROOT/name)==digest,name
for name,digest in figures['sources'].items():
    assert sha(ROOT/name)==digest,name

font_path='C:/Windows/Fonts/malgun.ttf'
font=ImageFont.truetype(font_path,23);small=ImageFont.truetype(font_path,18);big=ImageFont.truetype(font_path,37)
lookup={i['file']:i for i in items}
preferred=[
 '02_speed_maps/curved_course_speed_dark.png',
 '02_speed_maps/course_speed_dark.png',
 '03_lidar/lidar_paths_dark.png',
 '01_renders/07_vehicle_rear_legacy.png',
 '01_renders/02_irregular_pass.png',
 '01_renders/06_track_overview_oblique.png']
if closeups:
    preferred=['05_closeups/01_racing_front_close.png','05_closeups/05_solo_rear_close.png',
               '02_speed_maps/curved_course_speed_dark.png','03_lidar/lidar_paths_dark.png',
               '05_closeups/07_original_pass_close.png','01_renders/06_track_overview_oblique.png']

def contact(items,path,cols,cellw,cellh,heading):
    im=Image.new('RGB',(cols*cellw+48,math.ceil(len(items)/cols)*cellh+135),'#edf1f4')
    draw=ImageDraw.Draw(im);draw.text((30,25),heading,font=big,fill='#173449')
    draw.text((32,80),'개별 원본 파일은 폴더에서 열어 사용하세요. 아래 이미지는 선택용 미리보기입니다.',font=small,fill='#5b6f7e')
    for n,item in enumerate(items):
        x=24+(n%cols)*cellw;y=122+(n//cols)*cellh
        draw.rounded_rectangle((x+5,y+5,x+cellw-8,y+cellh-12),radius=9,fill='white')
        source=Image.open(OUT/item['file']).convert('RGB');source.thumbnail((cellw-34,cellh-102),Image.Resampling.LANCZOS)
        im.paste(source,(x+(cellw-source.width)//2,y+15+(cellh-102-source.height)//2))
        title=item['title'].replace(' · 어두운 배경',' · Dark').replace(' · 밝은 배경',' · Light')
        while draw.textlength(title,font=font)>cellw-30:title=title[:-2]+'…'
        draw.text((x+16,y+cellh-76),title,font=font,fill='#173449')
        draw.text((x+16,y+cellh-43),Path(item['file']).name,font=small,fill='#617482')
    im.save(path,quality=94,subsampling=0)

contact([lookup[p] for p in preferred],OUT/'00_recommended.jpg',3,700,660,'IT ARENA · 포스터 추천 이미지 6장')
ordered=[lookup[p] for p in preferred]+[i for i in items if i['file'] not in preferred]
contact(ordered,OUT/'00_all_images.jpg',4,600,520,f'IT ARENA · 전체 이미지 {count}장')
if closeups:
    contact(closeups,OUT/'00_closeups.jpg',3,800,620,'차량 근접 컷 · 신규 8장')
    contact([i for i in closeups if '/04_' in i['file'] or '/05_' in i['file'] or '/06_' in i['file']],
            OUT/'00_solo_shots.jpg',3,1000,760,'단독 주행 · 앞 / 뒤 / 코너')
# 실행 담당의 검토용 시트도 한글이 표시되는 폰트로 갱신한다.
contact(renders['items'],OUT/'01_renders/review_contact_sheet.jpg',2,650,440,'차량 · 트랙 · 주행 장면')

readme='''# IT ARENA 포스터 이미지 모음 — 2026-11-09

포스터에 바로 넣을 수 있는 **PNG 20장**, 새 그래프의 **벡터 SVG 8장**, 추천/전체 미리보기입니다. 렌더 원본은 원래 해상도를 유지했고 인위적으로 확대하지 않았습니다. 모든 자료는 시뮬레이션 또는 저장 기록 분석이며 실차 사진은 없습니다.

## 먼저 고를 이미지

- `00_recommended.jpg`: 추천 6장. `00_all_images.jpg`: 전체 20장.
- 곡선 코스 중심 이미지: `02_speed_maps/curved_course_speed_dark.png`(밝은 포스터는 `_light`).
- 차량이 큰 장면: `01_renders/07_vehicle_rear_legacy.png`.
- 기술 소개: `03_lidar/lidar_paths_dark.png`(밝은 포스터는 `_light`).
- 두 차량 장면: `01_renders/02_irregular_pass.png`.

## 구성과 해상도

| 폴더 | 내용 | 수량 |
|---|---|---:|
| `01_renders` | 차량 2장·트랙 전경 2장·추월/우회 4장 | 8 PNG |
| `02_speed_maps` | 최신 직각·변경 전 곡선 코스 각각 Dark/Light 지도와 지도+속도 프로필 | 6 PNG + 6 SVG |
| `03_lidar` | 최신 E1 관측점·지역 경로, Dark/Light | 2 PNG + 2 SVG |
| `04_reference_plots` | 잡음·가림 벽·코너·센서 배치의 기존 분석 그림 | 4 PNG |

속도 지도는 3600×4200, 지도+속도 프로필은 4800×3000, LiDAR 그림은 4800×2400입니다. 새 PNG는 300 dpi이며 SVG는 글꼴을 윤곽선으로 저장했습니다. SVG를 지원하는 편집기에서는 확대해도 선과 글자가 선명합니다. 교통 렌더는 1280×720, 트랙 전경은 1920×1200·1400×2150, 차량 세로 이미지는 1600×2000입니다. `review_contact_sheet.jpg`는 검토용이므로 20장에 포함하지 않습니다.

## 속도 지도 데이터

| 구분 | 코스·기록 | 지도에 표시한 기록 |
|---|---|---|
| 최신 직각 코스 | v2026.11.06, 11/9 E1 `e_all_posts_blank` | 출발 시뮬 시각 8.313 s~38.230 s, 651표본, 진행 46.65 m, 최고 10.251 km/h |
| 변경 전 곡선 코스 | v2026.11.01, 11/5 `solo_straight_closed_loop_v1/run1` | 출발 시뮬 시각 8.255 s~40.793 s, 708표본, 진행 48.65271 m, 최고 10.274 km/h |

각 실행의 노면·잔디·벽 OBJ와 같은 실행의 저장된 x·y, `abs(dynamics.truth_longitudinal_mps) × 3.6`을 사용했습니다. 지도 위 색은 차체의 종방향 속도이며 명령 속도나 바퀴 회전 속도가 아닙니다. 표본 위치를 직선으로 연결하고 선분 양 끝 속도의 평균으로 색을 정했습니다. 두 코스 모두 같은 0~11 km/h 눈금이며 속도를 평활화하거나 느린 구간을 삭제하지 않았습니다. 마지막 직선에서는 한 랩 뒤 추가 진행 궤적이 초기 궤적과 일부 겹칩니다.

두 지도는 서로 다른 코스·날짜·조건이므로 직접적인 알고리즘 성능 비교표가 아닙니다. 곡선 코스는 직각 코스로 바꾸기 전 마지막 완료 단독 증속 기록을 선정했습니다. 지도를 현재 기록에 맞추어 변형하거나 현재 속도를 옛 코스에 옮겨 그리지 않았습니다. 평가용 지도·정답 자세는 이 오프라인 그림에만 쓰였으며 제어 입력을 추가하지 않았습니다.

대시보드의 시간은 **출발 허가부터 마지막 STOP 전 기록 표본**까지입니다(최신 29.917초, 곡선 32.538초). 공식 계시선 랩타임이 아닙니다. 옛 보고서의 31.850초는 초록 평가 표본부터 거리 한랩까지의 별도 정의입니다. STOP 뒤의 제동 구간은 이 그림에 없으며 기존 제동 한계를 해소했다는 뜻도 아닙니다.

## LiDAR와 렌더의 범위

- 새 LiDAR 그림은 E1의 시뮬 시각 14.52/23.12초 스캔을 저장 운동 이력으로 보정하고, 당시 출력에 기록된 `path_points`를 그대로 표시했습니다. 계획 경로는 실제 주행 궤적이 아닙니다. 관측점이 없는 곳을 자유 공간으로 채우지 않았고 새로운 제어 판단은 실행하지 않았습니다.
- 교통 렌더 4장은 기존 소개 영상용 **기록 자세 재렌더의 원본 PNG**입니다. 새 주행이 아니며 바퀴 회전·앞바퀴 조향은 재현하지 않았습니다.
- `vehicle_*_legacy`는 9월 구형 차체·구형 코스 렌더입니다. 현재 기구 설계 확정본으로 사용하지 않습니다.
- 기존 분석 4장은 당시 진단·후보 비교·임시 배치 연구입니다. 해당 그림만으로 기능 완성이나 실차 검증을 주장하지 않습니다.

## 파일별 설명과 출처

'''
for item in ordered:
    readme+=f"- [{item['title']}]({item['file']}) — {item['width']}×{item['height']}. {item['note']}\n"
readme+='''
원본 경로·SHA-256·생성 방식은 `render_manifest.json`, `figure_manifest.json`, 전체 파일 목록/검사는 `asset_manifest.json`, `qa_report.json`에 있습니다. 재작성 도구는 `tools/build_figures.py`, 전달 묶음 도구는 `tools/package_assets.py`입니다. 원시 센서/상세 주행 기록은 전달 ZIP에 넣지 않았습니다. 기존 증거·제어 코드·월드·현재 시험은 변경하지 않았고 새 시뮬레이션을 실행하지 않았습니다.
'''
if closeups:
    readme=readme.replace('**PNG 20장**',f'**PNG {count}장**').replace('전체 20장',f'전체 {count}장').replace('20장에 포함',f'{count}장에 포함')
    readme=readme.replace('| `04_reference_plots` |', '| `05_closeups` | 차량 경쟁 3장·단독 주행 3장·원 시험 상대 근접 2장 | 8 PNG |\n| `04_reference_plots` |')
    readme=readme.replace('## 먼저 고를 이미지','''## 추가한 차량 근접 컷

`05_closeups/`에 **3840×2400 PNG 8장**을 추가했습니다. `00_closeups.jpg`에서 한 번에 볼 수 있으며 `00_solo_shots.jpg`는 단독 주행 3장만 모은 미리보기입니다.

- `01`~`03`: 두 차량을 앞·뒤·사선으로 크게 담은 홍보용 렌더. 기록된 자차·상대의 위치와 방향은 유지하되, 11/3 박스형 차량의 파랑·빨강 외관을 사용했습니다. 원래 시험 상대는 오목 블록입니다. 상대의 중복 식별판은 제외했습니다.
- `04`~`06`: 단독 직선·코너 주행 기록의 차량과 주변 트랙을 여유 있게 담은 컷. 11/3 박스형 차체를 바탕으로 LiDAR의 시각 간섭을 없애고 카메라 받침·렌즈·휠 허브를 더한 홍보용 외관입니다.
- `07`~`08`: 시험 당시의 파란 차량·빨간 오목 블록을 그대로 두고 카메라만 가까이 배치한 컷.

모두 원본 3D 장면을 가까운 카메라에서 고해상도로 다시 렌더했습니다. 저해상도 이미지를 늘린 파일이 아닙니다. 렌더 전용 정적 월드에 저장된 자세를 놓았으며 물리·제어를 다시 실행하지 않았습니다. 두 차량의 기록된 평면 위치·방향은 유지했습니다. 외관을 바꾼 홍보용 6장은 원 시험 화면이나 추가 주행 성공의 근거로 사용하지 않습니다. 기존 20장과 첫 ZIP은 그대로 보존했습니다.

**앞바퀴 조향:** 단독 주행 3장은 `trace.json`의 `dynamics.left_steering_rad/right_steering_rad`를 해당 시각으로 보간했습니다. 이 값은 `single_motor_drive.cpp`가 Gazebo 조향 관절의 실제 위치에서 읽은 값입니다. 코너 컷은 t=13.913초로 안쪽 약 17.13°, 바깥쪽 약 12.91°로 꺾입니다. 교통 기록에는 실제 관절각이 보존되지 않아 `actuation_trace.json`의 최종 MCU 조향 명령과 축거/윤거 135 mm로 Ackermann 좌우 목표를 계산해 표시했습니다. 두 방식을 manifest에 구분했습니다. 바퀴의 구름 회전 애니메이션은 추가하지 않았으며 홍보용 상대 차량의 앞바퀴는 해당 직선에 맞춰 중립으로 표시했습니다.

재렌더 도구는 `tools/build_closeup_scenes.py`, `tools/render_closeups.sh`, `tools/collect_closeups.py`입니다. 저장소의 `artifacts/videos/2026-11-09/feature_showcase/tools/build_replay_scene.py`와 기존 자세 재생 플러그인을 사용합니다. 각 컷의 입력 해시·자세 시각·조향 출처·카메라·사용 프레임 경로는 `closeup_manifest.json`에 남겼습니다. 구도 변경이 없는 그룹은 기존 고해상도 프레임을 재사용하며 수집 시 입력 일치를 확인합니다.

## 먼저 고를 이미지''')
    readme=readme.replace('새 시뮬레이션을 실행하지 않았습니다.','새 주행 시험은 실행하지 않았습니다. 근접 컷에는 렌더 전용 Gazebo 서버를 사용했습니다.')
(OUT/'README.md').write_text(readme,encoding='utf-8')
(OUT/'asset_manifest.json').write_text(json.dumps(dict(created_date='2026-11-09',image_count=count,vector_count=8,recommended=preferred,items=ordered),ensure_ascii=False,indent=2),encoding='utf-8')
qa=dict(image_count=count,vector_count=8,image_decode_ok=True,dimensions_match=True,source_copy_sha256_match=True,
        figure_source_hashes_match=True,raw_logs_in_archive=False,new_driving_test=False,render_only_gazebo_groups=3 if closeups else 0,
        latest=dict(samples=figures['plotted_sample_count'],peak_kmh=figures['peak_kmh']),legacy=figures['legacy'])
(OUT/'qa_report.json').write_text(json.dumps(qa,ensure_ascii=False,indent=2),encoding='utf-8')
paths=[OUT/'README.md',OUT/'00_recommended.jpg',OUT/'00_all_images.jpg']
paths.extend(OUT/name for name in ['asset_manifest.json','figure_manifest.json','render_manifest.json','qa_report.json'])
if closeups:paths.extend([OUT/'closeup_manifest.json',OUT/'00_closeups.jpg',OUT/'00_solo_shots.jpg'])
for i in ordered:
    paths.append(OUT/i['file'])
    if i.get('vector'):paths.append(OUT/i['vector'])
paths.extend(p for p in (OUT/'tools').iterdir() if p.suffix in ('.py','.sh'))
archive=OUT/('IT_ARENA_poster_assets_2026-11-09_v2.zip' if closeups else 'IT_ARENA_poster_assets_2026-11-09.zip')
with zipfile.ZipFile(archive,'w',zipfile.ZIP_DEFLATED,compresslevel=7) as z:
    for p in paths:z.write(p,p.relative_to(OUT).as_posix())
with zipfile.ZipFile(archive) as z:
    assert z.testzip() is None
    for p in paths:
        assert hashlib.sha256(z.read(p.relative_to(OUT).as_posix())).hexdigest()==sha(p)
print(json.dumps({'images':count,'vectors':8,'zip_mb':round(archive.stat().st_size/1e6,2),'zip_entries':len(paths),'archive':str(archive)},ensure_ascii=False))
