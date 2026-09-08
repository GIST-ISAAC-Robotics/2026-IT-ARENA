from pathlib import Path
import json
root=Path(__file__).resolve().parents[2]
out=Path(__file__).parent
cases=json.loads((out/'results.json').read_text(encoding='utf-8'))
profiles=json.loads((out/'stereo_profiles.json').read_text(encoding='utf-8'))
review="""## 2026-09-06 현재 비교 기준과 추천

9월 5일 요청의 후속 수정이다. 최신 사용자 기준은 **LiDAR 포함 ToF0개, LiDAR 없는 RGB 또는 스테레오 구성 ToF4개**이다.
RGB 단독은 RGB로 조향한다는 뜻이며 거리 센서까지 없다는 뜻이 아니다. ToF4는 사용자가 설명한 측면 보완 구상이다.
RGB+ToF4 두 안과 D436+S3 20Hz 확장안을 포함해 엑셀은 총20개 구성이다.

사용자는 RGB로 노면·경로를 찾고 가까운 물체는 별도 센서로 감시하는 역할 분리를 제안했다.
이 구조는 검토 가능한 대안이며, 스테레오가 충돌 방지에 필수라는 전제는 철회한다.
기존 RGB-D 시뮬레이션에서도 RGB가 노면 후보를 찾고 깊이가 평지 여부를 보조했다.
같은 높이의 노면과 잔디는 깊이만으로 분리되지 않는다. 다만 RGB-only의 현장 성능은 아직 검증하지 않았다.

후방 식별마커를 찾으면 차량 위치 추적에 도움이 되지만, 마커 영역만 제외하면 차량 전체가 제거되는 것은 아니다.
마커 가림·측면 접근·차량 외곽과 경로의 겹침을 처리해야 한다. 확인한 규정의 후방 식별마커 의무와
팀별 ArUco 사전·ID 확정은 다르다(후자는 미확정). 근접 감지 성공도 회피 공간·반응·제동 시간을 확보했다는 뜻은 아니다.
RGB+측면 ToF4는 정면의 앞차와 정후방 거리 관측이 부족할 수 있으므로, 4개라는 수량을 충돌 방지 보증으로 쓰지 않는다.

**예산 의견은 기본 RGB안 총60만원(지원30+자체30), 깊이 보조 확장까지 확보할 때 총120만원(지원30+자체90)**이다.
전자는 C1+RGB/ToF0, 후자는 D436+C1/ToF0을 기준으로 한다.
OV2710의720p60fps는 미확인이고 B0385는 국내 품절 비교가이므로, RGB안은 구조·가격 비교 후보이며 확정 구매안이 아니다.
가격이 무제한이면 현재 비교 후보 중 D436+S3 20Hz/ToF0을 우선 시험한다.
RGB 경로 인식·깊이 보조·저상 LiDAR 근접 감시를 분리하며, 깊이는 추가 정보로 활용한다.
더 비싼 카메라라는 이유만으로 최종 성능이 높다고 단정하지 않는다.

| 비교안 | 센서 합계 | 차체35~40만원 합산 |
|---|---:|---:|
| OV2710+C1 / ToF0 | 162,360원 | 512,360~562,360원 |
| B0385+C1 / ToF0 | 195,360원 | 545,360~595,360원 |
| OV2710+ToF4 | 204,160원 | 554,160~604,160원 |
| B0385+ToF4 | 237,160원 | 587,160~637,160원 |
| 보유 D435i+C1 / ToF0 | 99,000원 | 449,000~499,000원 |
| D436+C1 / ToF0 | 696,000원 | 1,046,000~1,096,000원 |
| D436+S3 20Hz / ToF0 | 1,336,200원 | 1,686,200~1,736,200원 |

D436 VAT가 별도면 해당 두 안에59,700원을 더한다. D436+C1은1,105,700~1,155,700원,
D436+S3는1,745,900~1,795,900원으로 후자는 총180만원 수준이다.
센서 합계에는 임의 예비비·부자재·배송·ESP32-S3를 추가하지 않았다.
차체35/40만원은 하드웨어팀 확정 전 가정이고 보유 D435i는 규정 평가액 제외를 별도 합의해야 한다.

### 저상 LiDAR와10Hz의 의미

저상 LiDAR는 조향 입력 대신 근접 차량·벽 감시를 맡기는 검토안이다.
수평층을 개방해도 바퀴가 조향한 상태·지지대·센서 자기 가림, 피치/롤·턱과 상대차의 검출 가능한 높이를 확인해야 한다.
2D 수평면 바깥의 표적은 다중 영역 ToF와 관측 형태가 다르므로 완전한 대체를 입증한 것이 아니다.

[C1 공식 사양](https://www.slamtec.com/en/c1/spec)은8~12Hz(대표10), 각 간격0.72°, 최소0.05m, 거리 정확도±30mm이다.
흰 표적70%는최대12m, 검은 표적10%는최대6m이다.
같은 방향의 재관측 약100ms 동안 접근 상대속도0.5/1/2m/s이면 간격은5/10/20cm 줄어든다.
이는 노출·패킷·처리·제동·조향 지연을 제외한 계산이다. 완성 스캔을 모아 전달하는 드라이버는 점별 데이터 나이도 달라진다.
나란히 같은 속도로 움직이면 상대 접근속도가 낮아10Hz도 가능성이 있지만, 급격한 끼어들기에서 충분하다고 판정할 근거는 없다.
기존 고정 속도 조향 시험을 병주 충돌 회피 성능으로 전용하지 않는다.

[S3 공식 사양](https://www.slamtec.com/en/s3/spec)은10~20Hz,32k점/초이다.
20Hz의 각 간격은0.225°이고 같은 방향 재관측은약50ms이다. 대표10Hz의0.1125°와 혼동하지 않는다.
최소0.05m와거리 정확도±30mm는 C1보다 거리 정확도까지 개선됐다는 근거가 아니다.

### 카메라 사양 재확인에서 바뀐 판단

[Orbbec V1.6 표](https://www.orbbec.com/wp-content/uploads/2025/06/Gemini-330-series-Datasheet-V1.6.pdf)의
Gemini335L 최소17cm는424×240이다. 848×480에서는335L34cm,33518cm이고,
[RealSense 2026-03 표4-12](https://www.realsenseai.com/wp-content/uploads/2026/03/RealSense-D400-Series-Datasheet-Mar-2026.pdf)의
D436은 같은 해상도19.5cm이다. 단순17/20cm 비교를 바로잡았다. 335L의 긴 기선을 무조건 우선할 이유는 없다.
D436은 근접 깊이와 기존 RealSense 작업 활용에,335L은 검증된 RGB1280×800@60fps 모드에 장점이 있다.

D436 상품 페이지의RGB최대1280×800와최대60fps는 동시 조합으로 확정할 수 없다.
상세 데이터시트 표4-5의RGB는1280×720@30과848×480@60이 확인되며, Calibration1280×800@15/25를
일반 RGB 모드로 쓰지 않았다. 해당 불일치는 엑셀에 명시했다.
D455의RGB도최대해상도30fps일 뿐640×360에서는90fps를 지원한다.

Gemini335/335L의 깊이100fps는848×100의세로14°특수 모드이다.
D435i300fps·D455100fps도848×100특수 지원 사례이며 현행 펌웨어/SDK 확인 조건이다.
[제조사 지원 답변](https://support.realsenseai.com/hc/en-us/community/posts/360043522273-Intel-publish-white-paper-on-high-speed-300-FPS-depth-capture-for-D435?sort_by=votes).
일반 조향용 넓은 영상의90/60fps와 구분했다.

OAK RGB는 현재 RVC2 센서 드라이버의 해상도별 지원 모드를 기록했다.
깊이 입력 센서fps와 StereoDepth 출력fps를 합치지 않았다.
[현행 v3 RVC2 벤치](https://docs.luxonis.com/overview/toplevel-features/depth)의
Fast Density No Subpixel800p50/400p110fps는 공통 벤치로 표시했으며 완제품별 절대상한·RGB/AI동시 출력 보증이 아니다.
Lite480p의 정확한 깊이 최대fps는 미공개/미확인으로 남겼다. 추측으로 빈칸을 채우지 않았다.

"""
budget=root/'docs/hardware/SENSOR_BUDGET_CASES_2026_09_05.md'
t=budget.read_text(encoding='utf-8')
a=t.index('최신 사용자 기준은')
b=t.index('\nRGB는',a)
t=t[:a]+"최신 사용자 기준은 **LiDAR 포함 ToF0개, LiDAR 없는 RGB·스테레오 ToF4개**이다.\n총20개 안이며 RGB+ToF4와 빠른 LiDAR+깊이 혼합안을 포함한다. 세부 판단은 아래9월6일 갱신을 따른다.\n"+t[b:]
t=t.replace('C1+ToF2 포함 **232,760원**, S3+ToF2 포함 **872,960원**','C1+RGB/ToF0 **162,360원**, S3+RGB/ToF0 **802,560원**')
a=t.index('| 구성 | ToF 수량 | 센서 합계 |')
b=t.index('\n*B0385',a)
table='| 구성 | ToF 수량 | 센서 합계 |\n|---|---:|---:|\n'
for row in cases:
    table+=f'| {row[1]} | {row[2]} | {row[3]:,}원 |\n'
t=t[:a]+table+t[b:]
t=t.replace('총17개 조합·17개 안','총20개 조합·20개 안').replace('17개 센서 조합','20개 센서 조합').replace('총17개 안','총20개 안')
t=t.replace('## 총예산에 대응하는 협의','## 9월5일 총예산 협의 당시 기록')
t=t.replace('## IR 조건을 반영한 추천 수정','## 9월5일 IR 조건을 반영한 추천 당시 기록')
t=t.replace('## 빠른 LiDAR와 스테레오의 판단','## 9월5일 빠른 LiDAR와 스테레오 판단 당시 기록')
anchor='## 자료와 규칙 상태'
t=t.replace(anchor,review+anchor,1)
budget.write_text(t,encoding='utf-8')
stereo=root/'docs/sensors/STEREO_CAMERA_AND_MINIMAL_TOF.md'
t=stereo.read_text(encoding='utf-8')
t=t.replace('## 2026-09-05 IR 조건 재검토와 현재 추천','## 2026-09-05 IR 조건 재검토 당시 기록',1)
a=t.index('## 2026-09-05')
table='### 전체 카메라 거리·해상도별 모드\n\n최대값은 서로 동시에 성립하지 않을 수 있다. 아래표의 별도 조건과 제품 출처를 함께 읽는다.\n\n| 모델 | 거리 | 깊이 최대해상도/fps | 깊이 최대fps/해상도 | RGB 최대해상도/fps | RGB 최대fps/해상도 |\n|---|---|---|---|---|---|\n'
for c in profiles:
    vals=[c[k].replace('\n','<br>') for k in ['name','range','dres','dfps','rres','rfps']]
    table+='| '+' | '.join(vals)+' |\n'
table+='\n'
for c in profiles:
    table+='- '+c['name'].replace('\n',' ')+' 출처: '+c['source'].replace('\n',' ')+'\n'
t=t[:a]+review+table+'\n다음은 이전 판단 이력으로, 현재 수량·예산·최대FPS 판단은 위 갱신 내용을 따른다.\n\n'+t[a:]
stereo.write_text(t,encoding='utf-8')
context=root/'docs/PROJECT_CONTEXT.md'
t=context.read_text(encoding='utf-8').replace('최종 갱신: 2026-09-05','최종 갱신: 2026-09-06',1)
a=t.index('- [스테레오 후보 문서]')
b=t.index('- 국내 판매처 요청',a)
t=t[:a]+"""- [스테레오 후보 문서](sensors/STEREO_CAMERA_AND_MINIMAL_TOF.md)와 엑셀10종에 거리·모드별 최소거리, 깊이/RGB 최대해상도에서fps 및 최대fps에서해상도를 추가했습니다. D436 RGB 상품표/상세표 불일치,335L의848×480 최소34cm 대D43619.5cm, 특수 좁은 영상의100/300fps와OAK 센서 입력/깊이 벤치의 차이를 구분합니다.
- 최신 사용자 기준은 **LiDAR가 있으면ToF0, LiDAR 없는RGB 또는스테레오는ToF4**입니다. RGB+ToF4 두 안과D436+S3 20Hz를 포함한20개 안으로 변경했습니다. RGB/RGB-D 조향과 저상LiDAR 근접 감시를 분리하고 수평층 개방 배치를 검토합니다. 시뮬레이션·팀 최종 장착은 변경하지 않았습니다.
- 사용자 후속 아이디어에 따라 RGB 주행+근접센서를 기본 후보로 검토하고 깊이카메라를 필수 전제로 두지 않습니다. 마커 영역 제외만으로 차량 전체 가림을 해결하거나 근접 검출만으로 충돌 방지가 보장되지는 않습니다. 후방 식별마커의ArUco 규격도 미확정입니다.
- 최신 예산 의견은 RGB+C1/ToF0 기본안 총60만원(지원30+자체30), 깊이 확장까지 확보할 때D436+C1/ToF0 총120만원(지원30+자체90)입니다. 무제한이면 현재 비교 후보 중D436+S3 20Hz를 우선 시험하며 차체35~40만원·D436VAT별도까지 가정하면1,745,900~1,795,900원입니다. 기존9월5일 단가를 유지했고 카메라fps·재고 및 하드웨어는 확정 전입니다.

"""+t[b:]
t=t.replace('C1+ToF2 포함232,760원/S3+ToF2 포함872,960원','최신 ToF0 기준C1+RGB162,360원/S3+RGB802,560원')
t=t.replace('LiDAR ToF2·스테레오 단독 ToF4를 적용한 17개 견적','LiDAR ToF0·RGB/스테레오 단독 ToF4를 적용한20개 견적')
t=t.replace('S3+Arducam B0385+ToF2 905,960원','S3+Arducam B0385/ToF0 835,560원')
context.write_text(t,encoding='utf-8')
activity=root/'docs/activity/2026-09-06.md'
entry="""## 센서 거리·FPS 상세화와 RGB 주행 예산 재검토

- 9월5일 시작한 사용자 요청을 자정 이후 마무리했다. 엑셀 스테레오10종의 거리·해상도별최소거리, 깊이/RGB 각각 최대해상도에서fps와최대fps에서해상도를 정리했다. 제조사 PDF와 현행 센서 드라이버·벤치 자료를 대조하고 조건·출처를 각 행에 남겼다.
- D436 RGB 최대1280×800의fps는 상품·상세표 불일치로 미확인 표시했다. 상세 RGB1280×720@30/848×480@60,335L의깊이848×480 최소34cm와D43619.5cm, D455RGB640×360@90, 좁은848×100특수 깊이 모드를 보완했다. OAK센서 입력과깊이출력을 구분하고 미공개 최대값을 추측하지 않았다.
- 후속 사용자의RGB 메인 주행 아이디어를 반영해 깊이카메라를 필수 전제로 둔 추천을 수정했다. RGB로노면·경로, 별도 거리센서로근접 간격을 감시하는 구조를 검토한다. 마커 가림·차량 전체외곽·전방 거리·반응시간은 별도 조건이다.
- 최종 수량은LiDAR포함ToF0, LiDAR없는RGB/스테레오ToF4이다. RGB+ToF4두 안과D436+S3 20Hz를 추가해20개 구성으로 갱신했다. 기존 단가·차체 가정·부자재/예비비 제외를 유지했다.
- C1+RGB는OV2710기준162,360원, B0385기준195,360원이다. RGB+ToF4는각204,160/237,160원으로같은RGB에서C1안이41,800원낮다. OV2710의720p60미확인/B0385국내품절조건을유지했다.
- 현재 의견은RGB+C1기본총60만원, 깊이확장D436+C1총120만원, 예산무제한후보D436+S3 20Hz총180만원수준이다. 차체35~40만원과D436VAT별도가정임을명시했다. 10Hz근접감시충분여부는상대접근속도·여유거리·스캔/처리/제동지연과저상시야가림에달려있다.
- 20개 합계·차체합산·가격순SORT와 단가/수량 연동,10종사양기록,모든LiDAR행ToF0/RGB추가행ToF4,기존차체합계보존을확인했다. 수식오류0건이며 변경 화면을 확인했다.
- 최신 배포 파일명은사용자이전링크유지를위해IT_ARENA_센서_예산안_2026-09-05_ToF수정.xlsx를유지하고내용갱신일은9월6일로표시했다. 새비교예산은개인검토안이며 시뮬레이션변경·팀구매결정·구매·외부게시는수행하지않았다.

"""
if activity.exists():
    activity.write_text(activity.read_text(encoding='utf-8')+'\n'+entry,encoding='utf-8')
else:
    activity.write_text('# 2026-09-06 활동 기록\n\n'+entry,encoding='utf-8')
print('Project budget, stereo profiles, context and 2026-09-06 activity updated.')

