from pathlib import Path
import json,re
root=Path(__file__).resolve().parents[2]
out=Path(__file__).parent
rows=json.loads((out/'results.json').read_text(encoding='utf-8'))
p=root/'docs/hardware/SENSOR_BUDGET_CASES_2026_09_05.md'
s=p.read_text(encoding='utf-8')
start=s.index('모든 센서 조합에 ToF 2개·4개안')
end=s.index('\nRGB는',start)
s=s[:start]+'최신 사용자 기준은 **LiDAR 포함 구성 ToF 2개, 스테레오 단독 구성 ToF 4개**이다.\nLiDAR+스테레오 혼합 구성도 LiDAR 기준으로2개를 적용했다. 총17개 안이며\n이전 조합별2/4 중복 비교를 대체한다. 장착 방향이나 팀 최종 설계의 확정은 아니다.\n'+s[end:]
start=s.index('| 구성 | ToF 2개 센서 합계 |');end=s.index('\n*B0385',start)
table='| 구성 | ToF 수량 | 센서 합계 |\n|---|---:|---:|\n'
for r in rows: table+=f'| {r[1].rsplit(" + ToF ",1)[0]} | {r[2]} | {r[3]:,}원 |\n'
s=s[:start]+table+s[end:]
s=s.replace('17개 센서 조합에 각각 ToF2/4를 적용한 총34개 안','17개 센서 조합에 LiDAR ToF2·스테레오 단독 ToF4를 적용한 총17개 안')
s=s.replace('IR 탑재 Gemini2+ToF2는874,700~924,700원. Gemini335+ToF2는926,400~976,400원. OAK Lite+ToF2는805,301~855,301원','IR 탑재 Gemini2+ToF4는945,100~995,100원. Gemini335+ToF4는996,800~1,046,800원. OAK Lite+ToF4는875,701~925,701원')
s=s.replace('C1+OV2710+ToF2는582,760~632,760원, ToF4는653,160~703,160원.','C1+OV2710+ToF2는582,760~632,760원.')
s=s.replace('IT_ARENA_센서_예산안_2026-09-05.xlsx','IT_ARENA_센서_예산안_2026-09-05_ToF수정.xlsx')
p.write_text(s,encoding='utf-8')
p=root/'docs/PROJECT_CONTEXT.md';s=p.read_text(encoding='utf-8')
s=s.replace('17개 조합·34개 안으로 확장했습니다. 각 조합에ToF2/4를 붙여 차액70,400원을 유지하고 각 안의 코멘트를 달았습니다.','17개 조합을 유지하고 최신 사용자 기준에 따라 LiDAR 포함 구성은ToF2, 스테레오 단독은ToF4로 총17개 안을 산정했습니다. 혼합 구성도LiDAR 기준2개입니다. 각 안의 코멘트와 가격순 표도 갱신했습니다.')
s=s.replace('각 조합마다 ToF2/4를 적용한 34개 견적','LiDAR ToF2·스테레오 단독 ToF4를 적용한 17개 견적').replace('OAK FF+ToF2/4 455,301/525,701원','OAK FF+ToF4 525,701원')
s=s.replace('## 현재 우선 작업: 센서 예산 협의','## 현재 우선 작업: 센서 예산 협의\n\n- 최신 엑셀은 `outputs/01a07090-budget/IT_ARENA_센서_예산안_2026-09-05_ToF수정.xlsx`입니다. 기존 파일이 열려 있어 별도 수정본으로 저장했습니다.')
p.write_text(s,encoding='utf-8')
p=root/'docs/sensors/STEREO_CAMERA_AND_MINIMAL_TOF.md';s=p.read_text(encoding='utf-8')
s=s.replace('전체17개 센서 조합을 ToF2/4로 통일한34개 견적이며','최신 사용자 기준에 따라 전체17개 조합을 LiDAR 포함ToF2·스테레오 단독ToF4로 산정한17개 견적이며')
s=s.replace('## 2026-09-05 IR 조건 재검토와 현재 추천','## 2026-09-05 IR 조건 재검토와 현재 추천\n\n최신 수량은 LiDAR 포함 구성2개, 스테레오 단독 구성4개이다. 혼합 구성은LiDAR 기준2개로 산정한다. 아래 과거2/4 병렬 비교는 이 기준으로 대체한다.')
p.write_text(s,encoding='utf-8')
with (root/'docs/activity/2026-09-05.md').open('a',encoding='utf-8') as f:
 f.write('\n\n## 센서 방식별 ToF 수량 수정\n\n- 사용자 지시에 따라 LiDAR 포함은ToF2, 스테레오 단독은ToF4로 수정했다. 혼합 구성도LiDAR 기준2개로 적용한다고 사용자에게 설명했다.\n- 기존34개 중 중복 수량안을 정리해17개 안으로 줄이고 코멘트·가격순 정렬·예산 조건의 합계를 갱신했다. 단가·하드웨어·카메라 사양은 유지했다.\n- 17개 합계·단가/수량 변경 시 정렬 연동·저장된SORT와정렬 결과·차체 합계 보존을 확인했다.\n- 기존 엑셀이열려 있어덮어쓰기EBUSY가 발생해 `_ToF수정.xlsx`로 별도 저장했다. 기존 파일과직전복사본을 보존했다. 시뮬레이션은 변경하지 않았다.\n')
assert len(rows)==17
assert all(r[2]==(2 if ('C1' in r[1] or 'RPLIDAR' in r[1]) else 4) for r in rows)
print('17 sensor rules verified; documentation synchronized')
