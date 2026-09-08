from pathlib import Path
import json, shutil
root=Path.cwd();out=root/'outputs/01a07090-budget'
rows=json.loads((out/'results.json').read_text(encoding='utf-8'))
for rel in ['docs/hardware/SENSOR_BUDGET_CASES_2026_09_05.md','docs/hardware/RGB_CAMERA_OPTIONS_2026_09_05.md']:
 p=root/rel;backup=out/'archive'/('before_tof_2_4_'+p.name)
 if not backup.exists():shutil.copyfile(p,backup)
p=root/'docs/hardware/SENSOR_BUDGET_CASES_2026_09_05.md';t=p.read_text(encoding='utf-8')
start=t.index('- 보유 D435i+ToF4:');end=t.index('\nRGB는',start)
t=t[:start]+'''모든 센서 조합에 ToF 2개·4개안을 동일하게 적용한다. 1개·5개안은 현재 견적에서 제외한다.
총11개 조합·22개 안이며, 같은 조합에서 두 개 추가 비용은 **70,400원**이다.
이것은 예산 비교 기준의 통일이며 실차 장착 방향이나 센서 수량의 최종 확정이 아니다.
''' +t[end:]
start=t.index('| 구성 | ToF 수 |');end=t.index('\n*B0385',start)
table='| 구성 | ToF 2개 센서 합계 | ToF 4개 센서 합계 |\n|---|---:|---:|\n'
for i in range(0,len(rows),2):
 name=rows[i][1].rsplit(' + ToF ',1)[0];table+=f'| {name} | {rows[i][3]:,}원 | {rows[i+1][3]:,}원 |\n'
t=t[:start]+table+t[end:]
t=t.replace('국내 저가 카메라 2개 안을 포함해 총15개 구성이다.','국내 저가 카메라를 포함한 11개 센서 조합에 각각 ToF2/4를 적용한 총22개 안이다.')
t=t.replace('| 총60만·보유 불가 | C1+B0385+ToF1은580,560~630,560원, ToF2는615,760~665,760원. 카메라 품절 조건부 |','| 총60만·보유 불가 | C1+OV2710+ToF2는582,760~632,760원, ToF4는653,160~703,160원. 카메라720p60fps 미확인 |')
p.write_text(t,encoding='utf-8')
p=root/'docs/hardware/RGB_CAMERA_OPTIONS_2026_09_05.md';t=p.read_text(encoding='utf-8')
start=t.index('| 구성 | 센서 단품 합계 |');end=t.index('\nB020101로 교체',start)
table='| 구성 | ToF 2개 센서 합계 | ToF 4개 센서 합계 |\n|---|---:|---:|\n'
for i in [2,4,16,18]:
 name=rows[i][1].rsplit(' + ToF ',1)[0];table+=f'| {name} | {rows[i][3]:,}원 | {rows[i+1][3]:,}원 |\n'
t=t[:start]+table+t[end:];t=t.replace('## 예산 반영\n','## 예산 반영\n\n모든 센서 조합을 ToF 2개·4개로 통일해 비교한다. 1개·5개안은 현재 표에서 제외한다.\n')
p.write_text(t,encoding='utf-8')
p=root/'docs/PROJECT_CONTEXT.md';t=p.read_text(encoding='utf-8').replace('ToF 수량별 15개 견적','각 조합마다 ToF2/4를 적용한 22개 견적')
tag='## 현재 우선 작업: 센서 예산 협의\n';t=t.replace(tag,tag+'\n- 사용자 후속 요청으로 11개 센서 조합을 각각 ToF 2개·4개 한 쌍으로 통일했습니다. 1개·5개안은 견적에서 제외하고 22개 안의 계산을 갱신했습니다. 각 쌍의 차액은70,400원입니다. 예산 비교 기준이며 기존 시뮬레이션·실차 장착 설계는 변경하지 않았습니다.\n')
p.write_text(t,encoding='utf-8')
with (root/'docs/activity/2026-09-05.md').open('a',encoding='utf-8') as f:f.write('''

## 센서 조합별 ToF 2개·4개 비교 통일

- 사용자의 최종 지시에 따라 각 센서 조합에 동일하게 ToF2/4를 적용했다. 4개만 남기라는 중간 요청은 후속 2개 추가 요청으로 대체되었다.
- 11개 조합·22개 안을 같은 조합끼리 인접 배치하고 동일 배경색으로 묶었다. 기존 1개·5개 사례는 현재 견적에서 제외했다.
- 22개 센서 합계·차체 가정 합산, 각 쌍 차액70,400원, 단가·수량 연동과 차체 시트 보존을 확인했다. 수식 오류0건 및 변경 표의 표시를 확인했다.
- 단가·재고 조건은 이전 확인값을 유지했다. 예산 비교 범위만 변경하며 기존 시뮬레이션·실차 장착 결정을 변경하지 않았다.
''')
print('관련 문서 갱신 완료')
