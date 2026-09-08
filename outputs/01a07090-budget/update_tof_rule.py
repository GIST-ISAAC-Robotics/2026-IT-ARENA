from pathlib import Path
import shutil
p=Path(__file__).parent
book=p/'IT_ARENA_센서_예산안_2026-09-05.xlsx'
backup=p/'archive/before_tof_by_sensor.xlsx'
if not backup.exists(): shutil.copy2(book,backup)
f=p/'value_camera.mjs'
s=f.read_text(encoding='utf-8')
s=s.replace("=>[2,4].map(count=>", "=>(refs.includes(6)||refs.includes(11)?[2]:[4]).map(count=>")
s=s.replace('ToF2: 좌우 우선 배치 시 ToF의 후방 보완은 생략.','ToF2: LiDAR 구성의 보완 센서이며 장착 방향은 별도 결정.')
s=s.replace('17개 조합 × ToF2/4','17개 조합 · LiDAR ToF2 / 스테레오 단독 ToF4')
s=s.replace('구성 비교의 34개 안','구성 비교의 17개 안')
s=s.replace('2개·4개는 동일 조건 예산 비교이며 확정 장착값이 아닙니다.','LiDAR 포함 구성은2개, 스테레오 단독 구성은4개로 산정합니다. 혼합 구성도LiDAR 기준2개이며 장착 방향은 별도 결정합니다.')
s=s.replace("['SG2-2: Gemini2 또는 S2-2: OAK Lite','Gemini2+ToF2 센서524,700원, 차체 포함874,700~924,700원. Lite+ToF2는805,301~855,301원.','Gemini2는 IR 탑재. Lite는 IR 없음·노면 시험 조건. Gemini335+ToF2는926,400~976,400원으로 비교 범위 확대 필요.']", "['S2-4: OAK Lite 또는 SG2-4: Gemini2','Lite+ToF4 센서525,701원, 차체 포함875,701~925,701원. Gemini2+ToF4는945,100~995,100원.','Lite는IR 없음·노면 시험 조건. IR 탑재 Gemini335+ToF4는996,800~1,046,800원으로 예산 협의 필요.']")
s=s.replace("['SG335L-2/4 또는 S5-2/4; LiDAR S3','Gemini335L+ToF2/4는 차체 포함1,025,400~1,075,400원 /1,095,800~1,145,800원. D436+ToF4는1,087,800~1,137,800원.','335L과D436이 상위 우선 후보. D436 VAT 확인. S3+OV2710+ToF4는1,293,360~1,343,360원이며 RGB60fps 미확인.']", "['SG335L-4 / S5-4 또는 L3v-2','Gemini335L+ToF4는 차체 포함1,095,800~1,145,800원. D436+ToF4는1,087,800~1,137,800원.','D436 VAT 확인. S3+OV2710+ToF2는1,222,960~1,272,960원이며 RGB60fps 미확인.']")
s=s.replace("put(r,'B21','17개 조합 · LiDAR ToF2 / 스테레오 단독 ToF4, 총34개 안');put(r,'C21','구성 비교에서 같은 조합을 묶고 가격순 비교에서 센서 합계 오름차순으로 봅니다. 두 개 추가 비용은70,400원입니다.');", "put(r,'B21','17개 안: LiDAR2 / 스테레오4');put(r,'C21','LiDAR 포함 구성은ToF2, 스테레오 단독 구성은ToF4로 계산합니다. 혼합 구성도LiDAR 기준2개입니다. 가격순 표도 함께 갱신됩니다.');")
start=s.index("put(s,'C7',4);")
end=s.index("if(before!==snap(h))",start)
s=s[:start]+"const testRow=6;put(s,'C'+testRow,4);if(s.getRange('D'+testRow).values[0][0]!==303160)throw Error('수량 연동 오류');checkSort();put(s,'C'+testRow,2);checkSort();\n"+s[end:]
s=s.replace("[s,'A1:G12','summary'],[s,'A13:G24','summary_more'],[s,'A25:G38','summary_upper'],[s,'A39:G46','summary_notes'],[q,'A1:G16','price_order']", "[s,'A1:G12','summary'],[s,'A13:G21','summary_more'],[s,'A22:G29','summary_notes'],[q,'A1:G21','price_order']")
s=s.replace('34개 합계·정렬','17개 합계·정렬')
# Earlier budget condition rows also referenced the superseded paired options.
anchor="r.getRange('B9:D10').values=["
s=s.replace(anchor,"r.getRange('B6:D8').values=[['B-4: 보유 D435i','ToF4 센서140,800원. 차체와 구동부 비용을 더해야 합니다.','보유 구동부 허용 및 평가 기준 확인.'],['B-4: 보유 D435i','차체35~40만 포함490,800~540,800원.','보유 D435i 사용 허용과 금액 산정 제외를 구분.'],['L1v-2: C1+OV2710','ToF2 센서232,760원. 차체 포함582,760~632,760원.','OV2710 720p60fps 미확인. 차체 실제 비용에 따라60만원을 초과할 수 있음.']];\n"+anchor)
f.write_text(s,encoding='utf-8')
f=p/'finish_ir_update.py';s=f.read_text(encoding='utf-8').replace('range(5,39)','range(5,22)').replace('34 sorted rows','17 sorted rows');f.write_text(s,encoding='utf-8')
