from pathlib import Path
import shutil
p=Path('outputs/01a07090-budget/value_camera.mjs')
t=p.read_text(encoding='utf-8')
t=t.replace('before_domestic_camera.xlsx','before_tof_2_4.xlsx')
t=t.replace("for(let row=20;row<=27;row++)s.getRange('B'+row+':J'+row).unmerge();", "s.getRange('A1:F1').unmerge();s.getRange('A2:F2').unmerge();\nfor(let row=20;row<=27;row++)s.getRange('B'+row+':F'+row).unmerge();")
t=t.replace("'A1:J30'","'A1:J40'").replace("'A1:F27'","'A1:F34'")
start=t.index('const cases=[');end=t.index("p.getRange('A22:E24')",start)
t=t[:start]+'''const groups=[
 ['B','보유 D435i',[],0],['L1v','C1 + OV2710†',[6,22],162360],['L1','C1 + B0385*',[6,12],195360],['L2','C1 + 보유 D435i',[6],99000],
 ['S2','OAK-D Lite FF',[7],384901],['S3','Gemini 335',[8],506000],['S4','신규 D435i',[9],563000],['S5','D436',[10],597000],
 ['L3v','S3 20 Hz + OV2710†',[11,22],802560],['L3','S3 20 Hz + B0385*',[11,12],835560],['H','D436 + C1',[10,6],696000]
];
const cases=groups.flatMap(([code,name,refs,base])=>[2,4].map(count=>[code+'-'+count,name+' + ToF '+count,count,refs,base+35200*count]));
''' +t[end:]
t=t.replace("2026-09-05 · 단위 원 · 센서 단품 합계", "2026-09-05 · 단위 원 · ToF 2개/4개 비교")
t=t.replace("i%2===0?'#F3F6FA':'#FFFFFF'","Math.floor(i/2)%2===0?'#F3F6FA':'#FFFFFF'")
t=t.replace('C5:C19','C5:C26').replace('D5:D19','D5:D26').replace('D5:F19','D5:F26')
t=t.replace('ToF 1~2개는 관측 범위를 줄인 안이며 4개 배치에도 정후방 사각이 있습니다. 표시 fps·Hz는 고속 인식·제동 보증이 아닙니다.','2개는 관측을 줄이는 안, 4개는 범위를 넓히는 비교안입니다. 동일 수량도 배치에 따라 범위가 다르며 고속 안전을 보증하지 않습니다.')
t=t.replace('const row=i+20;','const row=i+27;')
start=t.index("put(r,'B8'");end=t.index('for(let i=0;i<cases.length;',start)
t=t[:start]+'''r.getRange('B6:D10').values=[
 ['B-2 / B-4: 보유 D435i','ToF2/4 센서70,400/140,800원. 차체와 구동부 비용을 더해야 합니다.','보유 구동부 사용 여부 및 보유품 평가 기준 확인.'],
 ['B-2 / B-4: 보유 D435i','차체35~40만 포함 ToF2는420,400~470,400원, ToF4는490,800~540,800원.','보유 D435i 사용 허용과 금액 산정 제외를 구분.'],
 ['L1v-2 / L1v-4: C1+OV2710','센서232,760/303,160원. 차체 포함582,760~632,760원 /653,160~703,160원.','OV2710 720p60fps 미확인. 4개안은 차체35~40만 가정에서 총60만 초과.'],
 ['S2-2 / S2-4: OAK-D Lite FF','센서455,301/525,701원. 차체 포함805,301~855,301원 /875,701~925,701원.','ToF2/4의 관측 차이와 하드웨어 실제 금액에 따라 선택.'],
 ['S5-2 / S5-4 또는 L3v-2 / L3v-4','D436은 센서667,400/737,800원. S3+OV2710은872,960/943,360원. 차체35~40만원을 더해 비교.','D436 VAT·OV2710 촬영 조건 확인. S3+OV2710+ToF4는 총1,293,360~1,343,360원.']
];
put(r,'B21','11개 센서 조합 × ToF 2개/4개');put(r,'C21','같은 센서 조합마다 ToF2/4를 나란히 비교합니다. ToF 두 개 추가 비용은70,400원입니다. 장착 방향과 관측 범위는 별도 설계합니다.');
''' +t[end:]
start=t.index("put(p,'B22',64360)");end=t.index('if(before!==snap(h))',start)
t=t[:start]+'''put(p,'B22',64360);if(s.getRange('D7').values[0][0]!==233760||s.getRange('D8').values[0][0]!==304160)throw Error('카메라 단가 연동 오류');put(p,'B22',63360);
put(s,'C7',4);if(s.getRange('D7').values[0][0]!==303160)throw Error('수량 연동 오류');put(s,'C7',2);
for(let i=0;i<cases.length;i+=2)if(s.getRange('D'+(i+6)).values[0][0]-s.getRange('D'+(i+5)).values[0][0]!==70400)throw Error('짝 비교 차액 오류');
''' +t[end:]
t=t.replace("[s,'A1:F19','summary'],[s,'A20:F27','summary_notes'],[p,'A18:E22','camera_prices']","[s,'A1:F16','summary'],[s,'A17:F26','summary_more'],[s,'A27:F34','summary_notes']")
t=t.replace('A5:F19','A5:F26').replace('15개 합계','22개 합계')
p.write_text(t,encoding='utf-8')
print('ToF 2/4 비교 빌더 수정')
