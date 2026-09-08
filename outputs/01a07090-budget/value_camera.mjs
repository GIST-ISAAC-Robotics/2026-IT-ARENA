import fs from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {FileBlob,SpreadsheetFile} from '@oai/artifact-tool';
const path=fileURLToPath(new URL('IT_ARENA_센서_예산안_2026-09-05_ToF수정.xlsx',import.meta.url));
const archive=new URL('archive/before_ir_comparison.xlsx',import.meta.url);
// The source archive already exists from the preceding revision.
const wb=await SpreadsheetFile.importXlsx(await FileBlob.load(fileURLToPath(archive)));
const s=wb.worksheets.getItem('구성 비교'),p=wb.worksheets.getItem('단가와 출처'),r=wb.worksheets.getItem('예산 조건'),h=wb.worksheets.getItem('차체 견적 대조');
const q=wb.worksheets.add('가격순 비교'),g=wb.worksheets.add('스테레오 비교');
const put=(a,c,v)=>a.getRange(c).values=[[v]],snap=a=>JSON.stringify([a.getRange('A1:F28').formulas,a.getRange('A1:F28').values]);
const before=snap(h),priceBefore=JSON.stringify(p.getRange('A1:E22').values);
const dm=id=>'https://www.devicemart.co.kr/goods/view?no='+id;
const newPrices=[
 [24,'Orbbec Gemini 2',454300,'VAT 포함',dm(15013145),'413,000원+VAT. 저가 능동 스테레오 비교안. 실내 우선. 실재고·납기는 주문 전 확인.'],
 [25,'Orbbec Gemini 335L',605000,'VAT 포함','https://www.icbanq.com/P016230042','550,000원+VAT. RGB 글로벌 셔터·95mm 기선. 335와 별도 모델. 실재고·납기는 주문 전 확인.'],
 [26,'OAK-D S2 FF',649000,'VAT 포함',dm(15073622),'590,000원+VAT. 일반 S2에는 IR 프로젝터 없음. Lite 대비 800p 스테레오 비교용.'],
 [27,'OAK-D Pro FF / IMX378',852500,'VAT 포함',dm(15073612),'775,000원+VAT. IR 점무늬·투광기 탑재. RGB는 롤링 셔터. OV9782 모델과 구분. IR 사용 시 전원 조건 확인.'],
 [28,'RealSense D455',710000,'표시가 / VAT 미확인','https://reallystore.net/goods/view?no=364','710,000원 표시. 세금 포함 여부·해외구매 조건 확인. 표시가만 합계에 반영하며 확정 도착가가 아님.'],
 [29,'OAK-D Pro FF-97 / OV9782',946000,'VAT 포함',dm(15539204),'860,000원+VAT. IR 탑재·RGB 글로벌 셔터. IMX378 Pro보다 93,500원 높음. FF-97 상품명·센서 확인.']
];
p.getRange('A24:E29').values=newPrices.map(x=>x.slice(1));
p.getRange('A24:E29').format={font:{name:'Arial',size:11,color:'#243247'},rowHeight:92,wrapText:true,verticalAlignment:'center'};
p.getRange('B24:B29').format.fill='#FFF1CD';p.getRange('B24:B29').setNumberFormat('#,##0');
const groups=[
 ['B','보유 D435i',[],0,'재사용 허용 시 지출 최소. IR 탑재. 보유품의 규정 평가액은 별도이며 RGB는 롤링 셔터입니다.'],
 ['RV','OV2710 RGB 단독†',[22],63360,'국내 저가 RGB 주행안. RGB로 노면·경로·마커를 인식하고 측면 ToF로 근접 간격을 감시합니다. 720p60 출력은 미확인. 전방 앞차 가림·깊이 없는 거리 판단을 검증해야 합니다.'],
 ['RA','B0385 RGB 단독*',[12],96360,'글로벌 셔터 RGB 주행안. 사용자120fps 경험이 있으나 당시 프로필은 미확인, 국내 품절 비교가입니다. 측면 ToF4 밖의 전방·후방 관측 공백은 별도 검토합니다.'],
 ['L1v','C1 + OV2710†',[6,22],162360,'저가 RGB 주행 후보로도 검토. RGB가 경로를 찾고 낮은C1이 근접을 감시할 수 있습니다. RGB720p60은 미확인. 경로 가림과10Hz 반응·차체 가림 검증이 필요합니다.'],
 ['L1','C1 + B0385*',[6,12],195360,'글로벌 셔터 RGB 주행 후보로도 검토. RGB 경로 인식·낮은C1 근접 감시의 역할 분리가 가능하나 B0385는 국내 품절 비교가입니다. 10Hz 반응·가림 검증 필요.'],
 ['L2','C1 + 보유 D435i',[6],99000,'재사용 허용 시 저비용 혼합안. 전방 RGB-D로 조향하고 낮은 C1으로 근접 차량을 감시합니다. C1은 조향용이 아니며 10Hz 충돌 회피 성능은 미검증입니다.'],
 ['S2','OAK-D Lite FF',[7],384901,'저가 조건부 안. IR 프로젝터 없음, 깊이용 입력 480p, RGB 최대35fps. 무늬 적은 노면의 깊이 시험이 선행되어야 합니다.'],
 ['SG2','Gemini 2',[24],454300,'저가 IR 탑재안. 실내 우선 모델입니다. Gemini335보다 51,700원 저렴하므로 절감액과 처리 프로필을 함께 비교합니다.'],
 ['S3','Gemini 335',[8],506000,'신규 가성비 우선 후보. IR·능동/수동 깊이와 848×480@60fps 프로필이 장점입니다. RGB는 롤링 셔터입니다.'],
 ['SG335L','Gemini 335L',[25],605000,'RGB 1280×800@60fps 글로벌 셔터가 장점. 깊이848×480의 최소거리는34cm로 D436의19.5cm보다 깁니다. 17cm는 저해상도 조건입니다.'],
 ['S4','신규 D435i',[9],563000,'보유품과 같은 개발 환경이 장점입니다. 표시가가 D436과34,000원 차이여서 신규 구매 우선순위는 낮습니다. VAT 확인.'],
 ['S5','D436',[10],597000,'근접 깊이·기존 RealSense 작업 활용 우선 후보. RGB 글로벌 셔터, 상세 표의60fps는848×480입니다. 상품 최대1280×800의fps 조합은 미확인. VAT 확인.'],
 ['SD455','D455',[28],710000,'원거리 비교안. RGB 글로벌 셔터1280×800@30fps, 저해상도640×360@90fps. 깊이848×480 최소35cm로 근접 관측 조건 확인. VAT 확인.'],
 ['SO2','OAK-D S2 FF',[26],649000,'800p 스테레오 입력의 OAK 비교안. IR 프로젝터가 없어 Lite의 무늬 부족 문제를 해결하는 업그레이드는 아닙니다.'],
 ['SOP','OAK-D Pro FF / IMX378',[27],852500,'IR 점무늬·투광기와 장치 내 처리가 필요한 상위 OAK안. RGB는 롤링 셔터입니다. 전원·가격을 고려하면 회의 본문 우선순위는 낮습니다.'],
 ['SOPG','OAK-D Pro FF-97 / OV9782',[29],946000,'IR과 RGB 글로벌 셔터를 모두 갖춘 상위 OAK안. 고속 RGB 센서 입력이 깊이·인식 처리율을 뜻하지 않으며 추가 비용이 큽니다.'],
 ['L3v','RPLIDAR S3 20Hz + OV2710†',[11,22],802560,'빠른 회전 LiDAR 비교안. 20Hz는 S3 지원 범위의 상단이며 설정·실효 스캔을 확인해야 합니다. RGB60fps 조건도 미확인입니다.'],
 ['L3','RPLIDAR S3 20Hz + B0385*',[11,12],835560,'빠른 회전 LiDAR와 글로벌 셔터 RGB 비교안. B0385는 품절이며 실제 스캔·영상 지연 검증이 필요합니다.'],
 ['H','D436 + C1',[10,6],696000,'신규 혼합 우선 검토안. 전방 RGB-D 조향, 낮은 C1 근접 감시. ToF를 생략해 D436+ToF4보다41,800원 저렴합니다. 10Hz·차체/바퀴 가림 시험 및 VAT 확인.'],
 ['Hfast','D436 + RPLIDAR S3 20Hz',[10,11],1336200,'예산 제한이 없을 때 우선 시험할 혼합안. 전방 RGB-D 조향, 낮은 S3 근접 감시. C1보다640,200원 높으며 20Hz에서 각 간격0.225°. 가림·최종 지연·VAT 확인.']
];
const cases=groups.map(([code,name,refs,base,comment])=>{const count=refs.includes(6)||refs.includes(11)?0:4;return {code:code+'-'+count,name:name+(count?' + ToF '+count:' / ToF 없음'),count,refs,expected:base+35200*count,comment:comment+' '+(count===0?'저상 수평층 개방 배치 검토.':'ToF4는 좌우 측면 보완용. 무사각 보증은 아님.')};});
const end=4+cases.length,headers=['안','센서 구성','ToF 수량','센서 합계','차체 35만 합산','차체 40만 합산','코멘트'];
s.getRange('A1:F1').unmerge();s.getRange('A2:F2').unmerge();
for(let row=27;row<=34;row++)s.getRange('B'+row+':F'+row).unmerge();
s.getRange('A1:J60').clear({applyTo:'all'});
const font={name:'Arial',size:11,color:'#243247'};
function title(a,titleText,subtitle,last='G'){
 a.showGridLines=false;a.getRange('A1:'+last+'1').merge();put(a,'A1',titleText);a.getRange('A1').format={font:{...font,size:16,bold:true},rowHeight:43};
 a.getRange('A2:'+last+'2').merge();put(a,'A2',subtitle);a.getRange('A2').format={font:{...font,size:10,color:'#59677A'},rowHeight:37,wrapText:true};
 a.getRange('A3:'+last+'3').format.rowHeight=16;
}
for(const a of [s,q]){
 a.getRange('A1:G48').format={font,rowHeight:82,wrapText:true,verticalAlignment:'center'};
 [93,285,72,118,128,128,420].forEach((w,i)=>a.getRangeByIndexes(0,i,60,1).format.columnWidthPx=w);
 a.getRange('A4:G4').values=[headers];a.getRange('A4:G4').format={fill:'#34445A',font:{...font,bold:true,color:'#FFFFFF'},rowHeight:40,wrapText:true};
 a.freezePanes.freezeRows(4);a.getRange('D5:F'+end).setNumberFormat('#,##0');
}
title(s,'IT ARENA 센서 구성별 예산','2026-09-06 갱신 · 9/5 단가 · 단위 원 · 20개 조합 · LiDAR 포함 ToF0 / RGB·스테레오 단독 ToF4 · * B0385 품절 · † OV2710의 60fps 프로필 미확인');
title(q,'센서 합계가 낮은 순서','구성 비교의 20개 안을 자동 정렬합니다. 표시가 기준이며 RealSense 세금 미확정·품절 조건은 코멘트를 함께 확인합니다.');
cases.forEach((c,i)=>{
 const row=i+5;s.getRange('A'+row+':G'+row).values=[[c.code,c.name,c.count,null,null,null,c.comment]];
 s.getRange('A'+row+':G'+row).format.fill=i%2===0?'#F3F6FA':'#FFFFFF';
 s.getRange('D'+row).formulas=[['='+[...c.refs.map(n=>"'단가와 출처'!$B$"+n),'C'+row+"*'단가와 출처'!$B$5"].join('+')]];
 s.getRange('E'+row+':F'+row).formulas=[['=D'+row+"+'차체 견적 대조'!$F$27",'=D'+row+"+'차체 견적 대조'!$F$28"]];
 q.getRange('A'+row+':G'+row).format.fill=i%2===0?'#F3F6FA':'#FFFFFF';
});
s.getRange('C5:C'+end).format.fill='#FFF1CD';s.getRange('D5:D'+end).format.fill='#E6EDF5';q.getRange('D5:D'+end).format.fill='#E6EDF5';
q.getRange('A5').formulas=[["=SORT('구성 비교'!A5:G"+end+',4,1)']];
const notes=[
 ['추천 기준','RGB 주행+근접 센서도 기본안 후보입니다. 깊이가 필요하면 가성비Gemini335, 근접 깊이·기존 개발 활용D436, RGB800p60은335L을 비교합니다. D436+C1은 확장안이며 깊이카메라를 필수 전제로 두지 않습니다.'],
 ['IR의 역할','IR 점무늬는 무늬 적은 표면의深이 대응점 찾기를 돕습니다. Lite·일반 S2에는 없으며 추가 ToF가 그 기능을 대신하지 않습니다. 현장 효과는 표면·조명·거리별 시험이 필요합니다.'.replace('深이','깊이')],
 ['촬영 조건','스테레오 비교에서 RGB 셔터·깊이 프로필을 구분합니다. 최대 센서 입력 fps는 깊이·RGB·인식 알고리즘의 동시 처리율을 보증하지 않습니다.'],
 ['세금·배송','D435i/D436/D455는 VAT 포함 미확인 표시가입니다. 별도 부가세라면 단가의10%가 증가합니다. 배송비는 합계 제외. 모든 후보의 주문 시 재고·납기를 확인합니다.'],
 ['구매비 범위','센서 단가×수량만 계산합니다. 보유 D435i의 신규 지출은0원이며 규정 평가액은 미정입니다. ESP32-S3는 보드·담당 예산 미정으로 제외합니다.'],
 ['ToF 배치','LiDAR가 있으면 깊이카메라 유무와 관계없이ToF0개입니다. LiDAR 없는 RGB 또는 스테레오안은 측면ToF4개입니다. RGB 또는RGB-D로 조향하고 낮은LiDAR로 근접 감시하는 검토안이며, 10Hz 충분 여부는 상대속도·여유거리·지연에 달립니다.'],
 ['하드웨어','차체35/40만원은 확정 견적이 아닙니다. 모터·ESC·엔코더·공통 제어부의 실제 비용과 담당을 하드웨어팀과 확인합니다. 배선·장착비·예비비는 추가하지 않았습니다.'],
 ['사용법','노란 단가·ToF 수량을 수정하면 합계와 가격순 표가 갱신됩니다. 가격순 표는 직접 수정하지 않습니다. 상품 교체 시 품명·조건·코멘트도 같이 수정합니다.']
];
notes.forEach(([label,body],i)=>{const row=end+1+i;s.getRange('B'+row+':G'+row).merge();put(s,'A'+row,label);put(s,'B'+row,body);s.getRange('A'+row+':G'+row).format.rowHeight=55;});
q.getRange('A'+(end+2)+':G'+(end+2)).merge();put(q,'A'+(end+2),'정렬 기준: 센서 합계 오름차순. 세금·배송·보유품 평가 조건은 구성 비교 하단과 단가와 출처를 확인합니다. Excel 365/2021 이상 SORT 함수 사용.');q.getRange('A'+(end+2)).format.rowHeight=45;

g.getRange('A1:I42').format={font,rowHeight:170,wrapText:true,verticalAlignment:'center'};
[155,105,237,205,226,218,218,330,340].forEach((w,i)=>g.getRangeByIndexes(0,i,55,1).format.columnWidthPx=w);
title(g,'스테레오 카메라 거리와 출력 모드','2026-09-06 갱신 · 해상도는 가로×세로 픽셀 · @ 뒤는 해당 해상도의 fps · RGB/깊이 개별 모드이며 동시 출력 보증과 구분합니다.','H');
g.getRange('A4:I4').values=[['모델','카메라 단가\n원','거리 범위와 최소 거리\nm · 렌즈 기준','깊이 최대 해상도\n그 해상도의 fps','깊이 최대 fps\n그 fps의 해상도','RGB 최대 해상도\n그 해상도의 fps','RGB 최대 fps\n그 fps의 해상도','IR·셔터와 선정 코멘트','제조사 사양 출처']];
g.getRange('A4:I4').format={fill:'#34445A',font:{...font,bold:true,color:'#FFFFFF'},rowHeight:52,wrapText:true};
g.freezePanes.freezeRows(4);
const guide=JSON.parse(await fs.readFile(new URL('stereo_profiles.json',import.meta.url),'utf8'));
guide.forEach((c,i)=>{
 const row=i+5;
 g.getRange('A'+row+':I'+row).values=[[c.name,null,c.range,c.dres,c.dfps,c.rres,c.rfps,c.comment,c.source]];
 g.getRange('B'+row).formulas=[["='단가와 출처'!B"+c.ref]];
 g.getRange('A'+row+':I'+row).format.fill=i%2===0?'#F3F6FA':'#FFFFFF';
 if(c.ref===9||c.ref===28)g.getRange('A'+row+':I'+row).format.rowHeight=210;
});
g.getRange('B5:B14').setNumberFormat('#,##0');
g.getRange('C5:C14').format.fill='#E6EDF5';
g.getRange('D5:G14').format.font={...font,size:11};
g.getRange('I5:I14').format.font={...font,size:9,color:'#59677A'};
const guideNotes=[
 ['거리 해석','권장은 제조사의 적정 사용 거리이며 확정 측정 한계가 아닙니다. Min-Z는 렌즈에서의 전방 깊이 축 거리입니다. 차체 끝 기준 여유거리나 바닥에서 처음 보이는 거리로 그대로 쓰면 안 됩니다. 반사율·노출·해상도·깊이 탐색 범위와 장착 높이/각도에 따라 유효 영역이 달라집니다.'],
 ['모드 비교','각 최대값과 동시에 지원하는 해상도/fps를 짝지었습니다. 같은 최대fps를 지원하는 해상도가 여러 개면 확인한 가장 큰 해상도를 적었습니다. 더 낮은 해상도 전체 목록은 생략합니다. USB3, 펌웨어와 SDK 프로필·깊이/RGB 동시 출력은 주문 전 확인합니다.'],
 ['Gemini 최소거리','335L의17cm는424×240 조건입니다. 848×480에서는335L34cm, 33518cm이고 D436은19.5cm입니다. 335L의 긴 기선은 원거리 정밀도에 유리할 수 있으나 가까운 전방 관측의 우선순위를 뒤집을 수 있으므로 길이만으로 상위 추천하지 않습니다.'],
 ['D436 RGB 불일치','상품 페이지는 최대1280×800와최대60fps를 별개로 적지만, 2026-03 데이터시트 표4-5의 RGB(YUY2)에는1280×720@30과848×480@60까지만 확인됩니다. Calibration의1280×800@15/25는 일반 RGB로 전용하지 않습니다. 1280×800@60을 확정 구매조건으로 쓰지 않습니다.'],
 ['특수 고속 모드','Gemini335/335L의100fps는848×100으로 세로 시야가14°입니다. D435i300fps·D455100fps도848×100의 특수 모드이며 제조사 지원 사례가 있고 현행FW/SDK 확인이 필요합니다. 전방 노면을 넓게 보는 일반 조향 설정의 최대값과 분리합니다.'],
 ['OAK 깊이 기준','OAK는 센서 입력fps와 StereoDepth 출력fps를 구분합니다. 표의50fps(1280×800)/110fps(640×400)는 현행DepthAI v3 RVC2의 Fast Density No Subpixel 공통 벤치값이며 완제품별 절대상한·동시 RGB/AI 성능 보증이 아닙니다. Lite480p의 정확한 깊이 출력 상한은 제조사 자료에서 미확인입니다.'],
 ['OAK 세부 조건','400p는640×400,800p는1280×800입니다. 가까운 거리용Extended, 좌우 일치 검사, 서브픽셀, 필터는 처리율에 영향을 줍니다. v2의720p150fps 단순 모드 수치를 v3 프리셋과 섞어 순위를 매기지 않습니다. RGB열의129/143fps 등은 RVC2 센서 드라이버 지원 모드로 압축·전송·AI까지의 출력률과 다릅니다.'],
 ['IR과 초점','Lite·일반S2에는 IR 점무늬 프로젝터가 없고 Pro에는 점무늬와 IR 투광기가 있습니다. Lite의IR-cut 때문에 외부IR 추가만으로Pro와 같아진다고 가정하지 않습니다. 고정초점 자체가 차량용 결격 사유는 아닙니다. 제조사 권장 거리와 실제 표면에서 선명도를 확인합니다.'],
 ['Gemini 구분','Gemini2와330계열은 다른 제품군입니다. 335→335L은 기선50→95mm뿐 아니라 RGB 롤링→글로벌 셔터도 바뀝니다. 336/336L의 주요 차이는 깊이 IR-pass 광학 필터이며 프로젝터 유무 차이가 아닙니다. Le/Lg는 연결방식·가격 조건까지 다른 파생형입니다.'],
 ['혼합안 역할','LiDAR가 있으면 RGB만 있어도ToF0개입니다. LiDAR가 없으면RGB/스테레오 모두 측면ToF4개입니다. RGB-D가 전방 조향·신호/마커를 담당하고 낮은LiDAR가 근접 차량·벽을 감시하는 검토안입니다. 라이다 높이의 수평층을 개방하되 조향한 바퀴·지지대 가림, 피치/롤·턱, 검은 표면과 상대차의 감지 가능한 높이를 확인해야 합니다.'],
 ['10Hz 판단','C1은8–12Hz(대표10),0.72° 간격,최소0.05m·거리 정확도±30mm입니다. 같은 방향의 재관측 간격 약100ms 동안 접근속도0.5/1/2m/s면5/10/20cm가 줄어듭니다. 여기에 처리·제동·조향 지연이 더해집니다. 나란히 달리는 낮은 상대속도에는 사용 가능성이 있으나10Hz 충돌 회피 충분 판정은 미검증입니다.'],
 ['20Hz 대안','S3는10–20Hz,32k 측정/초로20Hz의 각 간격은0.225°입니다. 최소0.05m·거리 정확도±30mm로C1보다 거리 정확도까지 좋아진다고 가정하지 않습니다. 20Hz는 같은 방향의 재관측을 약50ms로 줄이지만 수평면 밖 물체나 차체 가림을 해결하지 않습니다.'],
 ['LiDAR 출처','https://www.slamtec.com/en/c1/spec\nhttps://www.slamtec.com/en/s3/spec'],
 ['요청 예산','RGB 기본안은 C1+RGB/ToF0으로 총60만원 협의를 검토합니다(저가카메라fps·재고 조건). 깊이 확장까지 확보하려면 총120만원(지원30+자체90). D436+C1+차체35~40만은1,046,000~1,096,000원, D436 VAT별도면1,105,700~1,155,700원입니다.'],
 ['예산 제한 없음','현재 후보 중 D436+S3 20Hz / ToF0을 우선 시험합니다. RGB 경로·깊이 보조·저상LiDAR 근접 감시용이며 깊이는 필수 전제가 아닙니다. 센서1,336,200원, 차체35~40만 및 D436 VAT별도 가정1,745,900~1,795,900원. RGB800p60 필수이면335L/OAK Pro와 재비교합니다.'],
 ['RGB 주행 검토','RGB로 노면·경로를 찾고 별도 근접 센서로 간격을 감시하는 구조도 가능합니다. 후방 마커는 차량 전체 외곽이 아니며 가려질 수 있습니다. 노면과 같은 높이의 잔디 구분은 깊이만으로도 해결되지 않습니다. RGB 경로 가림·조명 변화·급커브에서 깊이 보조의 실제 이득을 비교한 뒤 구매합니다.']
];
guideNotes.forEach(([label,body],i)=>{const row=i+17;put(g,'A'+row,label);g.getRange('B'+row+':I'+row).merge();put(g,'B'+row,body);g.getRange('A'+row+':I'+row).format.rowHeight=55;g.getRange('A'+row).format.font={...font,bold:true};});
r.getRange('B6:D8').values=[['B-4: 보유 D435i','ToF4 센서140,800원. 차체와 구동부 비용을 더해야 합니다.','보유 구동부 허용 및 평가 기준 확인.'],['B-4: 보유 D435i','차체35~40만 포함490,800~540,800원.','보유 D435i 사용 허용과 금액 산정 제외를 구분.'],['L1v-0: C1+OV2710 / ToF0','센서162,360원. 차체 포함512,360~562,360원.','낮은C1이 근접 영역을 감시한다는 전제. OV2710 720p60fps 미확인.']];
r.getRange('B9:D10').values=[
 ['S2-4: OAK Lite 또는 SG2-4: Gemini2','Lite+ToF4 센서525,701원, 차체 포함875,701~925,701원. Gemini2+ToF4는945,100~995,100원.','Lite는IR 없음·노면 시험 조건. IR 탑재 Gemini335+ToF4는996,800~1,046,800원으로 예산 협의 필요.'],
 ['SG335L-4 / S5-4 또는 L3v-0','Gemini335L+ToF4는 차체 포함1,095,800~1,145,800원. D436+ToF4는1,087,800~1,137,800원.','D436 VAT 확인. S3+OV2710/ToF0은1,152,560~1,202,560원이며 RGB60fps 미확인.']
];r.getRange('A9:D10').format.rowHeight=110;
put(r,'B19','보유 D435i / 신규 Gemini335');put(r,'C19','보유품 허용 시 D435i, 신규 가성비는 IR 탑재 Gemini335를 우선 비교합니다. Lite는 표면 시험을 전제로 한 저가안입니다.');
put(r,'B20','깊이 확장 총120만: D436+C1 / ToF0');put(r,'C20','센서696,000원. 차체35~40만 포함1,046,000~1,096,000원. D436 VAT 별도면1,105,700~1,155,700원. 깊이 보조까지 확보할 때 지원30+자체90을 제안합니다.');
put(r,'B21','20개 안: LiDAR0 / RGB·스테레오4');put(r,'C21','혼합은 RGB-D 조향·낮은 LiDAR 근접 감시용입니다. 보유D435i+C1+차체35~40만은449,000~499,000원. 보유 허용 및 평가액 제외 조건입니다.');r.getRange('A19:D23').format.rowHeight=100;
put(r,'A22','예산 제한 없음');put(r,'B22','D436+S3 20Hz / ToF0');put(r,'C22','센서1,336,200원. 차체35~40만 합산1,686,200~1,736,200원. D436 VAT 별도면1,745,900~1,795,900원으로 총180만 수준.');put(r,'D22','현재 비교 후보 중 시험 우선안. 20Hz 충돌 회피·장착 가림과 D436 RGB 출력 조건은 검증 전입니다.');
put(r,'A23','축소 협의');put(r,'B23','총110만: Gemini335+ToF4');put(r,'C23','센서646,800원. 차체35~40만 포함996,800~1,046,800원(VAT 포함). RGB 롤링 셔터와4개ToF 배치의 관측 범위를 수용하는 안입니다.');put(r,'D23','120/110/180만원은 요청 기준이며 별도 부자재·예비비 항목을 더한 금액이 아닙니다. 차체는 미확정 가정입니다.');
r.getRange('A22:D23').format={font,rowHeight:100,wrapText:true,verticalAlignment:'center'};
put(r,'A24','RGB 기본안');put(r,'B24','총60만 협의: C1+RGB / ToF0');put(r,'C24','OV2710안 차체 포함512,360~562,360원. B0385안545,360~595,360원. RGB 주행·근접 감시를 분리하는 후보이며 저가카메라60fps/재고와 RGB 경로 유지를 확인합니다.');put(r,'D24','깊이가 반드시 필요하다는 전제는 재검토합니다. RGB+ToF4안도 별도 비교하며 RGB 단독 조향의 실차 성공이나 충돌 방지를 보증하지 않습니다.');r.getRange('A24:D24').format={font,rowHeight:100,wrapText:true,verticalAlignment:'center'};
put(r,'A19','가성비 후보');put(r,'A20','상위 후보');put(r,'A21','비교 범위');put(r,'D19','가격·사용 조건 확인 후 회의 후보로 선별합니다. 팀 구매 결정은 아닙니다.');
for(let i=0;i<cases.length;i++){const row=i+5,vals=s.getRange('D'+row+':F'+row).values[0],e=cases[i].expected;if(JSON.stringify(vals)!==JSON.stringify([e,e+350000,e+400000]))throw Error('합계 불일치 '+cases[i].code);}
const checkSort=()=>{const actual=q.getRange('A5:G'+end).values,expected=s.getRange('A5:G'+end).values.slice().sort((a,b)=>a[3]-b[3]);if(JSON.stringify(actual)!==JSON.stringify(expected))throw Error('가격순 정렬 또는 코멘트 연동 오류');};
checkSort();put(p,'B25',100000);checkSort();put(p,'B25',605000);checkSort();
const testRow=5+cases.findIndex(c=>c.code==='L1v-0');put(s,'C'+testRow,4);if(s.getRange('D'+testRow).values[0][0]!==303160)throw Error('수량 연동 오류');checkSort();put(s,'C'+testRow,0);checkSort();
if(before!==snap(h))throw Error('차체 시트 변경');if(priceBefore!==JSON.stringify(p.getRange('A1:E22').values))throw Error('기존 단가 변경');
console.log((await wb.inspect({kind:'table',range:'구성 비교!A15:G20',include:'values,formulas',tableMaxRows:6,tableMaxCols:7,maxChars:1500})).ndjson);
console.log((await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:30},maxChars:1000})).ndjson);
await (await SpreadsheetFile.exportXlsx(wb)).save(path);
await fs.writeFile(new URL('results.json',import.meta.url),JSON.stringify(s.getRange('A5:G'+end).values,null,2));
await fs.writeFile(new URL('guide_results.json',import.meta.url),JSON.stringify(g.getRange('A5:I14').values,null,2));
for(const [sheet,range,name]of [[s,'A1:G12','summary'],[s,'A13:G24','summary_more'],[s,'A25:G32','summary_notes'],[q,'A1:G24','price_order'],[g,'A1:H9','stereo_guide'],[g,'A10:H14','stereo_more'],[g,'H5:I14','stereo_sources'],[g,'A17:I24','stereo_notes'],[g,'A25:I32','stereo_budget_notes'],[r,'A9:D10','rules'],[r,'A19:D24','rules_notes']]){
 const blob=await wb.render({sheetName:sheet.name,range,scale:1,format:'png'});await fs.writeFile(new URL(name+'.png',import.meta.url),new Uint8Array(await blob.arrayBuffer()));
}console.log('20개 합계·정렬·단가/수량 연동·기존 차체/단가 보존 확인. 6개 시트 저장.');
