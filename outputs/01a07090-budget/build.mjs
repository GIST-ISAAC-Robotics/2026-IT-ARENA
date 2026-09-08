import fs from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {Workbook,SpreadsheetFile} from '@oai/artifact-tool';
const wb=Workbook.create();
const summary=wb.worksheets.add('구성 비교');
const prices=wb.worksheets.add('단가와 출처');
const hardware=wb.worksheets.add('차체 견적 대조');
const rules=wb.worksheets.add('예산 조건');
function base(s,range,widths,title,sub,headers){
 s.showGridLines=false;
 const r=s.getRange(range);r.format.font={name:'Arial',size:11,color:'#243247'};r.format.rowHeight=34;r.format.verticalAlignment='center';r.format.wrapText=true;
 widths.forEach((w,i)=>s.getRangeByIndexes(0,i,100,1).format.columnWidthPx=w);
 s.getRange('A1').values=[[title]];s.getRange('A1').format.font={name:'Arial',size:15,bold:true};s.getRange('A1').format.wrapText=false;
 s.getRange('A2').values=[[sub]];s.getRange('A2').format.wrapText=false;s.getRange('A2').format.font={name:'Arial',size:10,color:'#59677A'};
 s.getRangeByIndexes(3,0,1,headers.length).values=[headers];s.getRangeByIndexes(3,0,1,headers.length).format={fill:'#34445A',font:{name:'Arial',size:11,bold:true,color:'#FFFFFF'},rowHeight:38,wrapText:true};
}
base(prices,'A1:E17',[230,115,175,430,355],'센서 단가와 구매 출처','2026-09-05 실제 공개 상품 페이지 대조 · 최저가 보장 및 판매처 확정 아님',['품목','단가 (원)','가격 구분','구매 링크','확인할 조건']);
const p=[
 ['VL53L7CX Pololu #3418',35200,'VAT 포함','https://www.devicemart.co.kr/goods/view?no=15145342','센서 IC가 아닌 전압조정·레벨변환 포함 캐리어. 해외 준비기간 약 1주 표기.'],
 ['RPLIDAR C1',99000,'VAT 포함','https://www.devicemart.co.kr/goods/view?no=15274068','10 Hz 대표값. USB 변환기·케이블 동봉 여부는 주문 전 확인.'],
 ['OAK-D Lite FF',384901,'VAT 포함','https://www.devicemart.co.kr/goods/view?no=15694056','FF 기준. 검색 캐시 391,270원과 다름. AF도 실제 페이지 384,901원으로 저가 이점 없음.'],
 ['Orbbec Gemini 335',506000,'VAT 포함','https://www.icbanq.com/P016230029','460,000원 + VAT. 검색 캐시 440,000원을 최종 계산에서 제외.'],
 ['RealSense D435i 신규',563000,'판매 표시가','https://reallystore.net/goods/view?no=363','배송 3,000원 별도. VAT 포함 여부는 견적서로 재확인. 보유 장비의 규정 평가액은 미정.'],
 ['RealSense D436 신규',597000,'판매 표시가','https://reallystore.net/goods/view?no=802','배송 3,000원 별도. VAT 포함 여부 재확인. 할인 전 656,700원 표시.'],
 ['RPLIDAR S3',739200,'VAT 포함','https://www.devicemart.co.kr/goods/view?no=15267468','672,000원 + VAT. 10~20 Hz 지원, 대표 10 Hz. 20 Hz 설정·실제 전달률 및 USB 구성품 확인.'],
 ['RGB USB 카메라 FIT1022',77044,'VAT 포함','https://www.devicemart.co.kr/goods/view?no=16005737','신호등·ArUco용 가격 기준 후보. AF·시야·진동·노출 시험 필요. 더 싼 UVC로 변경 가능하나 단가 미확인.'],
];
prices.getRange('A5:E12').values=p;prices.getRange('B5:B12').setNumberFormat('#,##0');prices.getRange('A5:E12').format.rowHeight=72;prices.getRange('B5:B12').format.fill='#FFF1CD';
prices.getRange('A14').values=[['산정 원칙']];prices.getRange('B14:E14').values=[['실제 표시가','배송·부자재는 별도 여유에 포함','실시간 가격을 검색 캐시보다 우선','결제·재고 확보·업체 견적 요청은 수행하지 않음']];prices.getRange('A14:E14').format.rowHeight=65;
prices.getRange('A16:E16').values=[['공통 지원품은 별도',null,null,'https://github.com/MOSW626/istech-it-arena/blob/main/MANUAL.md','Jetson·배터리·ST3215-HS 2개·PDB 2개를 중복 구매비에 넣지 않음.']];prices.getRange('A16:E16').format.rowHeight=65;

base(hardware,'A1:G28',[260,90,85,120,90,145,460],'하드웨어팀 차체 견적 대조','원본 파일은 변경하지 않았습니다. 가격 재검색 없이 첨부 단가×수량을 재계산했습니다.',['품목','단가 (원)','수량','금액 (원)','구분','원본 행','원본 구매 링크']);
const hw=JSON.parse(await fs.readFile(new URL('hardware.json',import.meta.url),'utf8'));
hardware.getRange('A5:G20').values=hw.map(x=>[x.name,x.unit,x.qty,null,x.kind,'Sheet1 행 '+x.row,x.url]);
for(let i=5;i<=20;i++)hardware.getRange('D'+i).formulas=[['=B'+i+'*C'+i]];
hardware.getRange('A22').values=[['필수 부품 합계']];hardware.getRange('D22').formulas=[['=SUM(D5:D18)']];
hardware.getRange('A23').values=[['선택 부품 합계']];hardware.getRange('D23').formulas=[['=SUM(D19:D20)']];
hardware.getRange('A24').values=[['선택 포함 합계']];hardware.getRange('D24').formulas=[['=SUM(D22:D23)']];
hardware.getRange('A26:C28').values=[['모터 가격 가정',100000,'사용자 추정'],['필수 + 모터 가정',null,null],['선택 포함 + 모터 가정',null,null]];
hardware.getRange('B27').formulas=[['=D22+B26']];hardware.getRange('B28').formulas=[['=D24+B26']];
hardware.getRange('B5:D28').setNumberFormat('#,##0');hardware.getRange('C5:C20').setNumberFormat('0');hardware.getRange('B26').format.fill='#FFF1CD';hardware.getRange('A5:G20').format.rowHeight=43;
hardware.getRange('E26:G28').values=[['未確定ではなく未견적 항목',null,'모터·ESC·엔코더·제어보드·본체 제작재료·통신·전원 부속·비상정지 구현비의 누락/지원 여부 확인'],['비교용 하드웨어 가정',350000,'낮은 비교값. 실제 확정 견적 아님.'],['비교용 하드웨어 가정',400000,'높은 비교값. 비용 상한/완성차 보장 아님.']];
hardware.getRange('E26').values=[['미견적 항목']];hardware.getRange('F27:F28').format.fill='#FFF1CD';hardware.getRange('F27:F28').setNumberFormat('#,##0');hardware.getRange('A26:G28').format.rowHeight=62;

base(summary,'A1:J28',[62,260,65,115,110,115,105,120,115,115],'센서 구성별 예산안','단위 원 · 부자재와 예산 여유 포함 · 팀 합의·구매 승인 전 비교안',['안','주 센서 구성','ToF 수','센서 단품 합','연결·장착 가정','소계','추가 여유','센서 요청액','차체 35만 합산','차체 40만 합산']);
// ToF counts include only active mounted units except the explicitly labelled spare row.
const cases=[
 ['A','보유 D435i + ToF 2',2,'0',30000,20000],
 ['B','보유 D435i + ToF 4',4,'0',45000,25000],
 ['B+','보유 D435i + ToF 5',5,'0',50000,25000],
 ['L0','C1 + 신규 RGB + ToF 1',1,"'단가와 출처'!B6+'단가와 출처'!B12",30000,30000],
 ['L1','C1 + 신규 RGB + ToF 2',2,"'단가와 출처'!B6+'단가와 출처'!B12",30000,30000],
 ['L2','C1 + 보유 D435i + ToF 4',4,"'단가와 출처'!B6",60000,30000],
 ['S1','OAK-D Lite FF + ToF 2',2,"'단가와 출처'!B7",30000,40000],
 ['S2','OAK-D Lite FF + ToF 4',4,"'단가와 출처'!B7",45000,40000],
 ['S3','Gemini 335 + ToF 4',4,"'단가와 출처'!B8",45000,60000],
 ['S4','신규 D435i + ToF 4',4,"'단가와 출처'!B9",45000,90000],
 ['S5','D436 + ToF 4',4,"'단가와 출처'!B10",45000,90000],
 ['L3','S3 20 Hz + RGB + ToF 2',2,"'단가와 출처'!B11+'단가와 출처'!B12",50000,70000],
 ['H','D436 + C1 + ToF 5',5,"'단가와 출처'!B10+'단가와 출처'!B6",75000,100000],
];
summary.getRange('A5:J17').values=cases.map(c=>[c[0],c[1],c[2],null,c[4],null,c[5],null,null,null]);
cases.forEach((c,i)=>{const r=i+5;summary.getRange('D'+r).formulas=[['='+c[3]+"+C"+r+"*'단가와 출처'!$B$5"]];summary.getRange('F'+r).formulas=[['=D'+r+'+E'+r]];summary.getRange('H'+r+':J'+r).formulas=[['=CEILING(F'+r+'+G'+r+',10000)',"=H"+r+"+'차체 견적 대조'!$F$27","=H"+r+"+'차체 견적 대조'!$F$28"]];});
summary.getRange('D5:J17').setNumberFormat('#,##0');summary.getRange('C5:C17').format.fill='#FFF1CD';summary.getRange('E5:E17').format.fill='#FFF1CD';summary.getRange('G5:G17').format.fill='#FFF1CD';summary.getRange('H5:H17').format.fill='#E6EDF5';summary.getRange('A5:J17').format.rowHeight=45;
const notes=[
 ['구매비 범위','보유 D435i의 신규 구매비만 0원. 규정 평가액은 미정이며 위 합계로 상한 통과를 확정하지 않습니다.'],
 ['부자재 가정','연결·장착은 MCU/전원·USB/I²C 배선·브래킷의 설계 예산. 정확한 보드·모델은 미선정입니다.'],
 ['추가 여유','배송·소액 부품·가격 변동·RealSense 세금 조건 재확인에 대비한 예산. 업체 견적 금액이 아닙니다.'],
 ['중복 방지','하드웨어 35/40만은 모터·ESC·엔코더·본체·공통 제어/비상정지 비용을 담아 재협의할 가정입니다. 공통 MCU 등을 양쪽에서 잡았다면 합칠 때 한 번만 셉니다.'],
 ['낮은 안의 한계','ToF 1~2개안은 주변 관측을 크게 포기하는 개발 축소안. 현재 4개 배치도 정후방을 보지 않으며 다중 차량 회피는 미검증입니다.'],
 ['상위안의 목적','D436은 RGB 글로벌 셔터 개선, S3는 360도 기하 관측 개선을 비교합니다. 센서 가격·Hz·FPS만으로 고속 우열을 확정하지 않습니다.'],
 ['예비품','기본표에 예비 센서는 포함하지 않았습니다. 필요 시 ToF 한 개 35,200원을 별도로 더하고 배송·예산 여유와 구분합니다.'],
 ['사용법','노란 입력 셀과 단가 시트 금액을 바꾸면 센서 요청액 및 차체 가정 합산이 갱신됩니다. 소계는 표시가+가정, 요청액은 여유 추가 후 만 원 단위 올림입니다.'],
];
notes.forEach((n,i)=>{const r=20+i;summary.getRange('A'+r).values=[[n[0]]];summary.getRange('B'+r+':J'+r).merge();summary.getRange('B'+r).values=[[n[1]]];summary.getRange('A'+r+':J'+r).format.rowHeight=42;});

base(rules,'A1:D21',[215,280,470,375],'예산 조건별 협의안','2026-09-05 사용자 메시지·첨부 Discord 캡처와 현재 공식 문서를 구분했습니다.',['조건','대응 후보','판단','협의할 사항']);
rules.getRange('A5:D10').values=[
 ['총 30만 / 보유 불가','완성차 견적 성립 보류','필수 차체 221,150원 + 모터 10만 가정 = 321,150원. 센서 이전에 초과. 현재 설계 기준 증액 또는 부품 재설계 필요.','제작 방식 전체가 불가능하다는 주장 대신 현 부품안의 부족액을 제시.'],
 ['지원 30만 / 보유 허용','A 또는 B를 조건부 센서 후보로','D435i만 보유해도 모터·ESC 비용은 남습니다. 차체 20만+ToF10만은 첨부 합계와 맞지 않습니다.','보유 구동부도 가능한지, ToF 신품 예산을 얼마 남길지 확인.'],
 ['총 60만 / 보유 허용','B: 보유 D435i + ToF4','표의 여유 포함 센서 22만 + 하드웨어35~40만 = 57~62만. 현재 개인 제안에 가장 가까움.','보유 D435i를 금액 산정에서 제외하는 조건인지 별도 확인.'],
 ['총 60만 / 보유 불가','L0~L1: C1 + RGB + ToF1~2','요청액 기준 63~71만 정도이므로 60만에 여유 있게 맞는다고 보장할 수 없습니다. 저가 RGB·차체 절감을 별도로 검토.','신규 스테레오를 고수하면 센서만 49만 수준부터. 차체20만 배분은 재협의.'],
 ['총 90만 / 보유 불가','S1 우선 / S2는 증액 비교','OAK-D Lite FF+ToF2는 소계 약49만. 요청액 53만 + 차체35~40만 = 88~93만. ToF4는 요청액 기준97~102만.','ToF4까지 확보하려면 총100만 안팎을 협의. Gemini335는 약111~116만.'],
 ['지원 30만 + 자체 최대100만','S5: D436+ToF4 / L3 비교','총130만 가정에서는 D436+ToF4 요청액88만+차체35~40만 =123~128만. S3안은136~141만으로 초과 가능.','남은 여유는 엔코더·배선·예비품·검증에 우선. C1까지 추가하는 H는140~145만.'],
];rules.getRange('A5:D10').format.rowHeight=96;
rules.getRange('A12:D17').values=[
 ['지원 30만원','이번 계산의 전제','공식 매뉴얼은 25~30만원, 첨부 9/5 메시지는 30만원 주문목록 요청. 새 전체 상한이 확정됐다는 근거는 아님.','https://github.com/MOSW626/istech-it-arena/blob/main/MANUAL.md'],
 ['추가 예산 상태','공식 댓글 재조회','9/2 운영진 댓글 외 새로운 합의 댓글을 확인하지 못함. 보유·대여·협찬 평가, 자체 예산 및 상한 범위는 협의 대상.','https://github.com/MOSW626/istech-it-arena/issues/14#issuecomment-5506542288'],
 ['공정성 선택지','보유 금지 / 기준가 산정','보유품 금지만이 편법 방지의 유일한 수단은 아님. 구매 시점과 무관하게 탑재품의 동일 기준가를 산정하는 안도 비교 가능.','새 제안이며 공식 확정 규정 아님. 실지출·기준가·규정 산정가를 구분.'],
 ['20 Hz와 30 FPS','시간 간격에 따른 이동만 비교','20 km/h에서 10 Hz 약55.6cm, 20 Hz 약27.8cm, 30 FPS 약18.5cm, 60 FPS 약9.3cm. 센서/처리 지연·제동거리·결측은 별도.','S3 기본10Hz를20Hz로 설정·검증해야 함. 카메라는 선택 깊이 프로필의 실제 출력으로 비교.'],
 ['스테레오의 교환조건','OAK / Gemini / D436','OAK는 수동 스테레오의 무늬 부족·조명 영향과 DepthAI 통합, Gemini는 Orbbec 통합, D436은 실제 RGB-D 정렬·Jetson 지연 검증 필요.','https://docs.luxonis.com/hardware/platform/features/depth'],
 ['LiDAR의 교환조건','C1 / S3','C1은 가격이 낮지만 대표10Hz. S3는20Hz·점밀도 개선 여지가 있으나 단일 높이와 순차취득 왜곡은 남음. RGB와 하단 ToF를 계속 산정.','https://www.slamtec.com/en/s3/spec'],
];rules.getRange('A12:D17').format.rowHeight=94;
rules.getRange('A19:D21').values=[
 ['1차 주문 준비','완성차 견적과 분리','30만원 지원 목록은 하드웨어팀과 겹치지 않게 공통 구매 후보만 선택. 모터/ESC 확정 전 임의로 전액을 소진하지 않음.','첨부 메시지는 외부 전송 지시가 아니라 상황 자료로 사용.'],
 ['시뮬레이션 한계','이번 견적은 성능 보증 아님','최근 D435i+ToF4는 단독 저속 완주·정지를 확인했으나 장시간 전달률·종료 실패가 남음. C1 비교도 실차·추월·고속 제품시험 아님.','기존 모델과 팀 설계 결정은 변경하지 않음.'],
 ['권고 제출 묶음','보유 22만 / 신규53·62·88만 / S3 101만','센서 요청액을 먼저 제시하고 하드웨어 확정분을 나중에 더합니다. 예산 조건이 정해지면 필요한 관측 기능을 기준으로 선택합니다.','구매·발송·GitHub 게시 없이 로컬 검토용 산출물로 작성.'],
];rules.getRange('A19:D21').format.rowHeight=85;

console.log((await wb.inspect({kind:'table',range:"'구성 비교'!A5:J17",include:'values',tableMaxRows:13,tableMaxCols:10,maxChars:8000})).ndjson);
if(hardware.getRange('D22').values[0][0]!==221150 || hardware.getRange('D24').values[0][0]!==268550) throw new Error('차체 합계 불일치');
prices.getRange('B5').values=[[40000]];
if(summary.getRange('D6').values[0][0]!==160000) throw new Error('ToF 단가 연동 실패');
prices.getRange('B5').values=[[35200]];
summary.getRange('C6').values=[[5]];
if(summary.getRange('D6').values[0][0]!==176000) throw new Error('ToF 수량 연동 실패');
summary.getRange('C6').values=[[4]];
console.log((await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#N/A|#NUM!|#NULL!|#SPILL!|#CALC!',options:{useRegex:true,maxResults:50},summary:'formula errors',maxChars:1500})).ndjson);
for(const [s,r,n] of [[summary,'A20:J27','summary_notes']]){
 const b=await wb.render({sheetName:s.name,range:r,scale:1,format:'png'});await fs.writeFile(new URL(n+'.png',import.meta.url),new Uint8Array(await b.arrayBuffer()));
}
await (await SpreadsheetFile.exportXlsx(wb)).save(fileURLToPath(new URL('IT_ARENA_센서_예산안_2026-09-05.xlsx',import.meta.url)));
await fs.writeFile(new URL('results.json',import.meta.url),JSON.stringify(summary.getRange('A5:J17').values,null,2));
