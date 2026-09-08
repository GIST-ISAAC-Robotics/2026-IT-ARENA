import fs from 'node:fs/promises';
import {fileURLToPath} from 'node:url';
import {FileBlob,SpreadsheetFile} from '@oai/artifact-tool';
const path=fileURLToPath(new URL('IT_ARENA_센서_예산안_2026-09-05.xlsx',import.meta.url));
await fs.mkdir(new URL('archive/',import.meta.url),{recursive:true});
const backup=new URL('archive/initial_with_allowances.xlsx',import.meta.url);
try { await fs.copyFile(path,backup,1); } catch(e) { if(e.code!=='EEXIST')throw e; }
const wb=await SpreadsheetFile.importXlsx(await FileBlob.load(path));
const s=wb.worksheets.getItem('구성 비교'),p=wb.worksheets.getItem('단가와 출처'),r=wb.worksheets.getItem('예산 조건');
const put=(sheet,cell,v)=>sheet.getRange(cell).values=[[v]];
put(s,'A2','단위 원 · 센서 단품만 합산 · MCU·배선·장착비·배송·예비비 제외 · B0385 품절 조건부 비교');
put(s,'E4','부자재 제외');put(s,'G4','여유 미산정');put(s,'H4','센서 합계');
s.getRange('E5:E17').values=[[0]];s.getRange('G5:G17').values=[[0]];
s.getRange('E5:E17').format.fill='#F2F2F2';s.getRange('G5:G17').format.fill='#F2F2F2';
for(let i=5;i<=17;i++)s.getRange('H'+i).formulas=[['=D'+i]];
for(const [row,name] of [[8,'C1 + B0385* + ToF 1'],[9,'C1 + B0385* + ToF 2'],[16,'S3 20 Hz + B0385* + ToF 2']])put(s,'B'+row,name);
p.getRange('A12:E12').values=[['Arducam B0385 / OV9782',96360,'VAT 포함 / 품절','https://www.devicemart.co.kr/goods/view?no=14900621','조건부 비교 단가. 컬러 글로벌 셔터·UVC. 문서상 MJPEG 1280×800 100fps, YUY2 동일 해상도 10fps. 판매명120fps와 구분.']];
put(p,'C14','MCU·부자재·배송·예비비 제외');
p.getRange('A18:E20').values=[
 ['Arducam B020101 / IMX291',104500,'VAT 포함 / 대안','https://www.devicemart.co.kr/goods/view?no=14900640','1080p 30fps, UVC, 수동 초점·광각. 해외 준비 2주 표기. B0385와 동등한 고속 대체품 아님.'],
 ['Arducam B0495 / AR0234',null,'국내 원화 미견적','https://blog.arducam.com/downloads/datasheet/B0495_AR0234_Global_Shutter_USB3.0_Camera_Module_Datasheet.pdf','상위 후보만 유지. USB3 UVC·컬러 글로벌 셔터·1920×1200 50fps(YUY2). 기본 초점 2m~무한대 재조정 필요. 합계 미반영.'],
 ['MCU 잠정 후보 ESP32-S3',null,'모델·가격·분담 미정','사용자 2026-09-05 후속 메시지','정확한 개발보드 미선정. 센서 합계 제외는 무료/공식 예산 면제를 뜻하지 않음. 카메라는 Jetson USB 연결을 기준으로 검토.']
];
p.getRange('A18:E20').format=p.getRange('A12:E12').format;
p.getRange('A18:E20').format.rowHeight=88;p.getRange('B18').setNumberFormat('#,##0');
const notes=[
 ['구매비 범위','센서 단가×수량만 합산. 보유 D435i 신규 구매비는 0원이나 규정 평가액은 미정입니다.'],
 ['MCU·부자재','ESP32-S3 잠정 후보. 개발보드 모델·분담 미정으로 합계 제외. 배선·브래킷도 사용자 작업 가정에 따라 제외하며 공식 면제 규정은 아닙니다.'],
 ['추가 여유 제거','종전 연결·장착 3~7.5만, 여유 2~10만원은 부품별 근거 없는 임시 가정이었습니다. 모두 제거하고 만원 단위 올림도 하지 않습니다.'],
 ['세금·배송','VAT 포함 확인 센서는 포함가 사용. D435i/D436의 VAT 여부는 미확정이며, 별도일 때 각각 56,300/59,700원 증가합니다. 배송은 주문별 실비 확인 후 별도 표시합니다.'],
 ['B0385 품절','* 국내 표시가 96,360원은 품절 상태의 비교용 가격입니다. B020101 104,500원으로 바꾸면 RGB 포함 안은 8,140원 증가하나 촬영 성능이 달라집니다.'],
 ['관측·성능 한계','ToF 1~2개는 관측 범위를 줄인 안. 4개 배치에도 정후방 사각이 있습니다. 글로벌 셔터/최대fps만으로 고속 인식·제동 성능을 보증하지 않습니다.'],
 ['하드웨어 가정','차체 35/40만은 아직 확정 견적이 아닙니다. ESP32-S3·ESC·엔코더의 담당 예산과 포함 여부를 하드웨어팀과 확인합니다. 예비 ToF는 단가×추가수량으로 별도 산정합니다.'],
 ['사용법','노란 ToF 수량·단가 셀을 바꾸면 합계가 갱신됩니다. E/G의 0은 합계 제외라는 뜻입니다. RGB 교체 때는 단가와 품명·재고 조건을 함께 갱신합니다.']
];
notes.forEach((n,i)=>{put(s,'A'+(20+i),n[0]);put(s,'B'+(20+i),n[1]);s.getRange('A'+(20+i)+':J'+(20+i)).format.rowHeight=52;});
r.getRange('B7:D10').values=[
 ['B: 보유 D435i + ToF4','센서 140,800원 + 하드웨어35~40만 = 490,800~540,800원. 부자재·MCU·배송·여유 제외.','보유 D435i의 사용 허용과 금액 산정 제외를 구분.'],
 ['L0~L1: C1 + B0385 + ToF1~2','센서230,560~265,760원. 차체 포함580,560~665,760원. B0385 품절 가격이므로 즉시 주문안 아님.','B020101 대체는 8,140원 증가. 신규 스테레오+ToF2는455,301원부터.'],
 ['S1 또는 S2: OAK-D Lite FF','ToF2는805,301~855,301원, ToF4는875,701~925,701원. 총90만은 하드웨어 실제금액에 따라 판단.','Gemini335+ToF4는996,800~1,046,800원. 임의 여유비 제거 후 비교값.'],
 ['S5: D436+ToF4 / L3 비교','D436+ToF4는1,087,800~1,137,800원. S3+B0385+ToF2는1,255,960~1,305,960원.','D436 세금 여부·B0385 품절·공통 전자부품 비용 확인. H는1,222,000~1,272,000원.']
];
r.getRange('B21:D21').values=[['보유14.08만 / OAK45.53·52.57만 / D43673.78만 / S3 90.60만','센서 단품 합계입니다. 임의 부자재·여유비를 제거했으며 RealSense 세금은 별도 확인합니다. RGB 기준은 Arducam B0385 품절 조건부입니다.','ESP32-S3 잠정·배선 등 합계 제외. 구매 및 외부 게시를 하지 않은 검토안.']];
const expected=[70400,140800,176000,230560,265760,239800,455301,525701,646800,703800,737800,905960,872000];
for(let i=0;i<13;i++)if(s.getRange('H'+(i+5)).values[0][0]!==expected[i])throw Error('합계 불일치 '+i);
put(p,'B12',104500);if(s.getRange('H9').values[0][0]!==273900)throw Error('RGB 연동 실패');put(p,'B12',96360);
console.log((await wb.inspect({kind:'match',searchTerm:'#REF!|#DIV/0!|#VALUE!|#NAME\\?|#NUM!',options:{useRegex:true,maxResults:30},maxChars:1500})).ndjson);
await (await SpreadsheetFile.exportXlsx(wb)).save(path);
for(const [sheet,range,name] of [[s,'A1:J17','summary'],[s,'A20:J27','summary_notes'],[p,'A12:E20','camera_prices'],[r,'A5:D10','rules'],[r,'A19:D21','rules_notes']]){
const b=await wb.render({sheetName:sheet.name,range,scale:1,format:'png'});await fs.writeFile(new URL(name+'.png',import.meta.url),new Uint8Array(await b.arrayBuffer()));
}
await fs.writeFile(new URL('results.json',import.meta.url),JSON.stringify(s.getRange('A5:J17').values,null,2));
console.log('수정 완료: 13개 합계·RGB 단가 연동 확인');
