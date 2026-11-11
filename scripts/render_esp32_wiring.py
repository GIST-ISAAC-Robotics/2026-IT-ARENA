"""ESP32 배선 설계의 SVG/HTML/핀표 생성. 실제 하드웨어를 제어하지 않는다."""
from pathlib import Path
from html import escape as esc
import json

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/hardware/schematics'
DATA = json.loads((OUT / 'esp32_gpio_v1.json').read_text(encoding='utf-8'))
BLUE, GREEN, PURPLE = '#2161ae', '#08776d', '#7641a7'
RED, GRAY, AMBER = '#ba3c36', '#576579', '#a86800'
INK = '#14273d'


class Drawing:
    def __init__(self, title, subtitle, height):
        self.height = height
        self.parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="1700" height="{height}" viewBox="0 0 1700 {height}" role="img" aria-label="{esc(title)}">',
                      '<style>text{font-family:"Malgun Gothic","Noto Sans KR",sans-serif;fill:#14273d} .small{font-size:17px}</style>',
                      f'<rect width="1700" height="{height}" fill="#f6f8fc"/>']
        self.text(50, 54, title, 31, bold=True)
        self.text(50, 90, subtitle, 19, color=GRAY)
        self.line(50, 113, 1650, 113, '#cad4e0', 1)

    def text(self, x, y, label, size=20, color=INK, bold=False, anchor='start'):
        self.parts.append(f'<text x="{x}" y="{y}" font-size="{size}" style="fill:{color}" font-weight="{700 if bold else 400}" text-anchor="{anchor}">{esc(label)}</text>')

    def line(self, x1, y1, x2, y2, color, width=3, dash=False):
        self.parts.append(f'<path d="M{x1},{y1} L{x2},{y2}" fill="none" stroke="{color}" stroke-width="{width}"{chr(32) + "stroke-dasharray=\"9 6\"" if dash else ""}/>')

    def box(self, x, y, w, h, title, sub='', color=BLUE, conditional=False):
        dash = ' stroke-dasharray="9 6"' if conditional else ''
        self.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="9" fill="white" stroke="{color}" stroke-width="2"{dash}/>')
        self.text(x+18, y+34, title, 22, color, True)
        if sub:
            self.text(x+18, y+62, sub, 16, GRAY)

    def pin(self, x, y, label, side='left', color=INK, size=20):
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="4" fill="white" stroke="{color}" stroke-width="2"/>')
        self.text(x+(14 if side=='left' else -14), y+7, label, size, color,
                  anchor='start' if side=='left' else 'end')

    def wire(self, x1, x2, y, color, label='', flow=0, dash=False):
        self.line(x1, y, x2, y, color, 3, dash)
        if label:
            self.text((x1+x2)/2, y-10, label, 15, color, anchor='middle')
        if flow:
            cx = (x1+x2)/2
            self.parts.append(f'<path d="M{cx-flow*7},{y-5} L{cx+flow*3},{y} L{cx-flow*7},{y+5}" fill="none" stroke="{color}" stroke-width="2"/>')

    def note(self, x, y, lines, color=GRAY, size=17, step=27):
        for i, text in enumerate(lines):
            self.text(x, y+i*step, text, size, color)

    def save(self, name):
        self.text(50, self.height-26, 'IT ARENA · 2026-11-11 · v1 | 단자 이름 기준 배선도 · 기판 핀의 물리적 순서/위치를 나타내지 않음', 16, GRAY)
        (OUT/name).write_text('\n'.join(self.parts+['</svg>'])+'\n', encoding='utf-8')


def signals():
    d=Drawing('ESP32 GPIO 배선도', '01 / 신호 · 현재 IMU 배선 유지 · 총 11 GPIO + 보드 내부 USB-UART 2핀 예약', 1230)
    d.text(50,150,'실선: 설계 결선 (실측 여부는 각 상자 참고)     주황 점선: 실물 확인 전 연결 보류     화살표: 신호 방향',18,GRAY)
    d.box(680,200,300,835,'ESP32-S3','Flash 16 MB / PSRAM 8 MB')
    d.box(50,200,370,435,'Adafruit 4438 · LSM6DSOX','현재 연결 유지 / 하단 긴 핀열',GREEN)
    imu = [('3V3','VIN','3.3 V',RED,0),('GND','GND','GND',GRAY,0),
           ('GPIO12','SCL','SPI SCLK',GREEN,-1),('GPIO11','SDA','SPI MOSI',GREEN,-1),
           ('GPIO13','DO','SPI MISO',GREEN,1),('GPIO10','CS','SPI CS (LOW 유효)',GREEN,-1),
           ('GPIO9','I1','INT1',GREEN,1)]
    for n,(esp,pin,net,col,flow) in enumerate(imu):
        y=300+n*45
        d.wire(420,680,y,col,net,flow)
        d.pin(420,y,pin,'right',col); d.pin(680,y,esp,'left',col)
    d.text(70,610,'SPI 실측 / I1 IRQ 동작은 미검증',16,GREEN)
    d.box(1260,200,390,330,'Cytron MD20A','설계 배정 / 구동 시험 전',BLUE)
    for y,gpio,pin,net,flow,col in [(300,'GPIO15','PWM','MOTOR_PWM',1,BLUE),(350,'GPIO6','DIR','MOTOR_DIR',1,BLUE),(400,'GND','GND','Signal GND',0,GRAY)]:
        d.wire(980,1260,y,col,net,flow)
        d.pin(980,y,gpio,'right',col);d.pin(1260,y,pin,'left',col)
    d.note(1280,451,['3.3 V PWM/DIR 직접 입력','PWM LOW = brake','VB+/VB− · MA/MB는 시트 02'],size=17)
    d.note(705,635,['호환 PCB 제조사·판본 미확인','GPIO 실크로 찾아 연결','헤더 P1/P2 순번은 미지정'],size=16)
    d.box(1260,640,390,395,'Bus Servo Adapter (A)','설계 배정 / UART 점퍼 A',PURPLE)
    for y,gpio,pin,net,flow,col in [(720,'GPIO17','TX / TXD','UART1 TX → TX',1,PURPLE),(770,'GPIO18','RX / RXD','UART1 RX ← RX',-1,PURPLE),(820,'GND','GND','H2 GND',0,GRAY)]:
        d.wire(980,1260,y,col,net,flow)
        d.pin(980,y,gpio,'right',col);d.pin(1260,y,pin,'left',col)
    d.note(1280,873,['이 제품은 TX↔TX / RX↔RX','일반 UART처럼 교차하지 않음','ESP32에서 전원 공급하지 않음','서보는 D / V+ / GND 3선','12 V 전원·서보 결선: 시트 02'],size=17)
    d.box(50,710,235,325,'PGE-213','XHP-4 / 모터축 엔코더',AMBER,True)
    d.note(70,810,['5 V 공급: 시트 02'],size=15)
    d.box(395,710,175,325,'CF-0108','TXS 확인 전 보류',AMBER,True)
    for y,a,b,left,right,gpio,net in [(850,'3 HA','B1','A1','A상','GPIO4','ENC_A'),(910,'4 HB','B2','A2','B상','GPIO5','ENC_B')]:
        d.wire(285,395,y,AMBER,'5 V',1,True)
        d.pin(285,y,a,'right',AMBER,18); d.pin(395,y,b,'left',AMBER,18)
        d.wire(570,680,y,AMBER,'3.3 V',1,True)
        d.pin(570,y,left,'right',AMBER,18);d.pin(680,y,gpio,'left',AMBER,18)
    d.note(70,972,['1 VCC / 2 GND','커넥터 방향·선 색은 실물 확인'],size=14,step=24)
    d.note(410,970,['전원·OE: 시트 02','A/B는 IC 기준 명칭'],size=13,step=24)
    d.note(705,970,['UART0 GPIO43/44','보드 내부 USB-UART 전용','외부 장치에 중복 배정 금지'],size=16)
    d.note(50,1080,['IMU: 3Vo, I2 및 상단 SCX / SDX / CS / DO / GND는 이번 배선에서 사용하지 않습니다.',
                    'CF: IC·LV/HV·OE 회로와 5 V→3.3 V 파형 확인 전 연결 보류. 엔코더 HA/HB를 ESP32에 직접 연결하지 않습니다.',
                    '현재 펌웨어는 IMU 진단용입니다. 이 도면만으로 모터·서보의 부팅/리셋/통신 단절 시 정지가 구현되지는 않습니다.'],size=18,step=31)
    d.save('esp32_gpio_signals_v1.svg')


def power():
    d=Drawing('ESP32 주변 전원·모터·서보 결선도', '02 / 승인된 전원 구성 유지 · 각 + 전원 레일 분리 · 공통 GND와 전력 귀환선 구분', 1390)
    d.text(50,150,'빨강: 전원 +     회색: GND/전력 귀환     파랑: 모터 출력     보라: 서보 데이터     주황 점선: CF 조건부',18,GRAY)
    d.box(50,190,340,175,'J01 CH1','19 V / 2.3 A',RED)
    d.box(670,190,340,175,'Jetson Orin Nano','DC 5.5×2.5 mm',BLUE)
    for y,a,b,col in [(275,'+','중앙 +',RED),(325,'−','외곽 −',GRAY)]:
        d.wire(390,670,y,col,'19 V' if col==RED else '전원 귀환');d.pin(390,y,a,'right',col);d.pin(670,y,b,'left',col)
    d.box(1310,190,340,175,'ESP32-S3','현재 확인한 USB-UART 포트',BLUE)
    d.wire(1010,1310,290,BLUE,'USB-A ↔ USB-C',1)
    d.note(1060,322,['데이터 + 5 V + GND'],size=16)
    d.note(1330,300,['단일 USB로 전원/통신','다른 5 V 전원 중복 공급 금지'],size=16,step=27)

    d.box(50,405,340,165,'ESP32 3V3 / GND','IMU 전원: 기존 연결',RED)
    d.box(670,405,340,165,'Adafruit 4438','하단 핀열',GREEN)
    for y,a,b,col in [(485,'3V3','VIN',RED),(535,'GND','GND',GRAY)]:
        d.wire(390,670,y,col);d.pin(390,y,a,'right',col);d.pin(670,y,b,'left',col)
    d.note(1060,458,['3Vo는 전원 입력으로 쓰지 않음','CF의 3.3 V도 같은 ESP32 3V3에서 분기','카메라·LiDAR는 Jetson 연결','ESP32 GPIO를 쓰지 않음'],size=18,step=29)

    d.box(50,610,340,175,'M01 CH1','배터리 전압 출력',RED)
    d.box(670,610,340,175,'MD20A','신호 PWM/DIR/GND: 시트 01',BLUE)
    d.box(1310,610,340,175,'MB4266-24180','브러시 DC 모터',BLUE)
    for y,a,b,col in [(695,'+','VB+',RED),(745,'−','VB−',GRAY)]:
        d.wire(390,670,y,col,'전력선');d.pin(390,y,a,'right',col);d.pin(670,y,b,'left',col)
    for y,a,b in [(695,'MA','6 (+)'),(745,'MB','5 (−)')]:
        d.wire(1010,1310,y,BLUE,'모터선');d.pin(1010,y,a,'right',BLUE);d.pin(1310,y,b,'left',BLUE)
    d.text(50,812,'MA→6 / MB→5는 최초 시험 극성입니다. 차량 전진 방향은 실측 후 부호를 확정하며, MA/MB는 GND가 아닙니다.',17,GRAY)

    d.box(50,845,340,180,'M01 CH5 또는 CH6','12 V / 2 A 한 채널만',RED)
    d.box(670,845,340,180,'Bus Servo Adapter (A)','DC 5.5×2.1 또는 전원 단자',PURPLE)
    d.box(1310,845,340,180,'ST3215-HS × 1','3선 버스 서보',PURPLE)
    for y,a,b,col in [(935,'+','전원 + / 중앙 +',RED),(985,'−','전원 − / GND',GRAY)]:
        d.wire(390,670,y,col);d.pin(390,y,a,'right',col);d.pin(670,y,b,'left',col,17)
    for y,a,b,col in [(925,'D','D / DATA',PURPLE),(965,'V+','V+',RED),(1005,'GND','G / GND',GRAY)]:
        d.wire(1010,1310,y,col);d.pin(1010,y,a,'right',col,17);d.pin(1310,y,b,'left',col,17)
    d.text(50,1055,'CH5/CH6 출력 병렬 연결 금지. 어댑터 USB는 이번 구성에서 비워 둡니다. 서보 케이블 D/V/G 위치·극성을 대조합니다.',17,GRAY)

    d.box(50,1090,340,180,'J01 CH4','5 V / 0.5 A · XT30',RED)
    d.box(670,1090,340,180,'PGE-213','XHP-4 커넥터 논리 핀 번호',AMBER)
    for y,a,b,col in [(1180,'+','1 VCC',RED),(1230,'−','2 GND',GRAY)]:
        d.wire(390,670,y,col);d.pin(390,y,a,'right',col);d.pin(670,y,b,'left',col)
    d.box(1310,1090,340,180,'CF-0108 전원 / OE','TXS0108E·모듈 회로 확인 조건',AMBER,True)
    for y,label,pin,col in [(1174,'ESP32 3V3','VCCA / LV',RED),(1202,'J01 CH4 +5 V','VCCB / HV',RED),(1230,'ESP32 3V3','OE *',AMBER),(1258,'공통 GND','GND',GRAY)]:
        d.wire(1090,1310,y,col,label,0,True);d.pin(1310,y,pin,'left',col,16)
    d.note(50,1305,['공통 GND: ESP32·IMU·CF·엔코더·MD20A Signal GND·어댑터 H2 GND와 해당 전원 −. 전력 귀환을 점퍼선에 맡기지 않습니다.',
                    '* OE는 모듈 회로 확인 후 3.3 V HIGH 유지 안입니다. 5 V 연결 금지. 실물 OE 처리·파형 확인 전 CF 결선은 보류합니다.'],size=17,step=26)
    d.save('esp32_gpio_power_v1.svg')


def tables():
    rows=['| 기능 / net | ESP32 | 연결 대상 | 현재 상태 |','|---|---|---|---|']
    for r in DATA['gpio']:
        rows.append(f"| {r['net']} | GPIO{r['gpio']} | {r['endpoint']} | {r['status']} |")
    return '\n'.join(rows)


def main():
    pins=[r['gpio'] for r in DATA['gpio']]
    assert len(pins)==len(set(pins))==11
    assert not set(pins)&set(DATA['excluded'])
    assert not set(pins)&{int(p) for p in DATA['future_reserved']}
    assert not set(pins)&set(DATA['spare_candidates'])
    signals();power()
    (OUT/'esp32_gpio_pin_table_v1.md').write_text('# ESP32 GPIO 연결표 v1\n\n[설계 설명](../ESP32_GPIO_WIRING_2026_11_11.md) · [배선도](esp32_gpio_wiring_v1.html)\n\n'+tables()+'\n\n자동 생성: `python scripts/render_esp32_wiring.py`. 원본: [esp32_gpio_v1.json](esp32_gpio_v1.json).\n',encoding='utf-8')
    th='<tr><th>기능</th><th>ESP32</th><th>연결 대상</th><th>상태</th></tr>'
    trs=''.join('<tr>'+''.join('<td>'+esc(str(t))+'</td>' for t in (r['net'],f"GPIO{r['gpio']}",r['endpoint'],r['status']))+'</tr>' for r in DATA['gpio'])
    images=''.join(f'<section><h2>{heading}</h2><p><a href="{name}.svg">SVG 원본</a> · <a href="{name}.png">PNG 이미지</a></p><div class="drawing">{(OUT/(name+".svg")).read_text(encoding="utf-8")}</div></section>' for heading,name in [('01 · 신호 배선','esp32_gpio_signals_v1'),('02 · 전원과 모터·서보 결선','esp32_gpio_power_v1')])
    refs=''.join(f'<li><a href="{esc(s["url"],quote=True)}">{esc(s["title"])}</a></li>' for s in DATA['sources'])
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IT ARENA · ESP32 배선도 v1</title>
<style>body{margin:0;background:#eaf0f7;color:#14273d;font-family:"Malgun Gothic",sans-serif}main{max-width:1700px;margin:auto;padding:28px}h1{font-size:30px}h2{margin:0}p,li{line-height:1.7}section{margin:24px 0;background:white;border:1px solid #ccd6e4;border-radius:12px;padding:20px}.drawing{overflow:auto}.drawing svg{width:100%;height:auto;min-width:1050px}a{color:#2161ae}table{border-collapse:collapse;width:100%;font-size:15px}td,th{padding:12px;border-bottom:1px solid #d8e0e9;text-align:left}th{background:#eaf0f7}.notice{border-left:5px solid #a86800;padding:8px 18px;background:#fff5dd}@media print{@page{size:A3 landscape;margin:10mm}main{padding:0}section{break-before:page;padding:0;border:0}.drawing svg{min-width:0;max-height:255mm}section p{display:none}a{color:inherit}}</style>
<main><h1>ESP32 주변 배선도 v1</h1><p>2026-11-11 · 현재 IMU 배선을 유지하고 모터 드라이버, 엔코더, 조향 서보까지 확장한 모듈 간 결선 설계입니다.</p>
<p class="notice">실물 검증 완료 도면이 아닙니다. CF-0108의 칩·패드·OE 및 변환 파형은 미확인입니다. GPIO 번호는 물리 헤더 순번이 아닙니다. 현재 IMU 진단 펌웨어로는 모터·서보를 제어하지 않습니다.</p>
<p><a href="../ESP32_GPIO_WIRING_2026_11_11.md">설계 설명과 조립 조건</a> · <a href="esp32_gpio_v1.json">기계 판독 핀·전원 목록</a> · <a href="esp32_gpio_pin_table_v1.md">핀표</a></p>'''+images+'<section><h2>GPIO 연결표</h2><p>방향은 ESP32 기준입니다. 서보 어댑터는 TX→TX / RX←RX를 따릅니다.</p><table>'+th+trs+'</table></section><section><h2>근거</h2><ul>'+refs+'</ul></section></main></html>'
    (OUT/'esp32_gpio_wiring_v1.html').write_text(page,encoding='utf-8')
    print('Generated 2 SVG sheets, standalone HTML, pin table. 11 distinct GPIO; no excluded/reserved collisions.')


if __name__=='__main__':
    main()
