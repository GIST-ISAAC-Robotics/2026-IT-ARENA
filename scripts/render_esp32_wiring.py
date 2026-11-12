"""ESP32 배선 설계의 SVG/HTML/핀표 생성. 실제 하드웨어를 제어하지 않는다."""
from pathlib import Path
from html import escape as esc
import json

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/hardware/schematics'
DATA = json.loads((OUT / 'esp32_gpio_v1.json').read_text(encoding='utf-8'))
BLUE, GREEN, PURPLE = '#2161ae', '#08776d', '#7641a7'
TEAL = '#087f96'
RED, GRAY, AMBER = '#ba3c36', '#576579', '#a86800'
INK = '#14273d'


class Drawing:
    def __init__(self, title, subtitle, height, width=1700):
        self.height = height
        self.parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}" role="img" aria-label="{esc(title)}">',
                      '<style>text{font-family:"Malgun Gothic","Noto Sans KR",sans-serif;fill:#14273d} .small{font-size:17px}</style>',
                      f'<rect width="{width}" height="{height}" fill="#f6f8fc"/>']
        self.text(50, 54, title, 31, bold=True)
        self.text(50, 90, subtitle, 19, color=GRAY)
        self.line(50, 113, width-50, 113, '#cad4e0', 1)

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
        self.text(50, self.height-26, f'IT ARENA · {DATA["updated"]} · v1 조립 인계 | 단자 이름 기준 · ①~④는 관리명이며 실제 핀 위치/순번이 아님', 16, GRAY)
        (OUT/name).write_text('\n'.join(self.parts+['</svg>'])+'\n', encoding='utf-8')



def unified():
    d=Drawing('ESP32 전체 배선도 · 신호 + 전원 + 분기',
              '부품별 한 번만 표시 · 단자 이름 기준 · 실제 보드 핀 위치/순서와 배선 길이는 나타내지 않음',2290,2740)
    def route(points,color=BLUE,width=3):
        path='M'+' L'.join(f'{x},{y}' for x,y in points)
        d.parts.append(f'<path d="{path}" fill="none" stroke="#f6f8fc" stroke-width="{width+6}" stroke-linejoin="round"/>')
        d.parts.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"/>')
    def dot(x,y,color,r=6):
        d.parts.append(f'<circle cx="{x}" cy="{y}" r="{r}" fill="{color}"/>')
    def horizontal(x1,x2,y,left,right,color=BLUE,label=''):
        route([(x1,y),(x2,y)],color)
        d.pin(x1,y,left,'right',color);d.pin(x2,y,right,'left',color)
        if label:d.text((x1+x2)/2,y-12,label,16,color,anchor='middle')
    def vertical(x,y,label,below=False,color=INK):
        d.parts.append(f'<circle cx="{x}" cy="{y}" r="4" fill="white" stroke="{color}" stroke-width="2"/>')
        if label:d.text(x,y+(30 if below else -18),label,18,color,anchor='middle')
    def sleeve(x,y,w,h,label='2심'):
        d.parts.append(f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="8" fill="#e8eef7" stroke="{GRAY}" stroke-width="2"/>')
        d.text(x+w/2,y+h/2+6,label,16,GRAY,anchor='middle')

    d.text(50,140,'빨강 +  |  회색 GND/−  |  초록 SPI  |  파랑 GPIO/USB  |  보라 UART/서보  |  주황 엔코더',19,GRAY)
    d.text(1950,140,'● 접속점 · 흰 틈의 교차는 미접속',20,INK,True)
    d.text(1510,90,'굵은 선 = 케이블 1개 · 내부 도체는 서로 분리',18,BLUE)

    # Uniform columns: sensors | MCU | actuators. PDBs sit together above.
    d.box(1130,165,340,115,'6S 배터리','22.2 V 공칭 / 25.2 V 완충',RED)
    d.box(60,340,660,300,'PDB · J01','CN1 입력 21–36 V DC',RED)
    d.box(1880,340,660,300,'PDB · M01','CN1 입력 24–36 V DC',RED)
    d.box(1050,520,500,280,'Jetson Orin Nano',color=BLUE)
    d.box(340,690,380,120,'B0390 카메라','CSI · 실물 22↔22핀 FPC',GREEN)
    d.box(1880,690,380,120,'RPLIDAR C1','지급 USB 어댑터 경유',TEAL)
    d.box(1050,880,500,1000,'ESP32-S3 호환 보드','Flash 16 MB / PSRAM 8 MB',BLUE)
    d.box(340,880,380,450,'Adafruit 4438','LSM6DSOX · 하단 긴 핀열',GREEN)
    d.box(340,1400,380,340,'CF-0108','기능 확인한 교체 보드 · 채널 1/2',AMBER)
    d.box(340,1840,380,240,'PGE-213','모터 뒤 장착 · NPN OC / 내부 풀업',AMBER)
    d.box(1050,1950,500,130,'STOP 스위치','2선 · I/O 유지형',BLUE)
    d.box(1880,880,380,130,'MB4266-24180','브러시 DC 모터',INK)
    d.box(1880,1070,380,280,'Cytron MD20A','3.3 V PWM/DIR 직접 입력',BLUE)
    d.box(1880,1420,380,120,'ST3215-HS × 1','서보혼·링크 장착 교정은 후속',PURPLE)
    d.box(1880,1590,380,280,'Bus Servo Adapter (A)','점퍼 두 개 모두 A · H2 사용',PURPLE)

    # A single battery pair enters a visible Y harness. Each thick arm is TWO cores.
    # Sleeves show where the isolated +/− conductors become one cable symbol.
    for x,label,col in [(1260,'+',RED),(1340,'−',GRAY)]:
        vertical(x,280,label,color=col)
        route([(x,280),(x,300)],col)
    sleeve(1240,300,120,35)
    route([(1300,335),(1300,370)],INK,9)
    route([(1300,370),(1160,445),(840,445)],INK,9)
    route([(1300,370),(1440,445),(1760,445)],INK,9)
    dot(1300,370,INK,7)
    sleeve(800,410,40,70)
    sleeve(1760,410,40,70)
    for y,col,label in [(430,RED,'CN1 +'),(465,GRAY,'CN1 −')]:
        route([(720,y),(800,y)],col)
        d.pin(720,y,label,'right',col)
        route([(1800,y),(1880,y)],col)
        d.pin(1880,y,label,'left',col)
    d.text(1400,325,'기존 XT60↔XT30 + Y 케이블',18,GRAY)
    d.text(1300,490,'Y 분기 · 상부 1단 / J01 근처',22,INK,True,anchor='middle')
    d.text(940,415,'J01행 · +/− 2심',18,RED,anchor='middle')
    d.text(1660,415,'M01행 · +/− 2심',18,RED,anchor='middle')

    # Upper electronics: barrel and sensor cables get the same generous corridor.
    for y,col,label in [(550,RED,'CN2(CH1) +'),(600,GRAY,'CN2(CH1) −')]:
        route([(720,y),(820,y)],col);d.pin(720,y,label,'right',col)
    sleeve(820,535,70,80)
    route([(890,575),(1050,575)],GRAY,8)
    d.pin(1050,575,'DC IN · 배럴잭','left',GRAY)
    d.text(855,520,'19 V / 최대2.3 A',16,RED,anchor='middle')
    d.text(855,640,'배럴 전원 케이블 1개',16,GRAY,anchor='middle')
    d.text(1070,620,'5.5×2.5 mm · 중앙 + / 외곽 −',17,GRAY)
    d.text(1070,686,'카메라·LiDAR·ESP32 연결',19,GRAY)
    route([(720,765),(1050,765)],GREEN,7)
    d.pin(720,765,'','right',GREEN);d.pin(1050,765,'CSI','left',GREEN)
    d.text(885,750,'FPC 1개 · 영상/전원',17,GREEN,anchor='middle')
    route([(1550,765),(1880,765)],TEAL,7)
    d.pin(1550,765,'USB-A','right',TEAL);d.pin(1880,765,'','left',TEAL)
    d.text(1715,750,'USB 1개 · 데이터/전원',17,TEAL,anchor='middle')
    route([(1300,800),(1300,880)],BLUE,7)
    d.text(1340,847,'USB-A → USB-C · 데이터 / 5 V / GND',18,BLUE)
    d.text(1070,975,'USB-UART 포트 사용',17,GRAY)
    d.note(80,445,['CN5(CH4): 5 V / 최대0.5 A → 엔코더·CF',
                   '예비 CH2+3: 합계1.5 A',
                   '예비 CH5+6: 합계0.6 A'],size=17,step=31)

    # Sensor power rails run outside the component column.
    vertical(160,640,'CN5 +',color=RED);vertical(240,640,'CN5 −',color=GRAY)
    route([(160,640),(160,1940)],RED)
    route([(240,640),(240,2010)],GRAY)
    for y,pin in [(1495,'VB'),(1940,'1 VCC')]:
        route([(160,y),(340,y)],RED);d.pin(340,y,pin,'left',RED)
    for y,pin in [(1530,'GND'),(2010,'2 GND')]:
        route([(240,y),(340,y)],GRAY);d.pin(340,y,pin,'left',GRAY)
    route([(240,1790),(1050,1790)],GRAY);d.pin(1050,1790,'GND②','left',GRAY)
    d.text(50,1470,'T5V · 5 V',18,RED,True)
    d.text(260,1770,'TGND',18,GRAY,True)

    for y,imu,esp,col,label in [
        (1010,'VIN','3V3①',RED,'3.3 V'),(1055,'GND','GND①',GRAY,''),
        (1100,'SCL','GPIO12',GREEN,'SCLK'),(1145,'SDA','GPIO11',GREEN,'MOSI'),
        (1190,'DO','GPIO13',GREEN,'MISO'),(1235,'CS','GPIO10',GREEN,'CS'),
        (1280,'I1','GPIO9',GREEN,'INT1')]:
        horizontal(720,1050,y,imu,esp,col,label)

    route([(720,1470),(1050,1470)],RED)
    d.pin(720,1470,'VA','right',RED);d.pin(1050,1470,'3V3②','left',RED)
    route([(720,1515),(875,1515),(875,1470)],RED);d.pin(720,1515,'OE','right',RED)
    d.text(875,1445,'T3V3',20,RED,True,anchor='middle')
    horizontal(720,1050,1595,'A1','GPIO4',AMBER,'ENC A · 3.3 V')
    horizontal(720,1050,1655,'A2','GPIO5',AMBER,'ENC B · 3.3 V')
    for x,cf,enc in [(510,'B1','3 HA'),(625,'B2','4 HB')]:
        route([(x,1740),(x,1840)],AMBER)
        vertical(x,1740,cf,color=AMBER);vertical(x,1840,enc,True,AMBER)
    d.text(360,1575,'VA/OE=3.3 V · VB=5 V',17,AMBER)
    d.note(1070,1370,['3V3 2핀 / GND 4핀 사용','①~④는 배선 관리명'],size=19,step=29)
    d.text(1070,1825,'GPIO43/44: 보드 내부 USB-UART 예약',17,GRAY)

    # Switch below MCU, no cramped slot between encoder and MCU.
    route([(1400,1880),(1400,1950)],BLUE)
    vertical(1400,1880,'GPIO8 · 내부 풀업',color=BLUE)
    vertical(1400,1950,'선 1',True,BLUE)
    route([(900,1790),(900,2045),(1050,2045)],GRAY)
    d.pin(1050,2045,'선 2 · GND','left',GRAY)
    d.text(1300,2025,'O/개방 = STOP',17,BLUE)
    d.text(1260,2058,'I 복귀만으로 출발 안 함',16,GRAY)

    # Motor and servo share an aligned actuator column, with power lanes outside.
    for y,gpio,pin,col,label in [(1170,'GPIO15','PWM',BLUE,'PWM'),
        (1220,'GPIO6','DIR',BLUE,'방향'),(1270,'GND③','GND',GRAY,'신호 GND')]:
        horizontal(1550,1880,y,gpio,pin,col,label)
    for x,drv,motor in [(1980,'MA','6 (+)'),(2160,'MB','5 (−)')]:
        route([(x,1010),(x,1070)],INK,5)
        vertical(x,1010,motor,color=INK);vertical(x,1070,'',color=INK)
        d.text(x+15,1048,drv,17,INK)
    d.text(1900,1325,'PWM LOW = brake · MA/MB ≠ GND',17,GRAY)

    power_routes=[
        (430,2660,1170,RED,'CN2(CH1) +','VB+','배터리 전압'),
        (480,2700,1220,GRAY,'CN2(CH1) −','VB−','전력 귀환'),
        (550,2580,1700,RED,'CN6(CH5) +','전원 +','12 V / 최대2 A'),
        (600,2620,1755,GRAY,'CN6(CH5) −','전원 −','전력 귀환')]
    for sy,lane,dy,col,pdb,load,label in power_routes:
        route([(2540,sy),(lane,sy),(lane,dy),(2260,dy)],col)
        d.pin(2540,sy,pdb,'right',col);d.pin(2260,dy,load,'right',col)
        d.text(2410,dy-14,label,16,col,anchor='middle')
    d.note(2040,445,['CH1~4: 입력 전압 그대로','고정 A 정격 미제시',
                    'CH5/6: 각 12 V / 최대2 A','CH2~4·CH6은 예비'],size=16,step=46)

    for y,gpio,pin,col,label in [(1700,'GPIO17','TX / TXD',PURPLE,'TX → TX'),
        (1755,'GPIO18','RX / RXD',PURPLE,'RX ← RX'),(1810,'GND④','H2 GND',GRAY,'신호 GND')]:
        horizontal(1550,1880,y,gpio,pin,col,label)
    route([(2070,1540),(2070,1590)],PURPLE,9)
    vertical(2070,1540,'서보 3핀',color=PURPLE)
    vertical(2070,1590,'',color=PURPLE)
    d.text(2100,1575,'전용 3선 케이블 1개',17,PURPLE)
    d.text(1900,1844,'전원: DC 5.5×2.1 또는 전원 단자',16,GRAY)
    d.note(1900,1965,['모터·서보 전력 −는 PDB로 직접 귀환',
                     '신호 GND로 부하 전류를 흘리지 않음',
                     'M01 CH5/6 병렬 연결 금지'],size=17,step=32)

    for x,y,col in [(160,1495,RED),(240,1530,GRAY),(240,1790,GRAY),
                    (900,1790,GRAY),(875,1470,RED)]:dot(x,y,col)
    d.note(50,2140,['TGND = J01 CN5− + 엔코더 GND + CF GND + ESP32 GND② + STOP 선 2 (총 5접점)',
                    'CN1=배터리 입력 · CH1~6 ↔ CN2~7 · 분기점은 절연 납땜 또는 공통 단자대로 제작',
                    'Y선의 굵은 가지는 각각 +/− 2심입니다. +끼리, −끼리 분기하며 +와 −를 서로 연결하지 않습니다.',
                    '기존 정격 제한: 6S 운용 중 M01 입력24 V 미만 가능. 전체 하네스 통전·부하 검증은 별도입니다.'],size=18,step=29)
    d.save('esp32_gpio_all_v1.svg')


def gpio_rows():
    return [(r['net'],f"GPIO{r['gpio']}",r['endpoint'],r['status']) for r in DATA['gpio']]


def header_rows():
    return [(r['pin'],r['destination'],r['role']) for r in DATA['header_allocation']]


def pdb_rows():
    rows=[]
    for board,b in DATA['pdb_boards'].items():
        rows.append((board,'입력',b['input']['connector'],b['input']['voltage'],'—','배터리 + / −'))
        rows.extend((board,f"CH{r['ch']}",f"CN{r['cn']}",r['voltage'],r['current'],r['load']) for r in b['outputs'])
    return rows


def md_table(headers,rows):
    return '\n'.join(['| '+' | '.join(headers)+' |','|'+'---|'*len(headers)]+['| '+' | '.join(row)+' |' for row in rows])


def html_table(headers,rows):
    return '<div class="table-wrap"><table><thead><tr>'+''.join('<th>'+esc(t)+'</th>' for t in headers)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+esc(t)+'</td>' for t in row)+'</tr>' for row in rows)+'</tbody></table></div>'


def validate():
    pins=[r['gpio'] for r in DATA['gpio']]
    assert len(pins)==len(set(pins))==12
    assert not set(pins)&set(DATA['excluded'])
    assert not set(pins)&{int(p) for p in DATA['future_reserved']}
    assert not set(pins)&set(DATA['spare_candidates'])
    assert [r['pin'] for r in DATA['header_allocation']]==['3V3①','3V3②','GND①','GND②','GND③','GND④']
    assert DATA['header_inventory']['3V3']==2 and DATA['header_inventory']['GND']==4
    nodes={r['id']:set(r['members']) for r in DATA['junctions']}
    assert nodes['T3V3']=={'ESP32 3V3②','CF VA','CF OE'}
    assert nodes['T5V']=={'J01 CN5(CH4)+','PGE-213 pin 1 VCC','CF VB'}
    assert nodes['TGND']=={'ESP32 GND②','J01 CN5(CH4)−','PGE-213 pin 2 GND','CF GND','STOP 스위치 선 2'}
    assert next(r for r in DATA['gpio'] if r['gpio']==8)['net']=='STOP_REQUEST'
    assert DATA['hardware_stop']['schematic_connected']
    assert DATA['hardware_stop']['input_mode']=='GPIO input with internal pull-up'
    assert not nodes['T3V3']&nodes['T5V']
    for b in DATA['pdb_boards'].values():
        assert b['input']['connector']=='CN1'
        assert [(r['ch'],r['cn']) for r in b['outputs']]==[(n,n+1) for n in range(1,7)]
    assert DATA['servo_power_channel']=='M01 CN6(CH5)'
    assert len(DATA['servo_cable']['signals'])==3


def main():
    validate();unified()
    gh=['기능 / net','ESP32','연결 대상','확인 범위']
    hh=['ESP32 관리명','연결 대상','역할']
    ph=['보드','채널','커넥터','전압','출력 전류 한도','조립 기준 연결']
    jh=['분배점','공통으로 연결할 단자','제작']
    jr=[(j['id'],' ↔ '.join(j['members']),j['implementation']) for j in DATA['junctions']]
    text='# ESP32 GPIO·전원 연결표 v1\n\n'+f"갱신: {DATA['updated']}. ①~④는 관리명이며 물리 핀 순번이 아니다.\n\n"
    text+='[설계 설명](../ESP32_GPIO_WIRING_2026_11_11.md) · [전체 배선도](esp32_gpio_wiring_v1.html)\n\n'
    for title,heads,rows in [('전원 헤더: 3V3 2개 / GND 4개',hh,header_rows()),('공통 분배점',jh,jr),('GPIO 신호',gh,gpio_rows()),('PDB 채널·커넥터·정격',ph,pdb_rows())]:
        text+='## '+title+'\n\n'+md_table(heads,rows)+'\n\n'
    text+='서보 어댑터와 ST3215-HS는 **3선 전용 케이블 1개**로 연결한다(D/DATA, V+, G/GND). 이는 기능 이름이며 실제 좌우 핀 순서가 아니다. 서보 전원 CH5(CN6)는 조립 기준 선택이며 CH6(CN7)은 예비다.\n\n'
    text+='배터리 공통 +/− 2심을 상부 1단 J01 근처까지 올린 뒤 Y 케이블로 J01 CN1·M01 CN1에 분기한다. 각 가지에서 +끼리, −끼리를 연결한다.\n\n'
    text+='케이블 묶음 표기: J01 CN2의 +/− 두 선은 한 전원 케이블로 Jetson 배럴잭에 연결한다. B0390은 22↔22 FPC 1개, C1은 지급 어댑터를 거친 USB 데이터 케이블 1개다. 굵은 선은 케이블 외피를 뜻하며 내부 도체를 단락시키는 표시가 아니다. [층별 케이블 배치도](vehicle_wiring_layers_v1.html).\n\n'
    text+='전류 한도는 소비 전류가 아니다. J01 CH2+3 및 CH5+6은 각각 합계 한도, M01 CH5/6은 각 채널 한도다. 모터·서보 전력 귀환은 PDB로 직접 배선한다.\n\n'
    text+='물리 STOP: 스위치 선 1 → GPIO8, 선 2 → TGND(기존 공통 GND에 1분기 추가). GPIO8은 내부 풀업 입력으로 설정하며 별도 3V3선은 연결하지 않는다. O/개방은 HIGH=STOP, I/도통은 LOW=허가 조건이다. I로 복귀해도 자동 출발하지 않는다. 무전원 도통 검사로 I/O 접점을 확인한다. 배선 설계 반영이며 펌웨어·실물 STOP 시험 완료를 뜻하지 않는다.\n\n'
    text+='자동 생성: `python scripts/render_esp32_wiring.py`. 원본: [esp32_gpio_v1.json](esp32_gpio_v1.json).\n'
    (OUT/'esp32_gpio_pin_table_v1.md').write_text(text,encoding='utf-8')
    images=''.join(f'<section><h2>{heading}</h2><p><a href="{name}.svg">SVG 원본</a> · <a href="{name}.png">PNG 이미지</a></p><div class="toolbar"><button onclick="zoomDiagram(0)">전체 보기</button><button onclick="zoomDiagram(1)">원본 크기</button><span>선 교차의 ●만 연결점입니다.</span></div><div class="drawing">{(OUT/(name+".svg")).read_text(encoding="utf-8")}</div></section>' for heading,name in [('전체 배선 · 한 장','esp32_gpio_all_v1')])
    refs=''.join(f'<li><a href="{esc(s["url"],quote=True)}">{esc(s["title"])}</a></li>' for s in DATA['sources'])
    page='<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IT ARENA · ESP32 배선도 v1</title>'
    page+='<style>body{margin:0;background:#eaf0f7;color:#14273d;font-family:"Malgun Gothic",sans-serif}main{max-width:2740px;margin:auto;padding:28px}h1{font-size:30px}h2{margin:0}p,li{line-height:1.7}section{margin:24px 0;background:white;border:1px solid #ccd6e4;border-radius:12px;padding:20px}.drawing,.table-wrap{overflow:auto}.drawing svg{width:100%;height:auto;min-width:1200px}.toolbar{display:flex;gap:8px;align-items:center;margin:12px 0}summary{cursor:pointer;padding:10px 0}button{padding:8px 14px;border:1px solid #b7c8da;border-radius:6px;background:white;cursor:pointer}a{color:#2161ae}table{border-collapse:collapse;width:100%;font-size:15px}td,th{padding:12px;border-bottom:1px solid #d8e0e9;text-align:left}th{background:#eaf0f7}.notice{border-left:5px solid #a86800;padding:8px 18px;background:#fff5dd}@media print{@page{size:A3 landscape;margin:10mm}main{padding:0}section{break-before:page;padding:0;border:0}.drawing svg{min-width:0;max-height:245mm}section p{display:none}a{color:inherit}}</style>'
    page+=f'<main><h1>ESP32 주변 배선도 v1 · 하드웨어팀 조립용</h1><p>{DATA["updated"]} 갱신 · 신호·전원·분기를 부품별로 연결한 한 장 · 3V3 2개와 GND 4개를 사용합니다.</p>'
    page+='<details><summary>조립 표기와 확인 범위</summary><p class="notice">①~④는 배선 관리명이며 보드의 물리적 위치·핀 순번이 아닙니다. T3V3는 CF VA·OE용 3.3 V 분기, T5V는 엔코더·CF VB용 5 V 분기, TGND는 센서·STOP 공통 접지입니다. STOP 선 1은 GPIO8, 선 2는 TGND에 연결합니다. GPIO8 내부 풀업을 사용하므로 별도 3V3선은 없고, ESP32 GND는 기존 4핀 그대로입니다. 각 단자의 실크를 찾아 연결하고 사용 위치에 라벨을 붙이십시오.</p><p>STOP 설계 동작: O/개방=HIGH=정지, I/도통=LOW=허가 조건. I 복귀만으로 자동 출발하지 않습니다. 접점은 무전원에서 확인하며 펌웨어 구현·실물 STOP 시험과 규정상 인정 여부는 별도입니다.</p>'
    page+='<p>IMU SPI·서보 무부하 왕복·엔코더 손 회전 및 약5% 모터 구동 중 계수를 확인했습니다. 전원 헤더 재분배와 서보 CH5 선택은 이번 조립 기준이며 전체 하네스 실물 검증이 아닙니다. 고속 파형·장착 교정·부하 전류·운영 제어는 별도입니다. 과거 실패와 종료 후 해체 구간 신호는 각 시험 보고서에 보존했습니다.</p></details>'
    page+='<p><a href="../ESP32_GPIO_WIRING_2026_11_11.md">설계 설명</a> · <a href="esp32_gpio_v1.json">핀·전원 JSON</a> · <a href="esp32_gpio_pin_table_v1.md">연결표 Markdown</a> · <a href="vehicle_wiring_layers_v1.html">층별 케이블 배치도</a></p>'+images
    page+='<section><h2>PDB 전체 채널과 정격</h2><p>두 보드 모두 CN1=배터리 입력, CHn=CN(n+1). 제조사 Preliminary PDF p.3을 대조했습니다. 최대 전류는 소비 전류가 아닙니다. J01의 공유 채널 합계 한도와 M01의 채널별 한도를 구분하십시오.</p>'+html_table(ph,pdb_rows())+'</section>'
    page+='<section><h2>전원 헤더와 분배점</h2><p>모터·서보 전류는 각 전원 −로 직접 귀환합니다. ESP32 GND와 센서 점퍼선을 전력 귀환 경로로 사용하지 않습니다.</p>'+html_table(hh,header_rows())+html_table(jh,jr)+'</section>'
    page+='<section><h2>GPIO 신호 연결표</h2><p>서보 어댑터 H2는 TX→TX / RX←RX. 어댑터와 서보 사이는 보유한 3선 전용 케이블 1개를 사용합니다.</p>'+html_table(gh,gpio_rows())+'</section>'
    page+='<section><h2>근거</h2><ul>'+refs+'</ul></section></main><script>function zoomDiagram(full){const svg=document.querySelector(".drawing svg");svg.style.width=full?"2740px":"100%";}</script></html>'
    (OUT/'esp32_gpio_wiring_v1.html').write_text(page,encoding='utf-8')
    print('Generated 1 unified SVG sheet, standalone HTML and connection tables; 12 GPIO, 6 power pins, 3 junctions including 5-way TGND, STOP and CH/CN mapping validated.')


if __name__=='__main__':
    main()
