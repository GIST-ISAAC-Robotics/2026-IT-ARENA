"""UNO Q 대체 배선의 SVG/HTML/연결표 생성. 하드웨어 접근 없음."""
from pathlib import Path
from html import escape as esc
import json
import xml.etree.ElementTree as ET
from render_esp32_wiring import Drawing, BLUE, GREEN, PURPLE, TEAL, RED, GRAY, AMBER, INK

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/hardware/schematics'
DATA = json.loads((OUT / 'uno_q_v1.json').read_text(encoding='utf-8'))
NAME = 'uno_q_all_v1'
WIDTH, HEIGHT = 3000, 3080


class Sheet(Drawing):
    def route(self, points, color=BLUE, width=3):
        path = 'M' + ' L'.join(f'{x},{y}' for x,y in points)
        self.parts.append(f'<path d="{path}" fill="none" stroke="#f6f8fc" stroke-width="{width+6}" stroke-linejoin="round"/>')
        self.parts.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"/>')

    def dot(self, x, y, color):
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="6" fill="{color}"/>')

    def horizontal(self, x1, x2, y, left, right, color, label=''):
        self.route([(x1,y),(x2,y)],color)
        self.pin(x1,y,left,'right',color,18)
        self.pin(x2,y,right,'left',color,18)
        if label:self.text((x1+x2)/2,y-14,label,16,color,anchor='middle')

    def save(self, name):
        self.text(50,HEIGHT-28,'IT ARENA · 2026-11-14 · UNO Q v1 / 부품 간 배선 설계 · 실제 핀 위치/배선 길이 아님 · 통전/구동 미실행',17,GRAY)
        source='\n'.join(self.parts+['</svg>'])+'\n'
        ET.fromstring(source)
        (OUT/name).write_text(source,encoding='utf-8')


def unified():
    d=Sheet('ESP32 대체 전체 배선도 · Jetson + Arduino UNO Q',
            'UNO Q STM32: 모터·엔코더·IMU / Jetson: 판단·카메라·라이다·USB 조향 / 기존 PDB와 센서·구동부 유지',HEIGHT,WIDTH)
    d.text(50,145,'실선 = 선정한 연결 설계  |  ● 접속점 · 흰 틈의 교차는 미접속  |  J12는 물리 핀, D/A는 UNO Q 실크 이름',20,INK,True)
    d.text(2030,145,'펌웨어 이식·무전원 출력 검사 전 구동 금지',20,RED,True)
    for args in [
        (1240,180,370,115,'6S 배터리','공칭22.2 V / 만충25.2 V',RED),
        (100,350,700,330,'PDB · J01','CN1 입력21–36 V DC',RED),
        (2050,350,650,330,'PDB · M01','CN1 입력24–36 V DC',RED),
        (1070,710,600,400,'Jetson Orin Nano Super','기존19 V · ROS/판단·카메라·라이다 유지',BLUE),
        (350,780,450,155,'B0390 카메라','기존22↔22 FPC 유지',GREEN),
        (2050,780,450,155,'RPLIDAR C1','기존 USB 어댑터 유지',TEAL),
        (1070,1300,600,1390,'Arduino UNO Q','STM32U585 · 3.3 V MCU 헤더 사용',BLUE),
        (350,1450,450,520,'Adafruit 4438','LSM6DSOX · SPI 유지 / 하단 긴 핀열',GREEN),
        (350,2050,450,350,'CF-0108 / TXS0108E','기존 기능 확인한 교체 보드 · 채널1/2',AMBER),
        (350,2500,450,250,'PGE-213 엔코더','모터축 · NPN open-collector / 5 V 유지',AMBER),
        (2050,1300,450,120,'MB4266-24180','기존 MA/MB 연결 유지',INK),
        (2050,1500,450,310,'Cytron MD20A','기존 드라이버 · 3.3 V 제어 입력',BLUE),
        (2050,2070,450,120,'ST3215-HS × 1','중앙2811 / 왼쪽3031 / 오른쪽2550 보존',PURPLE),
        (2050,2280,450,340,'Bus Servo Adapter(A)','USB-C 사용 · 점퍼캡 두 개 B',PURPLE)]:d.box(*args)

    # 배터리 Y: 굵은 각 가지는 분리된 +/− 두 도체다.
    for x,col,label in [(1385,RED,'+'),(1465,GRAY,'−')]:
        d.route([(x,295),(x,320)],col);d.text(x,282,label,20,col,anchor='middle')
    d.parts.append('<rect x="1365" y="320" width="120" height="35" rx="6" fill="#e8eef7" stroke="#576579"/>')
    d.text(1425,344,'+/− 2심',16,GRAY,anchor='middle')
    for path in [[(1425,355),(1425,400)],[(1425,400),(1270,480),(900,480)],[(1425,400),(1580,480),(1950,480)]]:d.route(path,INK,9)
    d.dot(1425,400,INK)
    for x in (870,1950):d.parts.append(f'<rect x="{x}" y="438" width="30" height="85" rx="5" fill="#e8eef7" stroke="#576579"/>')
    for y,col,label in [(455,RED,'CN1 +'),(505,GRAY,'CN1 −')]:
        d.route([(800,y),(870,y)],col);d.pin(800,y,label,'right',col)
        d.route([(1980,y),(2050,y)],col);d.pin(2050,y,label,'left',col)
    d.text(1425,550,'기존 Y 분기 · 각 가지 +/− 2심',21,INK,True,anchor='middle')
    d.text(1425,595,'발열 ESP32 본체·USB·전원·신호선은 모두 제거',20,RED,True,anchor='middle')
    d.note(120,450,['CH1/CN2: 19 V·2.3 A → Jetson','CH4/CN5: 5 V·0.5 A → 엔코더/CF','CH2/3의5 V는 합계1.5 A · UNO Q에 미사용'],size=18,step=43)
    d.note(2200,550,['CH1: 배터리 전압 → MD20A','CH5: 12 V·2 A → 서보','CH6: 별도12 V·2 A → UNO Q'],size=17,step=37)

    # Jetson DC IN과 카메라/라이다.
    for y,col,label in [(590,RED,'CN2 +'),(640,GRAY,'CN2 −')]:
        d.route([(800,y),(860,y)],col);d.pin(800,y,label,'right',col)
    d.parts.append('<rect x="860" y="575" width="45" height="80" rx="6" fill="#e8eef7" stroke="#576579"/>')
    d.route([(905,615),(950,615),(950,810),(1070,810)],GRAY,8)
    d.pin(1070,810,'DC IN 배럴잭','left',GRAY,18)
    d.horizontal(800,1070,870,'FPC','CSI',GREEN,'22↔22 FPC 1개')
    d.horizontal(1670,2050,870,'USB-A','USB',TEAL)

    # 운영 통신은 MCU의 직접 UART.
    for x,col,top,bottom in [(1170,BLUE,'J12 8 · TX','D0 · RX'),(1370,PURPLE,'J12 10 · RX','D1 · TX'),(1570,GRAY,'J12 14 · GND','GND①')]:
        d.route([(x,1110),(x,1300)],col)
        d.text(x,1085,top,18,col,anchor='middle');d.text(x+12,1280,bottom,18,col)
    d.text(1020,1180,'3.3 V UART · 1 Mbps / 8N1',19,BLUE,True,anchor='end')
    d.text(1020,1220,'TX→RX / RX←TX · 신호 GND',18,GRAY,anchor='end')
    d.note(1090,950,['UART 핀mux/콘솔 소유권 설정 후 사용','보호층 → /drive/safe → MCU 전송','UNO Q USB는 운영 중 미연결'],size=18,step=32)

    # M01 CH6 독립 전원: VIN 전용.
    for y,target,col in [(610,1380,RED),(660,1430,GRAY)]:
        x=1780 if col==RED else 1830
        d.route([(2050,y),(x,y),(x,target),(1670,target)],col,4)
        d.pin(2050,y,'CN7 +' if col==RED else 'CN7 −','left',col,18)
        d.pin(1670,target,'VIN · 12 V' if col==RED else 'GND② · 전원','right',col,18)
    d.note(1090,1485,['VIN 범위7–24 V · 공급은CH6의12 V','5V/3V3 핀에12 V 연결 금지'],size=17,step=29)

    # IMU SPI, IRQ, 전원.
    for y,left,right,col in [(1560,'VIN','3V3 → T3V3',RED),(1610,'GND','GND③ → TGND',GRAY),
        (1680,'SCL','D13 · SCK',GREEN),(1740,'SDA','D11 · MOSI',GREEN),(1800,'DO','D12 · MISO',GREEN),
        (1860,'CS','D10 · CS_N',GREEN),(1920,'I1','D2 · INT1',GREEN)]:d.horizontal(800,1070,y,left,right,col)
    d.note(370,2010,['SPI: SJ1 열림 · DO에 전원/GND 스트랩 금지'],size=16)
    d.route([(910,1560),(910,2130),(800,2130)],RED)
    d.route([(910,2130),(910,2180),(800,2180)],RED)
    for x,y in [(910,1560),(910,2130)]:d.dot(x,y,RED)
    d.pin(800,2130,'VA','right',RED,18);d.pin(800,2180,'OE','right',RED,18)
    d.horizontal(800,1070,2250,'A1','A2 · PA6 / TIM3 CH1',AMBER,'ENC A · 3.3 V')
    d.horizontal(800,1070,2320,'A2','A3 · PA7 / TIM3 CH2',AMBER,'ENC B · 3.3 V')
    d.note(1090,2390,['A2/A3는 아날로그가 아닌 타이머 입력','TIM3: quadrature x4 · 기존PWM 해제','D0/D1: USART1 전용 · 콘솔/내부pull 해제','D9: TIM4 CH3 · 20 kHz','D10: GPIO CS · TIM4 CH4 미사용'],size=18,step=36)
    d.note(1090,2600,['GND① JDIGITAL15 · ② JANALOG7',
                       'GND③ JANALOG6 · ④ JSPI6',
                       '①~④는 도면 관리명 · 헤더 GND 실크 확인'],size=17,step=30)

    # 센서 전원·접지 분기. 구동 전력은 이 경로로 귀환시키지 않는다.
    d.route([(170,680),(170,2630),(350,2630)],RED)
    d.route([(250,680),(250,2690),(350,2690)],GRAY)
    d.text(170,660,'CN5 +',17,RED,anchor='middle');d.text(250,660,'CN5 −',17,GRAY,anchor='middle')
    d.route([(170,2130),(350,2130)],RED);d.dot(170,2130,RED)
    d.route([(250,2190),(350,2190)],GRAY);d.dot(250,2190,GRAY)
    d.pin(350,2130,'VB','left',RED,18);d.pin(350,2190,'GND','left',GRAY,18)
    d.route([(1070,1610),(990,1610),(990,2450),(250,2450)],GRAY);d.dot(250,2450,GRAY);d.dot(990,1610,GRAY)
    d.text(270,2420,'TGND · 센서 기준 접지',18,GRAY,True)
    d.pin(350,2630,'1 VCC','left',RED,18);d.pin(350,2690,'2 GND','left',GRAY,18)
    for x,label in [(520,'3 HA'),(670,'4 HB')]:
        d.route([(x,2400),(x,2500)],AMBER)
        d.text(x,2380,'B1' if x==520 else 'B2',18,AMBER,anchor='middle');d.text(x+14,2480,label,18,AMBER)

    # DC 전력 / PWM / DIR / 기준접지.
    for x,label in [(2170,'MA'),(2380,'MB')]:
        d.route([(x,1420),(x,1500)],INK,5);d.text(x+15,1470,label,18,INK)
    for y,left,right,col in [(1600,'D9 · PWM','PWM',BLUE),(1660,'D4 · DIR','DIR',BLUE),(1720,'GND④','신호 GND',GRAY)]:d.horizontal(1670,2050,y,left,right,col)
    d.text(2070,1780,'PWM LOW = brake · MA/MB ≠ GND',17,BLUE)
    for y,ty,col,label in [(430,1580,RED,'CN2 +'),(480,1640,GRAY,'CN2 −')]:
        x=2880 if col==RED else 2930
        d.route([(2700,y),(x,y),(x,ty),(2500,ty)],col,4)
        d.pin(2700,y,label,'right',col,18);d.pin(2500,ty,'VB+' if col==RED else 'VB−','right',col,18)
    d.note(2050,1880,['모터/서보 전류는 각 PDB로 직접 귀환','MCU에서 명령 만료 감시 → PWM0','부팅·접속 복귀만으로 구동하지 않음'],size=18,step=34)

    # 조향 USB와 CH5 전원은 기존 부품 유지.
    d.route([(1670,1020),(1930,1020),(1930,2390),(2050,2390)],PURPLE,5)
    d.pin(1670,1020,'USB-C Host','right',PURPLE,18);d.pin(2050,2390,'USB-C','left',PURPLE,18)
    d.text(1950,2220,'C-to-C · 데이터',17,PURPLE)
    d.route([(2270,2190),(2270,2280)],PURPLE,7);d.text(2300,2240,'서보 전용3선 케이블',17,PURPLE)
    for y,ty,col,label in [(550,2440,RED,'CN6 +'),(600,2500,GRAY,'CN6 −')]:
        x=2780 if col==RED else 2830
        d.route([(2700,y),(x,y),(x,ty),(2500,ty)],col,4)
        d.pin(2700,y,label,'right',col,18);d.pin(2500,ty,'전원 +' if col==RED else '전원 −','right',col,18)
    d.note(2070,2540,['H2 UART TX/RX/GND는 미연결','USB 데이터와12 V 서보 전원은 별개'],size=17,step=32)
    d.note(60,2840,['선정 이유: MCU 하드웨어 엔코더·PWM / TXS→TXB 엔코더 연쇄 제거 / 내부115200 링크를 우회하는 직접UART',
        '전원 조건: M01 CH6/CN7 12 V·2 A → UNO Q VIN. CH5와 병렬 금지. M01 입력24 V 미만은 제조사 정격 밖.',
        '작업 전: 발열보드 분리·외부 하네스 전압/극성 확인. 실제 헤더 방향은 실크 기준. 한쪽 전원OFF·리셋 출력은 실물 검증 필요.',
        '범위: 물리 STOP은 사용자 지시로 제외. 서보·DC·엔코더·IMU·카메라·라이다 유지. 펌웨어 이식·Jetson 핀mux/UART 변경은 아직 미실행.'],size=19,step=39)
    d.save(NAME+'.svg')


def validate():
    names=[s['pin'] for s in DATA['signals']]
    assert len(names)==len(set(names))
    pins={s['net']:s['pin'] for s in DATA['signals']}
    assert (pins['ENC_A'],pins['ENC_B'])==('A2','A3')
    assert (pins['MOTOR_PWM'],pins['MOTOR_DIR'])==('D9','D4')
    assert (pins['HOST_RX'],pins['HOST_TX'])==('D0','D1')
    assert 'STOP_REQUEST' not in pins and 'D7' not in names
    assert DATA['power']['channel']==6 and DATA['power']['connector']=='CN7'
    assert DATA['power']['voltage_v']==12 and DATA['power']['destination']=='VIN'
    assert DATA['operation_usb_to_uno_q'] is False
    ground_pins=[(r['connector'],r['pin']) for r in DATA['ground_headers']]
    assert len(ground_pins)==len(set(ground_pins))==4


def main():
    validate();unified()
    headers=['기능','UNO Q 실크','STM32 핀/기능','연결 대상']
    rows=[(r['net'],r['pin'],r['mcu'],r['endpoint']) for r in DATA['signals']]
    extras=[('전원 VIN','M01 CH6/CN7 + → UNO Q VIN; CN7 − → GND② (JANALOG7)'),
        ('3.3 V','UNO Q 3V3 → IMU VIN + CF VA/OE (T3V3)'),('5 V','J01 CH4/CN5 + → PGE VCC + CF VB (T5V)'),
        ('UART GND','GND① (JDIGITAL15) → Jetson J12 pin14'),
        ('센서 GND','GND③ (JANALOG6) + IMU + CF + PGE + J01 CN5 −: TGND 5접점'),
        ('구동 GND','GND④ (JSPI6) → MD20A 신호 GND; 전력 −는 M01 직접 귀환'),
        ('서보','Jetson USB-C → 데이터 C-to-C → Adapter(A) USB-C, 점퍼2개 B'),
        ('카메라/라이다','기존 Jetson CSI FPC / USB-A 그대로 유지'),
        ('미연결','발열 ESP32 전부, UNO Q USB/5V/IOREF, 물리 STOP, Adapter H2 UART')]
    md='# Jetson + Arduino UNO Q 배선 설계 v1\n\n'
    md+='**현장 보유 UNO Q의 선정 설계. 펌웨어 이식·실물 검증 전이며 현재 구동 명령이 아닙니다.**\n\n'
    md+='[확대 가능한 전체도](uno_q_wiring_v1.html) · [PNG](uno_q_all_v1.png) · [설계 JSON](uno_q_v1.json)\n\n'
    md+='UNO Q의 D/A는 실크 이름, Jetson J12는 BOARD 물리 번호입니다. 실제 위치·방향은 보드 실크로 확인합니다.\n\n'
    md+='| '+' | '.join(headers)+' |\n|'+'---|'*len(headers)+'\n'+'\n'.join('| '+' | '.join(row)+' |' for row in rows)+'\n\n'
    md+='## 전원과 공통 연결\n\n| 항목 | 연결 |\n|---|---|\n'+'\n'.join('| '+' | '.join(row)+' |' for row in extras)+'\n\n'
    md+='## 구현 및 통전 전 조건\n\n'+'\n'.join('- '+r for r in DATA['conditions'])+'\n\n'
    md+='## 근거\n\n'+'\n'.join('- ['+s['title']+']('+s['url']+')' for s in DATA['sources'])+'\n'
    (OUT/'uno_q_pin_table_v1.md').write_text(md,encoding='utf-8')
    table='<table><thead><tr>'+''.join('<th>'+esc(s)+'</th>' for s in headers)+'</tr></thead><tbody>'
    table+=''.join('<tr>'+''.join('<td>'+esc(v)+'</td>' for v in row)+'</tr>' for row in rows)+'</tbody></table>'
    extra='<table>'+''.join('<tr><th>'+esc(k)+'</th><td>'+esc(v)+'</td></tr>' for k,v in extras)+'</table>'
    source=(OUT/(NAME+'.svg')).read_text(encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IT ARENA · UNO Q 대체 배선도</title><style>body{margin:0;background:#edf2f8;color:#14273d;font-family:"Malgun Gothic",sans-serif}main{max-width:3000px;margin:auto;padding:25px}h1{font-size:30px}p,li{line-height:1.75}.notice{border-left:5px solid #ba3c36;background:#fff2e6;padding:16px}section{margin-top:25px;background:white;padding:22px;border:1px solid #ccd6e4;border-radius:9px}.scroll{overflow:auto}.drawing svg{display:block;width:100%;height:auto}button{padding:10px 16px;margin:8px;background:white;border:1px solid #b7c8da;border-radius:6px;cursor:pointer}a{color:#2161ae}table{width:100%;border-collapse:collapse;min-width:850px}td,th{padding:12px;border-bottom:1px solid #d8e0e9;text-align:left}th{background:#eaf0f7}@media print{@page{size:A3 portrait;margin:7mm}.toolbar,header{display:none}main{padding:0}section{break-before:page;border:0;padding:0}}</style><main><header><h1>Jetson + Arduino UNO Q · 전체 배선 설계</h1><p>현장 보유 UNO Q를 보조 MCU로 사용합니다. 엔코더·PWM·IMU는 STM32, 판단·카메라·라이다·USB 조향은 Jetson이 담당합니다.</p><p class="notice"><b>연결을 선정한 설계이며 펌웨어 이식과 실물 검증은 아직 하지 않았습니다.</b> 기존 M01 CH6의12 V를 UNO Q VIN에 전용 배정하고 운영 통신은3.3 V UART를 사용합니다. 물리 STOP은 사용자 최신 지시로 제외했습니다.</p><p><a href="uno_q_pin_table_v1.md">핀별 연결표</a> · <a href="uno_q_all_v1.png">PNG</a> · <a href="uno_q_all_v1.svg">SVG</a> · <a href="uno_q_v1.json">설계 JSON</a> · <a href="esp32_gpio_wiring_v1.html">기존 ESP32 도면</a> · <a href="jetson_existing_parts_check_2026_11_14.md">비교·검토 근거</a></p></header><section><div class="toolbar"><button onclick="zoom(false)">전체 보기</button><button onclick="zoom(true)">원본 크기</button><span>● 접속점 / 선의 흰 틈 교차는 미접속 / 실크 이름 기준</span></div><div class="scroll drawing">'''+source+'</div></section><section><h2>신호 연결표</h2><div class="scroll">'+table+'</div></section><section><h2>전원·공통 배선</h2><div class="scroll">'+extra+'</div></section><section><h2>구현·실물 확인 조건</h2><ul>'+''.join('<li>'+esc(r)+'</li>' for r in DATA['conditions'])+'</ul><h2>공식 근거</h2><ul>'+''.join('<li><a href="'+esc(s['url'],quote=True)+'">'+esc(s['title'])+'</a></li>' for s in DATA['sources'])+'</ul></section></main><script>function zoom(full){document.querySelector(".drawing svg").style.width=full?"3000px":"100%";}</script></html>'
    (OUT/'uno_q_wiring_v1.html').write_text(page,encoding='utf-8')
    print(f'Generated UNO Q schematic: {len(rows)} unique MCU signals. No hardware accessed.')


if __name__=='__main__':main()
