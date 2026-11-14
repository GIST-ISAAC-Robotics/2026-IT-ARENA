"""기존 ESP32 도면 양식의 Jetson 대체 설계안. 하드웨어 접근 없이 파일만 생성."""
from pathlib import Path
from html import escape as esc
import json
import xml.etree.ElementTree as ET
from render_esp32_wiring import Drawing, BLUE, GREEN, PURPLE, TEAL, RED, GRAY, AMBER, INK

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/hardware/schematics'
DATA = json.loads((OUT / 'jetson_direct_v1.json').read_text(encoding='utf-8'))
BASE = json.loads((OUT / 'esp32_gpio_v1.json').read_text(encoding='utf-8'))
NAME = 'jetson_direct_all_v1'


class Sheet(Drawing):
    def route(self, points, color=BLUE, width=3, dash=False):
        path = 'M' + ' L'.join(f'{x},{y}' for x,y in points)
        self.parts.append(f'<path d="{path}" fill="none" stroke="#f6f8fc" stroke-width="{width+6}" stroke-linejoin="round"/>')
        self.parts.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"' + (' stroke-dasharray="10 7"' if dash else '') + '/>')

    def dot(self, x, y, color):
        self.parts.append(f'<circle cx="{x}" cy="{y}" r="6" fill="{color}"/>')

    def horizontal(self, x1, x2, y, left, right, color, label='', dash=False):
        self.route([(x1,y),(x2,y)],color,dash=dash)
        self.pin(x1,y,left,'right',color,18)
        self.pin(x2,y,right,'left',color,18)
        if label:self.text((x1+x2)/2,y-14,label,16,color,anchor='middle')

    def save(self, name):
        self.text(50,self.height-28,'IT ARENA · 2026-11-14 · Jetson 직접 제어 v1 / 설계안 · BOARD=J12 물리 핀 · 통전·구동 미실행',17,GRAY)
        text='\n'.join(self.parts+['</svg>'])+'\n'
        ET.fromstring(text)
        (OUT/name).write_text(text,encoding='utf-8')


def unified():
    d=Sheet('ESP32 없는 전체 배선도 · Jetson 직접 제어안',
            '기존 전원·모터·서보·센서 재사용 / 신호 핀은 J12 BOARD 번호 / 실제 핀1 방향·pinmux 확인 전 연결하지 않음',2600,2740)
    d.text(50,145,'실선 = 연결 설계  |  점선 = 직결 보류/회로 없음  |  ● 접속점 · 점 없는 교차는 미접속',21,INK,True)
    d.text(1660,145,'전기적 검증 전 설계안 · 그대로 통전하는 완성 회로가 아닙니다',19,RED,True)
    d.box(1130,180,340,110,'6S 배터리','기존 XT60↔XT30 + Y 분기',RED)
    d.box(60,350,660,290,'PDB · J01','CN1 입력 21–36 V DC',RED)
    d.box(1880,350,660,290,'PDB · M01','CN1 입력 24–36 V DC',RED)
    d.box(1050,670,500,1430,'Jetson Orin Nano Super','J12 40핀 헤더 · BOARD 물리 번호',BLUE)
    d.box(340,710,380,155,'B0390 카메라','기존 22↔22 FPC',GREEN)
    d.box(1880,710,380,155,'RPLIDAR C1','기존 지급 USB 어댑터',TEAL)
    d.box(340,950,380,450,'Adafruit 4438','LSM6DSOX · SPI → I2C 변경안',GREEN)
    d.box(340,1470,380,320,'CF-0108 / TXS0108E','기존 기능 확인한 보드 · 채널1/2',AMBER)
    d.box(340,1890,380,240,'PGE-213 엔코더','모터축 · 5 V 측 유지',AMBER)
    d.box(1880,950,380,120,'MB4266-24180','기존 MA/MB 연결 유지',INK)
    d.box(1880,1140,380,290,'Cytron MD20A','3.3 V PWM/DIR · 20 kHz 후보',BLUE)
    d.box(1880,1500,380,120,'ST3215-HS × 1','기존 위치 교정값 보존',PURPLE)
    d.box(1880,1690,380,300,'Bus Servo Adapter(A)','USB-C 사용 · UART 헤더 미사용',PURPLE)

    # Thick paths denote two isolated conductors in one cable, never a short.
    for x,col,label in [(1260,RED,'+'),(1340,GRAY,'−')]:
        d.route([(x,290),(x,325)],col);d.text(x,278,label,20,col,anchor='middle')
    d.parts.append('<rect x="1240" y="325" width="120" height="35" rx="6" fill="#e8eef7" stroke="#576579"/>')
    d.text(1300,349,'+/− 2심',16,GRAY,anchor='middle')
    d.route([(1300,360),(1300,400)],INK,9)
    d.route([(1300,400),(1160,480),(810,480)],INK,9)
    d.route([(1300,400),(1440,480),(1790,480)],INK,9)
    d.dot(1300,400,INK)
    for x in (780,1790):
        d.parts.append(f'<rect x="{x}" y="438" width="30" height="85" rx="5" fill="#e8eef7" stroke="#576579"/>')
    for y,col,label in [(455,RED,'CN1 +'),(505,GRAY,'CN1 −')]:
        d.route([(720,y),(780,y)],col);d.pin(720,y,label,'right',col)
        d.route([(1820,y),(1880,y)],col);d.pin(1880,y,label,'left',col)
    d.text(1300,540,'Y 분기 유지 · +끼리 / −끼리',21,INK,True,anchor='middle')
    d.text(1300,588,'ESP32 본체·USB·전원·신호선은 모두 제거',20,RED,True,anchor='middle')
    d.note(80,455,['CN2(CH1): 19 V / 최대2.3 A','CN5(CH4): 5 V / 최대0.5 A',
                   '3.3 V CH5/6은 이 안에서 미사용'],size=18,step=48)
    d.note(1900,555,['CH1: 입력 전압 그대로 / CH5: 12 V·최대2 A'],size=16)

    # Jetson supply and unchanged camera/LiDAR links.
    for y,col,label in [(560,RED,'CN2 +'),(610,GRAY,'CN2 −')]:
        d.route([(720,y),(840,y)],col);d.pin(720,y,label,'right',col)
    d.parts.append('<rect x="840" y="545" width="50" height="80" rx="6" fill="#e8eef7" stroke="#576579"/>')
    d.route([(890,585),(950,585),(950,755),(1050,755)],GRAY,8)
    d.pin(1050,755,'DC IN 배럴잭','left',GRAY)
    d.text(1070,790,'19 V · 중앙 + / 외곽 −',17,GRAY)
    d.route([(720,825),(1050,825)],GREEN,7)
    d.pin(1050,825,'CSI','left',GREEN);d.text(885,809,'FPC 1개',17,GREEN,anchor='middle')
    d.route([(1550,825),(1880,825)],TEAL,7)
    d.pin(1550,825,'USB-A','right',TEAL);d.text(1715,809,'USB 1개',17,TEAL,anchor='middle')
    d.text(1070,910,'J12 신호는 3.3 V 기준',22,BLUE,True)

    # One 3V3 output feeds sensor VIN and CF VA/OE. Old MCU is absent.
    d.horizontal(720,1050,1040,'VIN','1 · 3V3',RED,'T3V3')
    d.horizontal(720,1050,1090,'GND','6 · GND',GRAY)
    d.horizontal(720,1050,1170,'SDA','3 · I2C SDA',GREEN)
    d.horizontal(720,1050,1230,'SCL','5 · I2C SCL',GREEN)
    d.route([(875,1040),(875,1555),(720,1555)],RED)
    d.route([(720,1600),(875,1600),(875,1555)],RED)
    d.pin(720,1555,'VA','right',RED);d.pin(720,1600,'OE','right',RED)
    for x,y in [(875,1040),(875,1555)]:d.dot(x,y,RED)
    d.note(360,1295,['하단 CS → T3V3 (I2C 모드)',
                    '하단 DO 미연결 · 0x6A/0x6B 확인',
                    '3Vo·I1/I2·상단 보조핀 미연결'],size=17,step=29)
    d.note(1070,1320,['J12 3.3 V 공급: 합계100 mA 이내',
                     'IMU + CF + 추가 회로 부하 합산',
                     'PDB 3.3/5 V와 병렬 연결 금지'],size=17,step=32)

    # Dedicated 5V and reference ground distribution, kept distinct from power returns.
    for x,label,col in [(160,'+',RED),(240,'−',GRAY)]:
        d.route([(x,640),(x,2050 if x==240 else 1980)],col)
        d.text(x,677,label,16,col,anchor='middle')
    d.text(200,720,'CN5(CH4)',17,GRAY,anchor='middle')
    for y,label in [(1555,'VB'),(1980,'1 VCC')]:
        d.route([(160,y),(340,y)],RED);d.pin(340,y,label,'left',RED)
    for y,label in [(1605,'GND'),(2050,'2 GND')]:
        d.route([(240,y),(340,y)],GRAY);d.pin(340,y,label,'left',GRAY)
    d.route([(240,1835),(1050,1835)],GRAY);d.pin(1050,1835,'9 · GND','left',GRAY)
    for x,y,col in [(160,1555,RED),(240,1605,GRAY),(240,1835,GRAY)]:d.dot(x,y,col)
    d.text(55,1485,'T5V · 5 V',18,RED,True)
    d.text(270,1818,'TGND · 센서 공통',18,GRAY,True)
    d.horizontal(720,1050,1680,'A1','31 · ENC A',AMBER,'TXS→TXB 구동 조건',True)
    d.horizontal(720,1050,1740,'A2','33 · ENC B',AMBER,'충족 근거 없음 · 보류',True)
    for x,cf,enc in [(510,'B1','3 HA'),(625,'B2','4 HB')]:
        d.route([(x,1790),(x,1890)],AMBER)
        d.text(x,1770,cf,18,AMBER,anchor='middle')
        d.text(x+15,1870,enc,18,AMBER)
    d.text(360,2110,'5 V HA/HB를 J12에 직접 연결 금지',17,RED)

    # Motor high-current paths return to M01, independently of signal ground.
    for y,left,right,label in [(1240,'32 · PWM','PWM','하드웨어 PWM'),(1300,'29 · DIR','DIR','방향'),(1360,'25 · GND','신호 GND','')]:
        d.horizontal(1550,1880,y,left,right,GRAY if not label else BLUE,label)
    for x,driver,motor in [(1980,'MA','6 (+)'),(2160,'MB','5 (−)')]:
        d.route([(x,1070),(x,1140)],INK,5)
        d.text(x,1050,motor,18,INK,anchor='middle');d.text(x+12,1112,driver,18,INK)
    d.text(1900,1408,'PWM LOW = brake · MA/MB ≠ GND',17,GRAY)
    for sy,lane,dy,col,pdb,load,label in [
        (430,2660,1240,RED,'CN2(CH1) +','VB+','배터리 전압'),
        (480,2700,1300,GRAY,'CN2(CH1) −','VB−','전력 귀환'),
        (560,2580,1800,RED,'CN6(CH5) +','전원 +','12 V / 최대2 A'),
        (610,2620,1860,GRAY,'CN6(CH5) −','전원 −','전력 귀환')]:
        d.route([(2540,sy),(lane,sy),(lane,dy),(2260,dy)],col)
        d.pin(2540,sy,pdb,'right',col);d.pin(2260,dy,load,'right',col)
        d.text(2400,dy-15,label,17,col,anchor='middle')

    # USB control avoids the unselected J12 UART loading problem.
    d.route([(2070,1620),(2070,1690)],PURPLE,9)
    d.text(2100,1664,'전용 3선 케이블 1개',17,PURPLE)
    d.horizontal(1550,1880,1800,'USB-C Host','USB-C',PURPLE,'C-to-C · 데이터',False)
    d.note(1900,1860,['점퍼캡 2개: B (USB)',
                     'UART TX/RX/GND: 미연결',
                     'USB 등록·직접 통신은 아직 미시험'],size=17,step=36)
    d.note(1890,2060,['J12 8/10/14 핀은 조향에 사용하지 않음',
                     'USB 통신 / 서보 12 V는 기존 별도 전원',
                     '모터·서보 전력 −는 M01로 직접 귀환'],size=18,step=33)
    d.note(1070,1500,['핀번호는 ESP32 GPIO가 아닙니다.',
                     '32: PWM / 29: 출력',
                     '31·33: GPIO 입력 후보',
                     '서보: ID1 / 1 Mbps / 8N1'],size=18,step=34)
    d.note(1070,1980,['서보: USB 직렬 포트로 직접 통신',
                     'DC·IMU 핀 설정 시 기존 CSI 구성 보존'],size=17,step=32)

    # Physical STOP is excluded by the user's latest scope decision.
    d.box(340,2230,380,150,'물리 STOP · 사용자 제외','연결 없음 · 핀15 사용하지 않음',GRAY)
    d.box(1050,2190,500,190,'나머지 기능은 모두 유지','서보 · DC · 엔코더 · IMU · 카메라 · 라이다',BLUE)
    d.note(1070,2305,['엔코더 27/28 우회도 미채택',
                     'USB-C·전력 감시 I2C 공유 / LOW 여유 문제'],size=17,step=31)
    d.note(1650,2240,['구동 명령 만료 시 PWM 0 처리 필요',
                     'OS 정지 시 독립 차단은 구현되지 않음',
                     'Jetson 전원 OFF · 실제 통신/출력 미시험'],RED,19,35)
    d.note(50,2450,['미완료: 엔코더 TXS→TXB 입력 정합 · pinmux · PWM 종료 / 조향은 USB 경로 채택 / 물리 STOP 제외',
                    'J12 I2C 외 외부 pull-up/down은 >50 kΩ 조건. 저항만 추가해 동작·정지를 보장할 수 없습니다.',
                    '배터리 Y의 각 가지는 +/− 2심이며 +/−를 묶지 않습니다. 기존 M01 입력24 V 미만 운용 가능성은 별도 미해결입니다.'],size=18,step=30)
    d.save(NAME+'.svg')


def tables():
    signal=[(str(r['pin']),r['net'],r['function'],r['endpoint'],r['status']) for r in DATA['signals']]
    power=[(str(r['pin']),r['destination'],r['role']) for r in DATA['headers']]
    usb=[(r['name'],r['host'],r['device'],r['cable'],r['status']) for r in DATA['usb']]
    pdb=[]
    for name,board in BASE['pdb_boards'].items():
        for row in board['outputs']:
            pdb.append((name,f"CH{row['ch']} / CN{row['cn']}",row['voltage'],row['current'],row['load']))
    return [('USB 연결',['기능','호스트','장치','케이블','조건 / 현재 상태'],usb),
            ('J12 신호 연결표',['BOARD 핀','net','기능','연결 대상','미확인 / 변경'],signal),
            ('J12 전원·접지',['BOARD 핀','연결 대상','역할'],power),
            ('기존 PDB 채널 유지',['보드','채널 / 단자','전압','한도','대상'],pdb)]


def validate():
    pins=[r['pin'] for r in DATA['signals']]+[r['pin'] for r in DATA['headers']]
    assert len(pins)==len(set(pins)) and all(1<=p<=40 for p in pins)
    sig={r['net']:r['pin'] for r in DATA['signals']}
    assert sig['IMU_SDA']==3 and sig['IMU_SCL']==5
    assert sig['MOTOR_PWM']==32 and sig['MOTOR_DIR']==29
    assert {sig['ENC_A_3V3'],sig['ENC_B_3V3']}=={31,33}
    assert 'STOP_REQUEST' not in sig and 15 not in pins
    assert DATA['three_v3_budget_ma']==100
    assert not any(r['pin'] in {2,4,17,27,28} for r in DATA['signals'])


def main():
    validate();unified()
    md='# ESP32 없는 Jetson 직접 제어 배선안 v1\n\n'
    md+='**추가 부품 없는 연결 후보: 실제 pinmux·신호 정합·전원·정지 검증 전이며 통전 지시가 아닙니다.** [현재 검토 결과와 실측 대기](jetson_existing_parts_check_2026_11_14.md).\n\n'
    md+='[한 장 배선도](jetson_direct_wiring_v1.html) · [SVG](jetson_direct_all_v1.svg) · [기존 ESP32 도면](esp32_gpio_wiring_v1.html)\n\n'
    md+=DATA['numbering']+'입니다. ESP32 GPIO 번호와 혼동하지 않습니다.\n\n'
    html_tables=''
    for title,heads,rows in tables():
        md+='## '+title+'\n\n| '+' | '.join(heads)+' |\n|'+'---|'*len(heads)+'\n'
        md+='\n'.join('| '+' | '.join(row)+' |' for row in rows)+'\n\n'
        html_tables+='<section><h2>'+esc(title)+'</h2><div class="scroll"><table><thead><tr>'+''.join('<th>'+esc(h)+'</th>' for h in heads)+'</tr></thead><tbody>'+''.join('<tr>'+''.join('<td>'+esc(c)+'</td>' for c in row)+'</tr>' for row in rows)+'</tbody></table></div></section>'
    md+='## IMU 국소 연결\n\n'+'\n'.join('- '+s for s in DATA['imu_local'])+'\n\n'
    md+='## 실물 연결 전 해결할 항목\n\n'+'\n'.join('- '+s for s in DATA['holds'])+'\n\n'
    md+='I2C 선택 이유: Adafruit4438 SCL/SDA의10kΩ 풀업을 TXB0108 경유 SPI에 그대로 옮기지 않고, J12의 I2C 전용 핀을 사용합니다. 기존 SPI 앱은 직접 재사용하지 않습니다.\n\n'
    md+='물리 STOP은 사용자의 최신 지시로 제외했습니다. 핀15와 스위치 배선은 사용하지 않습니다. 나머지 여섯 기능은 유지하며 명령 만료 시 PWM0 처리가 필요합니다. OS 정지에 독립적인 모터 전력 차단은 구현되지 않았습니다.\n\n'
    md+='조향은USB-C로 연결하므로 미채택J12 UART안의풀업 조건 문제를 피합니다. CF의TXS 내부풀업·지속 구동으로Jetson TXB의±2mA 요구 충족을 보증할 수 없어 엔코더 직결은 미채택입니다. 상세 공식 근거는 위 현재 검토 결과에 기록했습니다.\n\n'
    md+='## 근거\n\n'+'\n'.join('- ['+s['title']+']('+s['url']+')' for s in DATA['sources'])+'\n\n'
    md+='NVIDIA carrier specification v1.3 §3.3/Table3-3/Table5-3은 공식 검색 색인 원문으로 확인했습니다. 직접 PDF 열기는403이었고 실제 설치 pinmux는 SSH 시간 초과로 미확인입니다. J12의 핀당1A 커넥터 정격과3.3V 공급합계100mA 예산을 구분합니다.\n\n'
    md+='생성: `python scripts/render_jetson_direct_wiring.py`. 설계 원본: `jetson_direct_v1.json`. 기존 ESP32 도면 파일은 덮어쓰지 않습니다.\n'
    (OUT/'jetson_direct_pin_table_v1.md').write_text(md,encoding='utf-8')
    refs=''.join('<li><a href="'+esc(s['url'],quote=True)+'">'+esc(s['title'])+'</a></li>' for s in DATA['sources'])
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IT ARENA · ESP32 없는 Jetson 배선안</title>
<style>body{margin:0;background:#eaf0f7;color:#14273d;font-family:"Malgun Gothic",sans-serif}main{max-width:2740px;margin:auto;padding:28px}h1{font-size:30px}p,li{line-height:1.7}section{margin:24px 0;background:white;border:1px solid #ccd6e4;border-radius:12px;padding:20px}.scroll{overflow:auto}.drawing svg{width:100%;height:auto}.toolbar{display:flex;flex-wrap:wrap;gap:12px;align-items:center;margin:16px 0}button{padding:10px 16px;cursor:pointer;border:1px solid #b7c8da;background:white;border-radius:6px}a{color:#2161ae}table{border-collapse:collapse;width:100%;min-width:800px;font-size:15px}td,th{padding:12px;border-bottom:1px solid #d8e0e9;text-align:left}th{background:#eaf0f7}.notice{padding:16px 20px;background:#fff1de;border-left:5px solid #a86800}.changes{display:grid;grid-template-columns:repeat(auto-fit,minmax(260px,1fr));gap:20px}.changes article{border:1px solid #ccd6e4;padding:16px;border-radius:8px}@media print{@page{size:A3 portrait;margin:8mm}main{padding:0}section{break-before:page;border:0;padding:0}.toolbar{display:none}.drawing svg{width:100%}header,.changes{display:none}}</style>
<main><header><h1>ESP32 없는 Jetson 직접 제어 배선안</h1><p>2026-11-14 · 기존 배선도와 같은 부품 배치·색상 · J12 BOARD 물리 핀 번호</p><p class="notice"><b>설계안 / 그대로 통전하지 마십시오.</b> 조향 USB 경로와 물리 STOP 제외를 반영했습니다. 엔코더 입력 정합, pinmux와 PWM 종료는 미검증입니다. 발열 ESP32와 그 USB·전원·신호는 완전히 분리하는 구성입니다.</p>
<p><b>공식 자료 대조:</b> 조향은 Jetson USB-C→데이터 C-to-C→Adapter(A), 점퍼캡 2개 B로 연결합니다. CF의 TXS→Jetson TXB 경로는 입력 구동 요구 충족 근거가 없어 직결 보류입니다. 물리 STOP은 사용자 지시로 제외했고 나머지 기능은 모두 유지합니다. <a href="jetson_existing_parts_check_2026_11_14.md">공식 근거와 판단</a></p>
<p><a href="esp32_gpio_wiring_v1.html">기존 ESP32 도면</a> · <a href="jetson_direct_pin_table_v1.md">핀별 연결표</a> · <a href="jetson_direct_v1.json">설계 JSON</a> · <a href="jetson_direct_all_v1.png">PNG</a> · <a href="jetson_direct_all_v1.svg">SVG 원본</a></p></header>
<div class="changes"><article><b>유지</b><p>J01/M01 전원, MD20A, 기존 서보 어댑터, 엔코더/CF, CSI 카메라와 USB 라이다</p></article><article><b>변경</b><p>ESP32 연결 제거. 조향 USB 및 Jetson PWM/GPIO 사용. IMU는 SPI에서 I2C로 변경하며 물리 STOP은 제외합니다.</p></article><article><b>미확정</b><p>점선은 엔코더 입력 정합 미완료 경로입니다. 27/28 우회도 미채택이며 Jetson 전원 OFF로 실물 검증은 하지 않았습니다.</p></article></div>
<section><h2>신호·전원·분기 전체도</h2><div class="toolbar"><button onclick="zoomDiagram(false)">전체 보기</button><button onclick="zoomDiagram(true)">원본 크기</button><span>● 있는 곳만 접속 · BOARD는 ESP32 GPIO 번호가 아닙니다.</span></div><div class="scroll drawing">'''
    page+=(OUT/(NAME+'.svg')).read_text(encoding='utf-8')+'</div></section>'+html_tables
    page+='<section><h2>확인 범위와 근거</h2><ul>'+''.join('<li>'+esc(s)+'</li>' for s in DATA['holds'])+'</ul><p>IMU 하단 CS→3.3V, DO 미연결로 기존 주소 선택을 유지하고0x6A/0x6B를 확인합니다. 주소 SJ1이 닫힌 보드에서 DO를 GND에 연결하면3.3V 단락 위험이 있습니다. 3Vo와 보조 핀은 사용하지 않습니다. 핀배정·문서 검토 결과이며 실제 통신·구동 성공이 아닙니다.</p><ul>'+refs+'</ul></section></main><script>function zoomDiagram(full){document.querySelector(".drawing svg").style.width=full?"2740px":"100%";}</script></html>'
    (OUT/'jetson_direct_wiring_v1.html').write_text(page,encoding='utf-8')
    print(f'Generated USB-servo design; {len(DATA["signals"])} unique signal pins, {len(DATA["headers"])} power/GND pins. Hardware untouched.')


if __name__=='__main__':
    main()
