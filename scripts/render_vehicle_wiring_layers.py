"""STEP 층 배치를 유지한 케이블 이동도. 원본 CAD/하드웨어를 변경하지 않는다."""
import json

from render_esp32_wiring import Drawing, OUT, BLUE, GREEN, PURPLE, RED, GRAY, AMBER, INK


def main():
    data = json.loads((OUT / 'vehicle_wiring_layers_v1.json').read_text(encoding='utf-8'))
    harnesses = data['harnesses']
    power = [r for r in harnesses if r['kind'] == 'power_pair' and r['crosses_scan_plane']]
    usb = [r for r in harnesses if r['kind'] == 'usb' and r['crosses_scan_plane']]
    assert len(power) == 4 and sum(r['conductors'] for r in power) == 8
    crossing = [r for r in harnesses if r['crosses_scan_plane']]
    assert len(usb) == 2 and sum(r['cable_count'] for r in crossing) == 6
    d = Drawing('층별 케이블 배치도 · LiDAR 높이를 지나는 선',
                '11/12 STEP의 부품 층 유지 · 높이/가로 간격은 보기 좋게 펼침 · 배선 경로는 제안, 실측 통로 검증 아님', 2080, 2740)
    d.parts.append('<rect x="50" y="130" width="2640" height="65" rx="12" fill="#e6effa"/>')
    d.text(75,172,'광축 높이 통과: 전원 4쌍(8가닥) + USB 2개 = 연결 묶음 6개',29,BLUE,True)
    d.text(1570,171,'CF·STOP 추가로 늘어나는 층간 케이블: 0개',22,GREEN,True)

    def band(y,h,title,sub,fill):
        d.parts.append(f'<rect x="50" y="{y}" width="2140" height="{h}" rx="12" fill="{fill}" stroke="#cad4e0"/>')
        d.text(75,y+34,title,24,INK,True)
        d.text(75,y+65,sub,17,GRAY)

    band(230,250,'상부 2단 · M01 / 카메라','M01 Z193~224 mm · 카메라는 STEP의 구형 보드 위치에 B0390 적용', '#f2edf8')
    band(520,300,'상부 1단 · J01 / Jetson','J01 Z153~184 mm · Jetson Z153~188 mm', '#edf3fa')
    band(860,350,'중간 · LiDAR 층','C1 몸체 Z104.8~146.1 mm · 광축 추정 Z134.6 mm', '#fdf3e8')
    band(1250,620,'하부 · 제어 / 구동 / 배터리','서로 높이가 다른 받침을 포함한 광축 아래 공간. CF·STOP은 이 구역에 추가 배정.', '#edf6f0')

    # This plane is behind the components; seven colored crossings are marked last.
    d.parts.append('<path d="M65,1010 H2175" stroke="#c38a3a" stroke-width="2" stroke-dasharray="10 8" fill="none"/>')
    d.text(90,989,'광축 높이 134.6 mm',22,AMBER,True)
    d.text(90,1044,'이 점선을 지나는 케이블을 셉니다.',18,GRAY)

    d.box(100,305,650,150,'M01 · 최상단 모터 전원 보드','CN1 입력 / CN2(CH1) 모터 / CN6(CH5) 서보',PURPLE)
    d.box(1660,305,470,150,'B0390 카메라','현재 STEP의 카메라 장착층 유지',GREEN)
    d.box(100,590,650,190,'J01 · 센서 / Jetson 전원 보드','CN2(CH1) Jetson / CN5(CH4) 센서',BLUE)
    d.box(1510,590,620,190,'Jetson Orin Nano','DC IN · CSI · USB-A',BLUE)
    d.box(1700,950,400,120,'RPLIDAR C1','몸체는 광축 위아래에 걸침',AMBER)
    d.box(1510,1105,620,90,'C1 USB 어댑터 · 광축 아래 고정','STEP 미반영 · 기본 리드 끝 근처에 배치 제안', '#087f96',True)
    d.box(100,1330,650,100,'6S 배터리','공통 +/− 한 쌍만 상부 Y 분기까지 올림',RED)
    d.box(785,590,210,190,'Y 분기','상부 1단 · 2심',RED,True)
    d.box(100,1460,650,100,'MD20A → 주행 모터','MD20A Z97~119 mm · 모터 2선은 하부 내부',AMBER)
    d.box(100,1590,650,100,'Bus Servo Adapter A → ST3215-HS','어댑터 Z91~106 mm · 서보 전용 3선 1개는 하부 내부',PURPLE)
    d.box(100,1720,650,100,'T5V / TGND → 엔코더·CF','J01 5 V 전원 도착 후 하부에서 분기',GREEN)
    d.box(1510,1330,620,100,'ESP32-S3','STEP Z14~26 mm · USB 전원/데이터',BLUE)
    d.box(1510,1460,295,100,'IMU · 4438','STEP Z37~42 mm',GREEN)
    d.box(1835,1460,295,100,'CF-0108 · 추가','하부 ESP32 인근',AMBER,True)
    d.box(1510,1600,295,100,'STOP · 추가','하부 차체 측면',BLUE,True)
    d.box(1835,1600,295,100,'TGND · 5접점','CF·STOP 접지는 하부',GRAY)
    d.text(1530,1755,'IMU / CF / STOP / PWM·DIR / UART',22,INK,True)
    d.text(1530,1790,'모두 하부 안에서 연결 · 광축 높이 통과 없음',20,GREEN)
    d.text(1530,1830,'점선 부품: STEP에 없는 추가 배치 · 정확한 고정 위치는 후속',16,GRAY)

    def route(points,color,width=6):
        path='M'+' L'.join(f'{x},{y}' for x,y in points)
        d.parts.append(f'<path d="{path}" fill="none" stroke="#f6f8fc" stroke-width="{width+6}" stroke-linejoin="round"/>')
        d.parts.append(f'<path d="{path}" fill="none" stroke="{color}" stroke-width="{width}" stroke-linejoin="round"/>')
        for x,y in (points[0],points[-1]):
            d.parts.append(f'<circle cx="{x}" cy="{y}" r="5" fill="white" stroke="{color}" stroke-width="2"/>')

    points={
        'P1':[(750,1380),(900,1380),(900,730)],
        'P2':[(900,730),(960,670),(960,350),(750,350)],
        'P3':[(750,390),(1020,390),(1020,1500),(750,1500)],
        'P4':[(750,430),(1080,430),(1080,1630),(750,1630)],
        'P5':[(750,740),(770,740),(770,830),(1140,830),(1140,1760),(750,1760)],
        'U1':[(1700,780),(1700,800),(1200,800),(1200,1375),(1510,1375)],
        'U2':[(1850,780),(1850,835),(1260,835),(1260,1140),(1510,1140)]}
    for r in harnesses:
        route(points[r['id']],r['color'])
    # Explicit two-core Y: battery stem, J01 arm and M01 arm. + and - stay isolated.
    route([(900,730),(840,670),(750,670)],INK,6)
    d.parts.append(f'<circle cx="900" cy="730" r="7" fill="{INK}"/>')
    d.text(625,700,'CN1 입력',17,GRAY)
    d.text(750,810,'+끼리 / −끼리 분기',16,GRAY,anchor='end')
    d.text(805,507,'P2 · M01행 2심',17,'#8f3748')
    # Local cables are explicit and remain above or below the scan plane.
    route([(680,590),(680,550),(1450,550),(1450,680),(1510,680)],GRAY,7)
    d.text(1180,537,'19 V 배럴 1개 · 같은 층',17,GRAY)
    route([(1895,455),(1895,590)],GREEN,7)
    d.text(1920,506,'22↔22 FPC 1개',18,GREEN)
    d.text(1920,539,'광축 통과 없음',16,GREEN)
    route([(1900,1070),(1900,1105)], '#087f96',5)
    d.text(1940,1094,'C1 기본 리드 1개',15, '#087f96')
    d.text(1530,751,'USB 2개: U1은 ESP32 / U2는 C1 어댑터',18,BLUE)

    # Physical direction markers and scan-plane cable count.
    for r in crossing:
        x={'P1':900,'P3':1020,'P4':1080,'P5':1140,'U1':1200,'U2':1260}[r['id']]
        d.parts.append(f'<circle cx="{x}" cy="1010" r="23" fill="white" stroke="{r["color"]}" stroke-width="3"/>')
        d.text(x,1016,r['id'],17,r['color'],True,anchor='middle')
        d.parts.append(f'<circle cx="{x}" cy="1082" r="19" fill="#fdf3e8"/>')
        d.text(x,1092,'↑' if r['direction']=='위로' else ('↓' if r['direction']=='아래로' else '↕'),28,r['color'],anchor='middle')
    d.text(100,1180,'전원 1쌍 = +/− 2가닥 · USB 1개 = 완성 케이블 1본',17,GRAY)

    d.box(2230,230,460,850,'광축 높이의 연결 묶음 6개','화살표는 층간 연결 방향 · 신호 흐름과 구분',BLUE)
    for i,r in enumerate(crossing):
        y=345+100*i
        d.text(2250,y,r['id']+'  '+r['label'],18,r['color'],True)
        desc=('전원 2심 1쌍 · '+r['direction']) if r['kind']=='power_pair' else 'USB 케이블 1개 · 전원/데이터'
        d.text(2290,y+33,desc,17,GRAY)
    d.text(2250,965,'P2: Y → M01은 위층 사이에서만 연결',17,INK,True)
    d.text(2250,1005,'Y → J01은 같은 상부 1단에서 연결',17,GRAY)
    d.text(2250,1050,'6묶음의 굵기가 같다는 뜻은 아닙니다.',17,GRAY)
    d.box(2230,1120,460,750,'기둥별 분산 배선안','좌우는 차량 전방을 바라보는 기준',BLUE)
    rows=[('후방 왼쪽 · U1 + U2','USB 2개 · 나란히 고정'),
          ('후방 오른쪽 · P1 + P5','전원 2쌍 = 4가닥'),
          ('전방 오른쪽 · P3','전원 1쌍 = 2가닥'),
          ('전방 왼쪽 · P4','전원 1쌍 = 2가닥')]
    for i,(title,sub) in enumerate(rows):
        y=1240+i*100
        d.text(2250,y,title,20,INK,True);d.text(2250,y+32,sub,18,GRAY)
    d.note(2250,1665,['기둥의 앞/안쪽 면으로 세로 배선.',
                       '뒤·바깥으로 감아 외형을 늘리지 않음.',
                       '4×4.2 mm 홈에 전부 수용한다고',
                       '가정하지 않음. 선 외경·플러그·타이',
                       '공간은 현물로 확인해야 합니다.'],size=17,step=30)
    d.note(65,1930,['같은 연결 번호는 한 케이블/전원쌍입니다. 이 그림의 교차선은 서로 접속되지 않습니다.',
                    'Y 분기는 상부 1단 J01 근처. CF·STOP은 하부, C1 어댑터는 광축 아래. 기존 STEP 부품 층은 유지합니다.',
                    '상판 아래 횡단은 광축 위에서 고정하고 C1 회전부·Jetson 팬을 피합니다. 길이·외경·간섭은 미검증입니다.'],size=20,step=33)
    d.save('vehicle_wiring_layers_v1.svg')
    svg=(OUT/'vehicle_wiring_layers_v1.svg').read_text(encoding='utf-8')
    page='''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>IT ARENA · 층별 케이블 배치도</title>
<style>body{font-family:"Malgun Gothic",sans-serif;background:#eaf0f7;color:#14273d;margin:0}main{max-width:2740px;padding:24px;margin:auto}a{color:#2161ae}p{line-height:1.7}.drawing{overflow:auto;background:white;border-radius:12px}.drawing svg{width:100%;height:auto;min-width:1200px}button{padding:8px 14px;margin:8px;border:1px solid #afbdd0;background:white;border-radius:6px;cursor:pointer}@media print{@page{size:A3 landscape;margin:8mm}p,button,h1{display:none}main{padding:0}.drawing svg{min-width:0;max-height:275mm}}</style>
<main><h1>층별 케이블 배치도</h1><p>배터리에서 공통 2심을 올려 상부 1단 J01 근처에서 Y 분기합니다. LiDAR 광축 높이를 지나는 연결은 전원 4쌍 + USB 2개입니다. CF·STOP을 하부에 추가해도 층간 케이블은 늘지 않습니다.</p>
<p><a href="esp32_gpio_wiring_v1.html">전체 회로도</a> · <a href="vehicle_wiring_layers_v1.svg">SVG</a> · <a href="vehicle_wiring_layers_v1.png">PNG</a> · <a href="../WIRING_LAYERS_2026_11_13.md">층별 배치 설명</a></p>
<button onclick="document.querySelector('svg').style.width='100%'">전체 보기</button><button onclick="document.querySelector('svg').style.width='2740px'">원본 크기</button><div class="drawing">'''+svg+'''</div></main></html>'''
    (OUT/'vehicle_wiring_layers_v1.html').write_text(page,encoding='utf-8')
    print('Generated STEP-based layer diagram: 4 power pairs + 2 USB cables across scan plane; Y split on upper layer 1; CF/STOP add 0 crossings.')


if __name__ == '__main__':
    main()
